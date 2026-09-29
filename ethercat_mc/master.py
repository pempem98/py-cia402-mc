"""EtherCAT master: bus bring-up and the single cyclic process-data loop.

One master owns the whole bus. There is exactly one process-data exchange per
cycle for all slaves together, so `send_processdata`/`receive_processdata` live
here and nowhere else. Axes never touch the wire: they publish their outputs
into a buffer and read their inputs from one, and this loop moves both.
"""
from __future__ import annotations

import logging
import sys
import threading
import time
from typing import Callable, Sequence

import pysoem

from .config import BusConfig

log = logging.getLogger(__name__)


class BusError(RuntimeError):
    """Raised when the bus cannot be brought up or has faulted."""


def _text(value: str | bytes) -> str:
    """pysoem returns adapter fields as bytes on some platforms."""
    if isinstance(value, bytes):
        return value.decode(errors="replace")
    return value


def find_adapters() -> list[tuple[str, str]]:
    """Return [(name, description)] for every usable network adapter."""
    return [
        (_text(a.name), _text(a.desc)) for a in pysoem.find_adapters()
    ]


def _raise_thread_priority() -> None:
    """Give the calling thread time-critical priority on Windows.

    At normal priority the cyclic thread was observed to miss 30-50 ms at a
    time, long enough for the drives to drop out of sync. Linux users should
    run under SCHED_FIFO instead (e.g. `chrt -f 80`).
    """
    if sys.platform != "win32":
        return
    import ctypes

    THREAD_PRIORITY_TIME_CRITICAL = 15
    handle = ctypes.windll.kernel32.GetCurrentThread()
    if not ctypes.windll.kernel32.SetThreadPriority(
        handle, THREAD_PRIORITY_TIME_CRITICAL
    ):
        log.warning("Could not raise cyclic thread priority")


def _boost_timer_resolution() -> Callable[[], None]:
    """Ask Windows for 1 ms timer resolution; return a function to undo it.

    Without this, `time.sleep` granularity is ~15.6 ms, which makes any cycle
    time below that meaningless.
    """
    if sys.platform != "win32":
        return lambda: None
    import ctypes

    ctypes.windll.winmm.timeBeginPeriod(1)
    return lambda: ctypes.windll.winmm.timeEndPeriod(1)


class CyclicTask:
    """Runs the process-data loop in its own thread at a fixed period.

    Each cycle: collect outputs from every axis, exchange process data, hand
    the inputs back to every axis. `on_cycle` is called once per cycle after
    the inputs are distributed, with the elapsed time in seconds.
    """

    def __init__(
        self,
        master: pysoem.Master,
        cycle_time_s: float,
        pdo_timeout_us: int = 2000,
        max_wkc_errors: int = 50,
    ):
        self._master = master
        self.cycle_time_s = cycle_time_s
        self._pdo_timeout_us = pdo_timeout_us
        self._max_wkc_errors = max_wkc_errors

        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._callbacks: list[Callable[[float], None]] = []

        self.expected_wkc = 0
        self.actual_wkc = 0
        self.wkc_errors = 0
        self.cycle_count = 0
        #: Worst observed gap between cycles, for diagnosing jitter.
        self.max_jitter_s = 0.0
        self.last_error: BaseException | None = None
        #: Set when the loop gives up; the application should stop.
        self.faulted = threading.Event()
        #: The working counter only reaches `expected_wkc` once every slave is
        #: in OP; in SAFE-OP outputs are not counted. Monitoring starts when
        #: the master calls arm_wkc_check() after the OP transition.
        self._wkc_armed = False
        #: Cycles that took more than twice the period, for diagnostics.
        self.late_cycles = 0

    def arm_wkc_check(self) -> None:
        """Start treating a low working counter as a bus fault."""
        self.wkc_errors = 0
        self.max_jitter_s = 0.0
        self.late_cycles = 0
        self._wkc_armed = True

    def add_callback(self, fn: Callable[[float], None]) -> None:
        """Register a per-cycle callback. Called from the cyclic thread, so it
        must not block: no I/O, no locks held across cycles."""
        self._callbacks.append(fn)

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("cyclic task already started")
        self.expected_wkc = self._master.expected_wkc
        log.info("Starting cyclic task: %.1f ms, expected WKC %d",
                 self.cycle_time_s * 1e3, self.expected_wkc)
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="ethercat-cyclic", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _run(self) -> None:
        restore_timer = _boost_timer_resolution()
        _raise_thread_priority()
        period = self.cycle_time_s
        next_wake = time.perf_counter()
        last = next_wake
        try:
            while not self._stop.is_set():
                now = time.perf_counter()
                dt = now - last
                last = now
                if self.cycle_count > 0:
                    self.max_jitter_s = max(self.max_jitter_s, abs(dt - period))
                    if dt > 2 * period:
                        self.late_cycles += 1

                self._master.send_processdata()
                self.actual_wkc = self._master.receive_processdata(
                    timeout=self._pdo_timeout_us
                )

                if not self._wkc_armed:
                    pass
                elif self.actual_wkc < self.expected_wkc:
                    self.wkc_errors += 1
                    if self.wkc_errors >= self._max_wkc_errors:
                        raise BusError(
                            f"working counter {self.actual_wkc} < expected "
                            f"{self.expected_wkc} for {self.wkc_errors} cycles"
                        )
                elif self.wkc_errors:
                    # A healthy cycle clears the streak; only a sustained run
                    # of bad counters means the bus is really broken.
                    self.wkc_errors = 0

                for fn in self._callbacks:
                    fn(dt)
                self.cycle_count += 1

                # Absolute schedule: sleeping for a fixed period would let the
                # work time accumulate as drift.
                next_wake += period
                slack = next_wake - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
                elif slack < -period:
                    # Fell behind by more than a full cycle: resynchronise
                    # rather than trying to catch up with back-to-back frames.
                    next_wake = time.perf_counter()
        except BaseException as exc:  # noqa: BLE001 - reported via faulted
            self.last_error = exc
            self.faulted.set()
            log.exception("Cyclic task stopped: %s", exc)
        finally:
            restore_timer()


class EtherCATMaster:
    """Owns the pysoem master, the slaves and the cyclic task."""

    def __init__(self, cfg: BusConfig):
        self.cfg = cfg
        self.master = pysoem.Master()
        # Release the GIL inside every blocking pysoem call. Without this, a
        # 50 ms state_check or a slow SDO read on the application thread
        # starves the cyclic thread; the drives then see irregular process
        # data (measured 31 ms gaps) and refuse the SAFE-OP -> OP transition.
        self.master.always_release_gil = True
        self.task: CyclicTask | None = None
        self._open = False
        self.slave_count = 0

    # --- bring-up --------------------------------------------------------
    def open(self, adapter: str | None = None) -> None:
        name = adapter or self.cfg.adapter
        if not name:
            raise BusError("no network adapter configured")
        self.master.open(name)
        self._open = True
        log.info("Opened adapter %s", name)

    def scan(self) -> int:
        """Enumerate slaves; the bus ends up in PRE-OP. Returns the count."""
        count = self.master.config_init()
        if count <= 0:
            raise BusError(
                "no EtherCAT slaves found - check power, cabling (into the IN "
                "port) and the link LEDs"
            )
        self.slave_count = count
        for i, s in enumerate(self.master.slaves):
            log.info("Slave %d: %s vendor=0x%08X product=0x%08X rev=0x%08X",
                     i, s.name, s.man, s.id, s.rev)
        return count

    def mailbox_health(self, reads: int = 50) -> list[tuple[int, int, int]]:
        """Read the statusword over SDO `reads` times per slave.

        Returns [(slave, ok, total)]. Run in PRE-OP, before configuration: a
        healthy link answers every request, so any loss here means something
        else is on the wire - typically a second master (TwinCAT) holding the
        same segment - and PDO configuration would fail with a WkcError.
        """
        results = []
        for i, s in enumerate(self.master.slaves):
            ok = 0
            for _ in range(reads):
                try:
                    s.sdo_read(0x6041, 0)
                    ok += 1
                except Exception:  # noqa: BLE001 - counting failures is the point
                    pass
            results.append((i, ok, reads))
        return results

    def check_mailbox_health(self, reads: int = 50, min_ratio: float = 0.95) -> None:
        """Raise BusError if any slave drops too many mailbox requests."""
        bad = []
        for i, ok, total in self.mailbox_health(reads):
            log.info("Slave %d mailbox: %d/%d replies", i, ok, total)
            if ok < min_ratio * total:
                bad.append(f"slave {i} answered {ok}/{total}")
        if bad:
            raise BusError(
                "unreliable mailbox communication (" + "; ".join(bad) + "). "
                "The drives are not the likely cause. Check that no other "
                "EtherCAT master uses this adapter: switch TwinCAT to Stop "
                "(or stop the TwinCAT3 System Service) and close any TwinCAT "
                "XAE online view. Run `python -m ethercat_mc.cli linktest "
                "--adapter ...` to re-measure."
            )

    def configure(self, config_funcs: Sequence[Callable[[int], None] | None]) -> None:
        """Attach each slave's PRE-OP configuration hook and map the PDOs.

        `config_funcs[i]` is called by SOEM while slave i is in PRE-OP, which
        is the only state where the PDO assignment objects may be written.
        """
        for i, fn in enumerate(config_funcs):
            if fn is not None:
                self.master.slaves[i].config_func = fn
        io_size = self.master.config_map()
        log.info("Process image: %d bytes", io_size)

        if self.cfg.use_dc:
            # config_dc must run after config_map and before OP.
            self.master.config_dc()
            log.info("Distributed clocks configured")

        if self.master.state_check(pysoem.SAFEOP_STATE, 50_000) != pysoem.SAFEOP_STATE:
            self.master.read_state()
            details = "; ".join(
                f"slave {i} ({s.name}): {pysoem.al_status_code_to_string(s.al_status)}"
                for i, s in enumerate(self.master.slaves)
                if s.state != pysoem.SAFEOP_STATE
            )
            raise BusError(f"bus did not reach SAFE-OP: {details}")
        log.info("Bus in SAFE-OP")

    def start_cyclic(self) -> CyclicTask:
        """Start the process-data loop. Must run before requesting OP: slaves
        drop out of OP if process data stops arriving."""
        self.task = CyclicTask(
            self.master,
            self.cfg.cycle_time_s,
            self.cfg.pdo_timeout_us,
            self.cfg.max_wkc_errors,
        )
        self.task.start()
        return self.task

    def go_operational(self, timeout_s: float = 5.0) -> None:
        """Request OP and wait for every slave to get there."""
        if self.task is None or not self.task.running:
            raise BusError("start the cyclic task before requesting OP")

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            # Re-issue the request every pass, per slave: a slave that missed
            # the first request (seen on eRob) otherwise sits in SAFE-OP with
            # no AL error until the timeout.
            self.master.read_state()
            for s in self.master.slaves:
                if s.state & pysoem.STATE_ERROR:
                    # e.g. SAFE-OP + error after a watchdog trip: the error
                    # must be acknowledged before the slave accepts OP.
                    log.warning("Acknowledging AL error on %s: %s", s.name,
                                pysoem.al_status_code_to_string(s.al_status))
                    s.state = (s.state & 0x0F) | pysoem.STATE_ACK
                    s.write_state()
                elif s.state != pysoem.OP_STATE:
                    s.state = pysoem.OP_STATE
                    s.write_state()
            if all(s.state == pysoem.OP_STATE for s in self.master.slaves):
                self.task.arm_wkc_check()
                log.info("Bus in OP")
                return
            if self.task.faulted.is_set():
                raise BusError(f"cyclic task failed: {self.task.last_error}")
            # Poll with a single-frame read_state() and a plain sleep, never a
            # blocking state_check(): that call held the cyclic thread off the
            # wire for its whole timeout (the drives measured 50 ms cycles)
            # and they then refused OP.
            time.sleep(0.02)
        self.master.read_state()
        details = "; ".join(
            f"slave {i} ({s.name}): state={s.state} "
            f"{pysoem.al_status_code_to_string(s.al_status)}"
            for i, s in enumerate(self.master.slaves)
            if s.state != pysoem.OP_STATE
        )
        raise BusError(f"bus did not reach OP: {details}")

    # --- teardown --------------------------------------------------------
    def close(self) -> None:
        """Stop the loop and put the bus back in INIT. Safe to call twice."""
        if self.task is not None:
            self.task.stop()
            self.task = None
        if self._open:
            try:
                self.master.state = pysoem.INIT_STATE
                self.master.write_state()
            except Exception:  # noqa: BLE001 - closing must not raise
                log.warning("Could not set INIT state on close", exc_info=True)
            try:
                self.master.close()
            finally:
                self._open = False
            log.info("Bus closed")

    def __enter__(self) -> "EtherCATMaster":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
