"""
eRob: quay theo goc nhap tay qua EtherCAT (pysoem).
- Thu che do Profile Position (PP). Neu eRob khong chuyen sang PP,
  tu dong dung CSP: Python tu nhich target position moi chu ky.
- Chay voi quyen Administrator (Windows) hoac sudo (Linux).
- KEP CHAT KHOP, chua gan tai. Ctrl+C de dung.
"""
import struct
import sys
import threading
import time

import pysoem

COUNTS_PER_REV = 524288      # encoder dau ra 19-bit -> kiem tra lai manual eRob70
VEL_DEG_S = 10.0             # toc do mac dinh (do/s)
ACC_DEG_S2 = 20.0            # gia toc PP (do/s^2)
MAX_STEP_DEG = 360.0         # gioi han goc moi lenh
MAX_VEL_DEG_S = 30.0         # gioi han toc do
TOL_DEG = 0.05               # sai so chap nhan "toi dich"
MODE = "csp"                 # "csp" (Python tu tao quy dao) hoac "pp" (drive tu chay)


def deg2cnt(deg):
    return int(round(deg * COUNTS_PER_REV / 360.0))


def cnt2deg(cnt):
    return cnt * 360.0 / COUNTS_PER_REV


class Drive:
    def __init__(self, master, slave):
        self.master = master
        self.slave = slave
        self.cw = 0
        self.target = 0          # gia tri gui vao 0x607A
        self.goal = 0            # dich cuoi (dung cho CSP)
        self.vel_cnt = deg2cnt(VEL_DEG_S)
        self.acc_cnt = deg2cnt(ACC_DEG_S2)
        self.target_f = 0.0      # target dang float cho CSP
        self.v = 0.0             # toc do hien tai cua quy dao (counts/s)
        self.csp = False
        self.status = 0
        self.pos = 0
        self.running = True

    def loop(self):
        last = time.perf_counter()
        while self.running:
            now = time.perf_counter()
            dt = min(now - last, 0.02)
            last = now
            if self.csp:                     # CSP: quy dao hinh thang ve phia goal
                diff = self.goal - self.target_f
                dist = abs(diff)
                if dist < 1:
                    self.target_f = float(self.goal)
                    self.v = 0.0
                else:
                    stop_dist = self.v * self.v / (2 * self.acc_cnt)
                    if dist <= stop_dist:
                        self.v = max(self.v - self.acc_cnt * dt, self.acc_cnt * dt)
                    else:
                        self.v = min(self.v + self.acc_cnt * dt, self.vel_cnt)
                    step = self.v * dt
                    if step >= dist:
                        self.target_f = float(self.goal)
                        self.v = 0.0
                    else:
                        self.target_f += step if diff > 0 else -step
                self.target = int(round(self.target_f))
            self.slave.output = struct.pack("<Hi", self.cw, int(self.target))
            self.master.send_processdata()
            self.master.receive_processdata(2000)
            data = self.slave.input
            if len(data) >= 6:
                self.status, self.pos = struct.unpack("<Hi", data[:6])
            time.sleep(0.001)


def sdo_w(slave, idx, sub, fmt, val):
    slave.sdo_write(idx, sub, struct.pack(fmt, val))


def sdo_r(slave, idx, sub, fmt):
    data = slave.sdo_read(idx, sub)
    return struct.unpack(fmt, data[:struct.calcsize(fmt)])[0]


def make_config(master):
    def setup(pos):
        s = master.slaves[pos]
        # RxPDO: Controlword + Target position
        sdo_w(s, 0x1C12, 0, "<B", 0)
        sdo_w(s, 0x1600, 0, "<B", 0)
        sdo_w(s, 0x1600, 1, "<I", 0x60400010)
        sdo_w(s, 0x1600, 2, "<I", 0x607A0020)
        sdo_w(s, 0x1600, 0, "<B", 2)
        sdo_w(s, 0x1C12, 1, "<H", 0x1600)
        sdo_w(s, 0x1C12, 0, "<B", 1)
        # TxPDO: Statusword + Position actual
        sdo_w(s, 0x1C13, 0, "<B", 0)
        sdo_w(s, 0x1A00, 0, "<B", 0)
        sdo_w(s, 0x1A00, 1, "<I", 0x60410010)
        sdo_w(s, 0x1A00, 2, "<I", 0x60640020)
        sdo_w(s, 0x1A00, 0, "<B", 2)
        sdo_w(s, 0x1C13, 1, "<H", 0x1A00)
        sdo_w(s, 0x1C13, 0, "<B", 1)
        print("Da cau hinh PDO.")
    return setup


def wait_until(cond, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.005)
    return False


def select_mode(drv):
    """MODE="csp": dung CSP (8). MODE="pp": thu PP (1), khong duoc thi dung CSP."""
    s = drv.slave
    if MODE == "csp":
        sdo_w(s, 0x6060, 0, "<b", 8)
        time.sleep(0.2)
        mode = sdo_r(s, 0x6061, 0, "<b")
        drv.csp = True
        print(f"Che do: CSP (mode doc lai = {mode})")
        return mode
    try:
        sdo_w(s, 0x6081, 0, "<I", deg2cnt(VEL_DEG_S))
        sdo_w(s, 0x6083, 0, "<I", deg2cnt(ACC_DEG_S2))
        sdo_w(s, 0x6084, 0, "<I", deg2cnt(ACC_DEG_S2))
        sdo_w(s, 0x6060, 0, "<b", 1)
        time.sleep(0.2)
        mode = sdo_r(s, 0x6061, 0, "<b")
    except Exception as e:
        print(f"Loi khi dat che do PP: {e}")
        mode = sdo_r(s, 0x6061, 0, "<b")
    if mode == 1:
        drv.csp = False
        print("Che do: Profile Position (PP)")
    else:
        print(f"eRob khong sang PP (mode hien tai = {mode}). Dung CSP.")
        sdo_w(s, 0x6060, 0, "<b", 8)
        drv.csp = True
    return mode


def hold(drv):
    """Dat target/goal = vi tri hien tai (tranh giat khi enable)."""
    drv.target = drv.goal = drv.pos
    drv.target_f = float(drv.pos)
    drv.v = 0.0


def enable(drv):
    hold(drv)
    if drv.status & 0x0008:
        print("Drive dang loi, reset...")
        drv.cw = 0x80
        time.sleep(0.2)
        drv.cw = 0x00
        time.sleep(0.2)
    for cw, expect, name in [(0x06, 0x21, "Ready to switch on"),
                             (0x07, 0x23, "Switched on"),
                             (0x0F, 0x27, "Operation enabled")]:
        hold(drv)
        drv.cw = cw
        if not wait_until(lambda: drv.status & 0x6F == expect, 2.0):
            print(f"Khong len duoc '{name}'. Statusword = 0x{drv.status:04X}")
            return False
        print(f"  -> {name}")
    return True


def move_to(drv, target_cnt, timeout):
    tol = deg2cnt(TOL_DEG)
    if drv.csp:
        drv.goal = target_cnt
        ok = wait_until(lambda: abs(drv.pos - target_cnt) <= tol and drv.v == 0.0, timeout)
        print(f"  Vi tri: {cnt2deg(drv.pos):.3f} deg "
              f"({'toi dich' if ok else 'CHUA TOI - het thoi gian'}) | SW: 0x{drv.status:04X}")
        return ok
    else:
        drv.cw = 0x0F                                   # bit4 = 0, giu 50 ms
        time.sleep(0.05)
        drv.target = target_cnt
        time.sleep(0.02)
        drv.cw = 0x3F                                   # bit4 new set-point + bit5 change immediately
        time.sleep(0.05)                                # giu du lau de drive chac chan nhan
        drv.cw = 0x0F
        time.sleep(0.05)
    ok = wait_until(lambda: abs(drv.pos - target_cnt) <= tol and drv.status & 0x0400, timeout)
    print(f"  Vi tri: {cnt2deg(drv.pos):.3f} deg "
          f"({'toi dich' if ok else 'CHUA TOI - het thoi gian'}) | SW: 0x{drv.status:04X}")
    return ok


def main():
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.winmm.timeBeginPeriod(1)

    adapters = pysoem.find_adapters()
    for i, a in enumerate(adapters):
        print(f"[{i}] {a.desc}")
    idx = int(input("Chon card noi voi eRob: "))

    master = pysoem.Master()
    master.open(adapters[idx].name)
    drv = None
    thread = None
    try:
        if master.config_init() <= 0:
            print("Khong tim thay eRob.")
            return
        slave = master.slaves[0]
        slave.config_func = make_config(master)
        master.config_map()
        if master.state_check(pysoem.SAFEOP_STATE, 50000) != pysoem.SAFEOP_STATE:
            print("Khong len SAFE-OP:", pysoem.al_status_code_to_string(slave.al_status))
            return

        drv = Drive(master, slave)
        thread = threading.Thread(target=drv.loop, daemon=True)
        thread.start()
        time.sleep(0.1)
        hold(drv)

        master.state = pysoem.OP_STATE
        master.write_state()
        for _ in range(50):
            if master.state_check(pysoem.OP_STATE, 50000) == pysoem.OP_STATE:
                break
        if master.state != pysoem.OP_STATE:
            master.read_state()
            print("Khong len OP:", pysoem.al_status_code_to_string(slave.al_status))
            return
        print("EtherCAT: OP")

        select_mode(drv)
        print(f"Vi tri hien tai: {cnt2deg(drv.pos):.3f} deg")
        if not enable(drv):
            return

        vel = VEL_DEG_S
        print("\nLenh: 15 / -15 (tuong doi), =90 (tuyet doi), v 20 (toc do), p (vi tri), q (thoat)")
        while True:
            cmd = input("\n> ").strip().lower()
            if not cmd:
                continue
            if cmd == "q":
                break
            if cmd == "p":
                mode = sdo_r(slave, 0x6061, 0, "<b")
                print(f"  Vi tri: {cnt2deg(drv.pos):.3f} deg | SW: 0x{drv.status:04X} | "
                      f"Mode: {mode} | Error: 0x{sdo_r(slave, 0x603F, 0, '<H'):04X}")
                continue
            try:
                if cmd.startswith("v"):
                    v = float(cmd[1:])
                    if not 0 < v <= MAX_VEL_DEG_S:
                        print(f"  Toc do phai trong (0, {MAX_VEL_DEG_S}] do/s")
                        continue
                    vel = v
                    drv.vel_cnt = deg2cnt(v)
                    if not drv.csp:
                        sdo_w(slave, 0x6081, 0, "<I", deg2cnt(v))
                    print(f"  Toc do = {vel} do/s")
                    continue
                if cmd.startswith("="):
                    target_deg = float(cmd[1:])
                else:
                    target_deg = cnt2deg(drv.pos) + float(cmd)
            except ValueError:
                print("  Lenh khong hop le.")
                continue

            delta = target_deg - cnt2deg(drv.pos)
            if abs(delta) > MAX_STEP_DEG:
                print(f"  Goc qua lon ({delta:.1f} do), gioi han {MAX_STEP_DEG} do")
                continue
            if drv.status & 0x6F != 0x27:
                print(f"  Mat Enable (SW 0x{drv.status:04X}), bat lai...")
                if not enable(drv):
                    break
            print(f"  Quay {delta:+.2f} do -> {target_deg:.2f} do")
            move_to(drv, deg2cnt(target_deg), abs(delta) / vel + 5.0)

    except KeyboardInterrupt:
        print("\nDung boi nguoi dung.")
    finally:
        if drv:
            hold(drv)
            drv.cw = 0x06
            time.sleep(0.2)
            drv.cw = 0x00
            time.sleep(0.1)
            drv.running = False
        if thread:
            thread.join(timeout=1)
        master.state = pysoem.INIT_STATE
        master.write_state()
        master.close()
        print("Da tat drive va dong ket noi.")


if __name__ == "__main__":
    main()
