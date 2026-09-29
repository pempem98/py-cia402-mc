"""Brute-force object dictionary scan over SDO, for drives without SDO Info.

Reads sub-index 0 of every index in a range; a missing object answers with
abort 0x06020000 at once, so the whole range is quick. For each object found,
sub-indices 1..n are read too when sub 0 looks like an entry count. Read-only:
nothing is ever written. Names are unavailable this way, only addresses,
sizes and values.
"""
from __future__ import annotations

import json
import time
import warnings

import pysoem

ABORT_NO_OBJECT = 0x06020000
ABORT_NO_SUBINDEX = 0x06090011
ABORT_WRITE_ONLY = 0x06010001


def _read(slave, index: int, sub: int, retries: int = 4):
    """Return (bytes | None, abort_code | None, error_name | None)."""
    size = 0  # pysoem default buffer
    for _ in range(retries):
        try:
            return slave.sdo_read(index, sub, size) if size else slave.sdo_read(index, sub), None, None
        except pysoem.SdoError as e:
            return None, e.abort_code, None
        except pysoem.PacketError:
            # Usually "data container too small": a large object (string or
            # domain). Retry once with a big buffer, then give up on it.
            if size:
                return None, None, "packet error"
            size = 4096
        except (pysoem.WkcError, pysoem.Emergency, pysoem.MailboxError):
            time.sleep(0.02)
        except Exception as e:  # noqa: BLE001 - a scan must not stop halfway
            return None, None, type(e).__name__
    return None, None, "no reply"


def _describe(raw: bytes) -> dict:
    d = {"hex": raw.hex(), "size": len(raw)}
    if len(raw) in (1, 2, 4, 8):
        d["uint"] = int.from_bytes(raw, "little", signed=False)
        d["int"] = int.from_bytes(raw, "little", signed=True)
    elif raw and all(32 <= b < 127 for b in raw.rstrip(b"\x00")):
        d["text"] = raw.rstrip(b"\x00").decode()
    return d


def scan(slave, start: int = 0x1000, stop: int = 0x6FFF, progress=None) -> dict:
    warnings.simplefilter("ignore", FutureWarning)
    found: dict[str, dict] = {}
    unreadable: list[str] = []
    for index in range(start, stop + 1):
        if progress and index % 0x400 == 0:
            progress(index, len(found))
        raw0, abort0, err0 = _read(slave, index, 0)
        if abort0 == ABORT_NO_OBJECT:
            continue
        key = f"0x{index:04X}"
        if err0:
            unreadable.append(key)
            continue
        entry = {"sub0": _describe(raw0) if raw0 is not None else {"abort": hex(abort0)}}
        # Sub 0 of a record/array is a one-byte count; a one-byte VAR looks the
        # same, so probe sub 1: "no such sub-index" means it was a plain VAR.
        if raw0 is not None and len(raw0) == 1 and 0 < raw0[0] < 255:
            subs = {}
            for sub in range(1, raw0[0] + 1):
                raw, abort, err = _read(slave, index, sub)
                if abort == ABORT_NO_SUBINDEX and sub == 1:
                    break
                if raw is not None:
                    subs[str(sub)] = _describe(raw)
                elif abort is not None and abort != ABORT_NO_SUBINDEX:
                    subs[str(sub)] = {"abort": hex(abort)}
                elif err:
                    subs[str(sub)] = {"error": err}
            if subs:
                entry["subs"] = subs
        found[key] = entry
    return {"objects": found, "unreadable": unreadable}


def save(result: dict, path: str, device: dict) -> None:
    result = {"device": device, "time": time.strftime("%Y-%m-%d %H:%M:%S"), **result}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=1)
