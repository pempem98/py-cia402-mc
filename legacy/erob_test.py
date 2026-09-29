"""
Kiem tra ket noi EtherCAT voi eRob bang pysoem (CHI DOC, khong lam khop quay).
Can: Npcap (tick "WinPcap API-compatible mode") + pip install pysoem
Chay CMD/PowerShell bang quyen Administrator.
"""
import struct
import sys

import pysoem


def read_sdo(slave, index, sub, fmt, name):
    try:
        data = slave.sdo_read(index, sub)
        if fmt == "str":
            value = data.decode(errors="ignore").strip("\x00")
        else:
            value = struct.unpack(fmt, data[:struct.calcsize(fmt)])[0]
        if isinstance(value, int) and fmt in ("<I", "<H"):
            print(f"  0x{index:04X}:{sub:02d} {name:<22} = {value} (0x{value:X})")
        else:
            print(f"  0x{index:04X}:{sub:02d} {name:<22} = {value}")
    except Exception as e:
        print(f"  0x{index:04X}:{sub:02d} {name:<22} -> loi: {e}")


def main():
    adapters = pysoem.find_adapters()
    if not adapters:
        print("Khong tim thay card mang nao. Kiem tra da cai Npcap chua.")
        sys.exit(1)

    print("Danh sach card mang:")
    for i, a in enumerate(adapters):
        print(f"  [{i}] {a.desc}")
    idx = int(input("Chon so thu tu card noi voi eRob: "))

    master = pysoem.Master()
    master.open(adapters[idx].name)
    try:
        count = master.config_init()
        if count <= 0:
            print("\nKHONG tim thay slave EtherCAT nao.")
            print("-> Kiem tra nguon 48V, cap cam vao cong IN, den link tren eRob.")
            return

        print(f"\nTim thay {count} slave:")
        for i, s in enumerate(master.slaves):
            print(f"\n[{i}] {s.name}")
            print(f"  Vendor ID: 0x{s.man:08X}  Product: 0x{s.id:08X}  Rev: 0x{s.rev:08X}")
            print(f"  Trang thai hien tai: {s.state} (1=INIT, 2=PREOP, 4=SAFEOP, 8=OP)")
            read_sdo(s, 0x1008, 0, "str", "Device name")
            read_sdo(s, 0x100A, 0, "str", "Software version")
            read_sdo(s, 0x6041, 0, "<H", "Statusword")
            read_sdo(s, 0x6061, 0, "<b", "Mode of operation")
            read_sdo(s, 0x6064, 0, "<i", "Position actual")
            read_sdo(s, 0x603F, 0, "<H", "Error code")
    finally:
        master.close()


if __name__ == "__main__":
    main()
