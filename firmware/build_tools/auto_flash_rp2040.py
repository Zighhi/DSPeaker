"""
Automated RP2040 UF2 Flasher with Explicit OS Hardware Sync (fsync) Enforced.
"""

import os
import sys
import time

def find_rp2040_drive():
    for drive in ["F:\\", "E:\\", "D:\\", "G:\\", "H:\\", "I:\\"]:
        if os.path.exists(drive):
            try:
                files = os.listdir(drive)
                if "INFO_UF2.TXT" in files or "INDEX.HTM" in files:
                    return drive
            except Exception:
                pass
    return None

def main():
    print("==================================================")
    print("AUTOMATED RP2040 UF2 FLASHER (HARDWARE FSYNC ENFORCED)")
    print("==================================================")

    drive = find_rp2040_drive()
    if not drive:
        print("Error: RP2040 Bootloader drive (e.g. F:\\) not found!")
        print("Please hold BOOT button and reconnect USB cable.")
        sys.exit(1)

    print(f"Found RP2040 Bootloader Drive at: {drive}")

    default_uf2 = r"E:\Projects\Speaker_rebuild\01_Firmware\rp2040_dsp\miXZer\build\miXZer.uf2"
    uf2_file = sys.argv[1] if len(sys.argv) > 1 else default_uf2
    if not os.path.exists(uf2_file):
        print(f"Error: UF2 file not found at {uf2_file}")
        sys.exit(1)

    target_file = os.path.join(drive, "aura_speaker_dsp.uf2")

    print(f"Flashing {uf2_file} ({os.path.getsize(uf2_file)} bytes) -> {target_file}...")

    # Write raw binary blocks and force immediate OS hardware sync
    with open(uf2_file, "rb") as f_in:
        data = f_in.read()

    with open(target_file, "wb") as f_out:
        f_out.write(data)
        f_out.flush()
        os.fsync(f_out.fileno()) # Enforce OS hardware write sync!

    print("==================================================")
    print("SUCCESS: 100% UF2 BINARY FLASHED AND HARDWARE SYNCED!")
    print("==================================================")

if __name__ == "__main__":
    main()
