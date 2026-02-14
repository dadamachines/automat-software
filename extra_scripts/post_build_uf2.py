"""
PlatformIO post-build script: Convert firmware .bin to .uf2 format.

UF2 (USB Flashing Format) files can be flashed by dragging them onto the
USB mass-storage drive exposed by the UF2 bootloader (double-tap reset).

SAMD21 family ID: 0x68ed2b88
Application base address: 0x2000 (after 8 KB bootloader)

Reference: https://github.com/microsoft/uf2
"""

Import("env")
import struct
import os

UF2_MAGIC_START0 = 0x0A324655  # "UF2\n"
UF2_MAGIC_START1 = 0x9E5D5157
UF2_MAGIC_END    = 0x0AB16F30

UF2_FLAG_FAMILY_ID = 0x00002000

SAMD21_FAMILY_ID = 0x68ED2B88
APP_BASE_ADDRESS = 0x2000
BLOCK_SIZE       = 256  # payload bytes per UF2 block


def convert_bin_to_uf2(bin_path, uf2_path):
    with open(bin_path, "rb") as f:
        bin_data = f.read()

    num_blocks = (len(bin_data) + BLOCK_SIZE - 1) // BLOCK_SIZE
    blocks = []

    for block_no in range(num_blocks):
        offset = block_no * BLOCK_SIZE
        chunk = bin_data[offset:offset + BLOCK_SIZE]
        flags = UF2_FLAG_FAMILY_ID

        # UF2 block: 32 bytes header + 476 bytes data + 4 bytes final magic
        header = struct.pack(
            "<IIIIIIII",
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            flags,
            APP_BASE_ADDRESS + offset,
            BLOCK_SIZE,
            block_no,
            num_blocks,
            SAMD21_FAMILY_ID,
        )

        # Pad data to 476 bytes (512 - 32 header - 4 final magic)
        padding = b"\x00" * (476 - len(chunk))
        footer = struct.pack("<I", UF2_MAGIC_END)

        blocks.append(header + chunk + padding + footer)

    with open(uf2_path, "wb") as f:
        for block in blocks:
            f.write(block)

    print(f"UF2: {uf2_path} ({len(bin_data)} bytes, {num_blocks} blocks)")


# Firmware version for the output filename (keep in sync with automatConstants.h)
FIRMWARE_VERSION = "3.0.2"


def post_build_uf2(source, target, env):
    firmware_bin = os.path.join(
        env.subst("$BUILD_DIR"), env.subst("${PROGNAME}.bin")
    )
    # Named UF2: automat-sw_<version>.automat.uf2
    uf2_name = f"automat-sw_{FIRMWARE_VERSION}.automat.uf2"
    firmware_uf2 = os.path.join(env.subst("$BUILD_DIR"), uf2_name)

    if os.path.isfile(firmware_bin):
        convert_bin_to_uf2(firmware_bin, firmware_uf2)
    else:
        print(f"UF2: WARNING — {firmware_bin} not found, skipping UF2 generation")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", post_build_uf2)
