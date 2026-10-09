"""
Automated Build & Flash Script for miXZer RP2040 PIO I2S Firmware
"""

import os
import shutil
import subprocess
import sys

# Paths
PICO_SDK_PATH = "C:/Users/Zighi/.pico-sdk/sdk/2.2.0"
PICOTOOL_BIN_PATH = "C:/Users/Zighi/.pico-sdk/picotool/2.2.0"
TOOLCHAIN_PATH = "C:/Users/Zighi/.pico-sdk/toolchain/14_2_Rel1/bin"
CMAKE_PATH = "C:/Users/Zighi/.pico-sdk/cmake/v3.31.5/bin/cmake.exe"
NINJA_PATH = "C:/Users/Zighi/.pico-sdk/ninja/v1.12.1/ninja.exe"

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
BUILD_DIR = os.path.join(PROJECT_DIR, "build")
BUILD_TOOLS = os.path.join(os.path.dirname(PROJECT_DIR), "archive_rp2040", "build_tools")
auto_flash_script = os.path.join(BUILD_TOOLS, "auto_flash_rp2040.py")

# Environment
os.environ["PATH"] = PICOTOOL_BIN_PATH + os.pathsep + TOOLCHAIN_PATH + os.pathsep + os.path.dirname(CMAKE_PATH) + os.pathsep + os.path.dirname(NINJA_PATH) + os.pathsep + os.environ.get("PATH", "")
os.environ["PICO_SDK_PATH"] = PICO_SDK_PATH

if os.path.exists(BUILD_DIR):
    shutil.rmtree(BUILD_DIR, ignore_errors=True)
os.makedirs(BUILD_DIR, exist_ok=True)

print("==================================================================")
print("BUILDING miXZer HARDWARE PIO I2S FIRMWARE (RP2040-ZERO)")
print("==================================================================")

# 1. CMake Configure
cmake_cmd = [
    CMAKE_PATH,
    "-G", "Ninja",
    f"-DCMAKE_MAKE_PROGRAM={NINJA_PATH}",
    f"-DPICO_SDK_PATH={PICO_SDK_PATH}",
    f"-Dpicotool_DIR={PICOTOOL_BIN_PATH}",
    "-DPICOTOOL_FETCH_FROM_GIT=OFF",
    f"-DCMAKE_C_COMPILER={TOOLCHAIN_PATH}/arm-none-eabi-gcc.exe",
    f"-DCMAKE_CXX_COMPILER={TOOLCHAIN_PATH}/arm-none-eabi-g++.exe",
    ".."
]

res = subprocess.run(cmake_cmd, cwd=BUILD_DIR)
if res.returncode != 0:
    print("CMake Configuration Failed!")
    sys.exit(1)

# 2. Ninja Build
ninja_cmd = [NINJA_PATH]
res = subprocess.run(ninja_cmd, cwd=BUILD_DIR)
if res.returncode != 0:
    print("Ninja Build Failed!")
    sys.exit(1)

uf2_file = os.path.join(BUILD_DIR, "miXZer.uf2")
print("------------------------------------------------------------------")
print(f"BUILD SUCCESSFUL!")
print(f"UF2 Binary Ready: {uf2_file}")
print("==================================================================")
