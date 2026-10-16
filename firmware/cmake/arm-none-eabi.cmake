# Cross-compilation toolchain for the STM32H743VIT6 (Cortex-M7, double-precision FPU).
#
# Expects the Arm GNU Toolchain on PATH; scripts/install_arm_toolchain.sh
# installs the pinned release (14.3.rel1) that CI uses.
#
#   cmake -S . -B build-arm -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER   arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY      arm-none-eabi-objcopy CACHE FILEPATH "objcopy")
set(CMAKE_SIZE         arm-none-eabi-size    CACHE FILEPATH "size")

# The compiler cannot link a hosted test binary, so skip the default probe.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# fpv5-d16 rather than the single-precision fpv5-sp-d16: the filter design runs
# in double at start-up and on this part hardware doubles cost nothing extra.
set(PA_MCU_FLAGS "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT   "${PA_MCU_FLAGS} -ffunction-sections -fdata-sections")
# -fno-use-cxa-atexit: the firmware never exits, so registering destructors for
# objects with static storage duration is pure overhead and it drags in the
# __cxa_atexit machinery that -nostartfiles has already removed.
set(CMAKE_CXX_FLAGS_INIT "${PA_MCU_FLAGS} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics -fno-use-cxa-atexit")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${PA_MCU_FLAGS} -Wl,--gc-sections")
