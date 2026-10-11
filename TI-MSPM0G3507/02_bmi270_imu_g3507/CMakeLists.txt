cmake_minimum_required(VERSION 3.22)

project(mspm0g3507_hardware_spi_bmi270 C ASM)

set(SDK_ROOT "D:/ti/mspm0_sdk_2_11_00_07")

set(STARTUP_FILE "${SDK_ROOT}/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c"
)
set(DRIVERLIB "${SDK_ROOT}/source/ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a")

add_executable(bmi270_spi_basic
    src/main.c
    src/app_bmi270.c
    src/oled.c
    src/font.c
    src/servo.c
    src/uart_pc.c
    src/timebase.c

    imu/imu.c
    imu/bmi270_port.c
    imu/mahonyRK4.c
    imu/bmi270_minimal.c
    imu/bmi270_config.c

    generated/ti_msp_dl_config.c
    ${SDK_ROOT}/source/ti/driverlib/dl_common.c
    ${STARTUP_FILE}
)

target_include_directories(bmi270_spi_basic PRIVATE
    include
    imu
    generated
    ${SDK_ROOT}/source
    ${SDK_ROOT}/source/third_party/CMSIS/Core/Include
)

target_compile_options(bmi270_spi_basic PRIVATE
    @${CMAKE_SOURCE_DIR}/generated/device.opt
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mthumb
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -Wall
    -O2
    -g
)

target_link_options(bmi270_spi_basic PRIVATE
    -T${CMAKE_SOURCE_DIR}/generated/device_linker.lds
    -Wl,-Map,bmi270_spi_basic.map
    -Wl,--gc-sections
    -nostartfiles
    -static
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mthumb
    -mfloat-abi=soft
    --specs=nano.specs
    --specs=nosys.specs
)

target_link_libraries(bmi270_spi_basic PRIVATE
    ${DRIVERLIB}
    gcc
    c
    m
)

add_custom_command(TARGET bmi270_spi_basic POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:bmi270_spi_basic>
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:bmi270_spi_basic> bmi270_spi_basic.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:bmi270_spi_basic> bmi270_spi_basic.bin
)
