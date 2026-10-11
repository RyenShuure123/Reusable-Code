cmake_minimum_required(VERSION 3.22)

project(Car3519new C ASM)

set(SDK_ROOT "D:/ti/mspm0_sdk_2_11_00_07")
set(SYSCONFIG_CLI "D:/ti/sysconfig/sysconfig_cli.bat")

set(GENERATED_DIR "${CMAKE_SOURCE_DIR}/generated")
set(SYSCFG_SCRIPT "${CMAKE_SOURCE_DIR}/syscfg/car3519_minimal.syscfg")
set(GENERATED_C "${GENERATED_DIR}/ti_msp_dl_config.c")
set(GENERATED_H "${GENERATED_DIR}/ti_msp_dl_config.h")
set(DEVICE_OPT "${GENERATED_DIR}/device.opt")
set(DEVICE_LINKER "${GENERATED_DIR}/device_linker.lds")

set(STARTUP_FILE
    "${SDK_ROOT}/source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g351x_gcc.c"

)
set(DRIVERLIB
    "${SDK_ROOT}/source/ti/driverlib/lib/gcc/m0p/mspm0gx51x/driverlib.a")

set(OPENOCD_EXECUTABLE
    "D:/DevEnv/ti-openocd/openocd_1.5.0.75/openocd/bin/openocd.exe")
set(OPENOCD_SCRIPTS
    "D:/DevEnv/ti-openocd/openocd_1.5.0.75/openocd/share/openocd/scripts")
set(OPENOCD_CONFIG
    "${CMAKE_SOURCE_DIR}/openocd/mspm0g3519_daplink.cfg")

if(NOT EXISTS "${SDK_ROOT}/.metadata/product.json")
    message(FATAL_ERROR "未找到 MSPM0 SDK：${SDK_ROOT}")
endif()

if(NOT EXISTS "${SYSCONFIG_CLI}")
    message(FATAL_ERROR "未找到 SysConfig CLI：${SYSCONFIG_CLI}")
endif()

add_custom_command(
    OUTPUT
        "${GENERATED_C}"
        "${GENERATED_H}"
        "${DEVICE_OPT}"
        "${DEVICE_LINKER}"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${GENERATED_DIR}"
    COMMAND "${SYSCONFIG_CLI}"
        --compiler gcc
        --product "${SDK_ROOT}/.metadata/product.json"
        --script "${SYSCFG_SCRIPT}"
        --output "${GENERATED_DIR}"
    DEPENDS "${SYSCFG_SCRIPT}"
    COMMENT "SysConfig：生成 MSPM0G3519 配置文件"
    VERBATIM
)

add_custom_target(syscfg_generation
    DEPENDS
        "${GENERATED_C}"
        "${GENERATED_H}"
        "${DEVICE_OPT}"
        "${DEVICE_LINKER}"
)

add_executable(app
    src/main.c
    src/LED.c
    src/button.c
    src/control_timer.c
    src/drive_control.c
    src/encoder.c
    src/loop.c
    src/motor.c
    src/pid.c
    src/uart_bluetooth.c
    src/vofa_justfloat.c

    imu/imu_uart.c
    imu/yaw_uart_receiver.c

    Module/font.c
    Module/oled.c

    tasks/task1.c
    tasks/task2.c
    tasks/task3.c

        "${GENERATED_C}"
    "${SDK_ROOT}/source/ti/driverlib/dl_common.c"
    "${STARTUP_FILE}"
)

add_dependencies(app syscfg_generation)

target_include_directories(app PRIVATE
    include
    imu
    tasks
    Module
    "${GENERATED_DIR}"
    "${SDK_ROOT}/source"
    "${SDK_ROOT}/source/third_party/CMSIS/Core/Include"
)

target_compile_definitions(app PRIVATE __MSPM0G3519__)

target_compile_options(app PRIVATE
    "@${DEVICE_OPT}"
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mthumb
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -Wall
    -O0
    -g3
)

target_link_options(app PRIVATE
    "-T${DEVICE_LINKER}"
    -Wl,-Map,app.map
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

target_link_libraries(app PRIVATE
    "${DRIVERLIB}"
    gcc
    c
    m
)

add_custom_command(TARGET app POST_BUILD
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:app>"
    COMMAND "${CMAKE_OBJCOPY}" -O ihex
        "$<TARGET_FILE:app>" app.hex
    VERBATIM
)

add_custom_target(flash
    COMMAND "${OPENOCD_EXECUTABLE}"
        -s "${OPENOCD_SCRIPTS}"
        -f "${OPENOCD_CONFIG}"
        -c "program $<TARGET_FILE_DIR:app>/app.hex verify reset exit"
    DEPENDS app
    COMMENT "OpenOCD：通过 LC DAPLink 烧录 app.hex"
    VERBATIM
)
