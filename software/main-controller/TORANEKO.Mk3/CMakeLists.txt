cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME TORANEKO.Mk3)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# User sources (App/). Add one file per line as each step is implemented
set(APP_SOURCES
    App/app.c
    App/ctrl/control.c
    App/ctrl/odometry.c
    App/ctrl/run.c
    App/ctrl/trajectory.c
    App/ctrl/wall_ctrl.c
    App/ctrl/wall_dist.c
    App/drv/encoder.c
    App/drv/fan.c
    App/drv/flash_memory.c
    App/drv/imu.c
    App/drv/led.c
    App/drv/motor.c
    App/drv/switch.c
    App/drv/timebase.c
    App/drv/uart.c
    App/drv/wall.c
    App/maze/maze_map.c
    App/maze/search.c
    App/maze/path.c
    App/maze/dijkstra.c
    App/sys/adjust.c
    App/sys/encoder_cal.c
    App/sys/explore.c
    App/sys/failsafe.c
    App/sys/flash.c
    App/sys/goal.c
    App/sys/log.c
    App/sys/mode.c
    App/sys/output.c
    App/sys/scheduler.c
    App/sys/sensor_record.c
    App/sys/shortest.c
    App/sys/turn_test.c
    App/sys/sensor_view.c
    App/sys/ui.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${APP_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    App
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Warnings for user code only (App/); generated HAL code is left as is
set_source_files_properties(${APP_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

# 走っているあいだに時間がかかる計算（迷路の計算と、制御周期の割り込みで動く制御）だけは、Debug でも
# 速くなるようにビルドする（-O2）。-O0 のままだと、探索で区画の入口に着いてから次の動きを決めるまでに
# 30〜40ms かかり、そのあいだに機体が 10mm ほど進んで、曲がり始めが遅れた（2026-10-24）。
# ここに挙げたファイルは、デバッガで 1 行ずつ止めて見ると、行が飛んで見えることがある
set(APP_FAST_SOURCES
    App/maze/maze_map.c
    App/maze/search.c
    App/maze/path.c
    App/maze/dijkstra.c
    App/ctrl/control.c
    App/ctrl/odometry.c
    App/ctrl/trajectory.c
    App/ctrl/wall_ctrl.c
    App/ctrl/wall_dist.c
)
set_source_files_properties(${APP_FAST_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-O2")

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# printf で小数（%f）を出せるようにする（走行ログの出力に使う。KOGUMA_PORTING.md の B6）。
# nano.specs（小さい printf）は、そのままだと %f を出さないので、小数を出す部分をリンクに入れさせる。Flash が約 13KB 増える（測った値：28.6KB → 42.1KB）
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
