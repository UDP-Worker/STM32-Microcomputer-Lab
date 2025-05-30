cmake_minimum_required(VERSION 3.16)

# ← 在这里插入这行
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(arm_data_move LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)

add_executable(${PROJECT_NAME}
        data_move.S
)

target_link_options(${PROJECT_NAME} PRIVATE
        -nostdlib
        -nostartfiles
        -Wl,--entry=__main
)
