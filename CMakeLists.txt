cmake_minimum_required(VERSION 3.10)
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

#Project name
set(PRJ_NAME opack)
#Enter MCU model here
set(MCU_NAME STM32F107VC)
#Enter MCU value line here
set(MCU_VALUE_LINE STM32F10x_CL)

#Path to local toolchain directory
set(TOOLCHAIN_DIR "./" CACHE STRING "User specified path")
message("tool chaindir is" ${TOOLCHAIN_DIR})
set(LINKER_SCRIPT_NAME STM32F107XC_FLASH)
set(START_UP_FILE_NAME startup_stm32f107xc.s)
#MCU architecture
set(ARCH cortex-m3)

#FPU operations hard or soft
set(FLOAT_ABI soft)

#FPU type
set(FPU vfp)

#Switch between debug and release build
#set(CMAKE_BUILD_TYPE Debug)

find_program(ARM_CC arm-none-eabi-gcc ${TOOLCHAIN_DIR}/bin)
if (NOT ARM_CC)
    message(FATAL_ERROR "ARM_CC not found!")
endif()

find_program(ARM_AS arm-none-eabi-as ${TOOLCHAIN_DIR}/bin)
if (NOT ARM_AS)
    message(FATAL_ERROR "ARM_AS not found!")
endif()

find_program(ARM_CXX arm-none-eabi-g++ ${TOOLCHAIN_DIR}/bin)
if (NOT ARM_CXX)
    message(FATAL_ERROR "ARM_CXX not found!")
endif()

find_program(ARM_OBJCOPY arm-none-eabi-objcopy ${TOOLCHAIN_DIR}/bin)
if (NOT ARM_OBJCOPY)
    message(FATAL_ERROR "ARM_OBJCOPY not found!")
endif()

find_program(ARM_SIZE_TOOL arm-none-eabi-size ${TOOLCHAIN_DIR}/bin)
if (NOT ARM_SIZE_TOOL)
    message(FATAL_ERROR "ARM_SIZE_TOOL not found!")
endif()

#Specify cross compiler
include(CMakeForceCompiler)
set(CMAKE_CPP_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER  ${ARM_CC})
set(CMAKE_CXX_COMPILER ${ARM_CXX})

#Set linker script
set(CMAKE_LINKER ${ARM_CC})
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/${LINKER_SCRIPT_NAME}.ld)
set(START_UP_FILE ${CMAKE_SOURCE_DIR}/CMSIS/${START_UP_FILE_NAME})
#Project name here
project(${PRJ_NAME} ASM C)

#Common compiler flags
set(COMMON_FLAGS "-mcpu=${ARCH} -mthumb -mthumb-interwork -mlittle-endian -mfloat-abi=${FLOAT_ABI} -mfpu=${FPU}  -Wall  -g -O0 -ffunction-sections -lm -lgcc -lc ")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=c11 -g3")# -Werror")
set(CMAKE_AS_FLAGS "${COMMON_FLAGS}")

set(CMAKE_EXE_LINKER_FLAGS "-mcpu=${ARCH} -nostartfiles -O0 -Wl,--gc-sections -specs=nosys.specs -lnosys -mfpu=${FPU} -mthumb -Wall -lm -lgcc -lc -T ${LINKER_SCRIPT}")

add_definitions(-D${MCU_NAME})
add_definitions(-DUSE_HAL_DRIVER -DASSEMBLY -DARM_MATH_CM3)


#include paths
#USUAL LIBS
include_directories("CMSIS")
include_directories("project_src")
include_directories("syscalls")
include_directories("STM32F1xx_HAL_Driver/Inc")
include_directories("STM32F1xx_HAL_Driver/Src")

#include files
file(GLOB_RECURSE PROJ_SRC "project_src/*")
file(GLOB_RECURSE CMSIS "CMSIS/*")
file(GLOB SYSCALLS "syscalls/*")
file(GLOB HAL "STM32F1xx_HAL_Driver/Inc/*"
              "STM32F1xx_HAL_Driver/Src/*"
                      "STM32F1xx_HAL_Driver/Legacy/*")



add_executable(${PROJECT_NAME}.elf

    #User files
    ${PROJ_SRC}
    #Libraries
    ${HAL} ${SYSCALLS}
    ${CMSIS}
    #Startup and linker script files
    ${START_UP_FILE} ${LINKER_SCRIPT}
)











































