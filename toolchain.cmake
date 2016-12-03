SET(CMAKE_SYSTEM_NAME Generic)
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET (CMAKE_C_COMPILER arm-none-eabi-gcc)
SET (CMAKE_CXX_COMPILER arm-none-eabi-g++)


SET(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F030F4Px_FLASH.ld)
SET(COMMON_FLAGS "-mcpu=cortex-m0 -mthumb -mthumb-interwork -ffunction-sections -fdata-sections -g -fno-common -fmessage-length=0")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu99")
SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T ${LINKER_SCRIPT}")