cmake_minimum_required(VERSION 2.8)
include_directories("${PROJECT_SOURCE_DIR}/lib/include")
include_directories("${PROJECT_SOURCE_DIR}/../common")
#include_directories("${PROJECT_SOURCE_DIR}/msvc/include")
include_directories("${PROJECT_SOURCE_DIR}/lib/src/")
include_directories("${PROJECT_SOURCE_DIR}/apps/src/cli/")
include_directories("${PROJECT_SOURCE_DIR}/apps/src/cli/cmd")
include_directories("${PROJECT_SOURCE_DIR}/apps/src/cli/interactive")
include_directories("${PROJECT_SOURCE_DIR}/../../common")
#include_directories("${PROJECT_SOURCE_DIR}/msvc/include")
#include_directories("F:/Downloads/libusbx-1.0.16/libusbx-1.0.16/libusb")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
find_package(USB1 REQUIRED)
find_package(Tecla REQUIRED)
#SET (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SAFESEH:NO")
#SET (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /SAFESEH:NO")
#SET (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /SAFESEH:NO")
if( TECLA_LIB )
  add_definitions( -DINTERACTIVE )
endif(TECLA_LIB)
# Main file
set( SOURCES src/cli/main.c src/cli/common.c )

# Interactivity
set( SOURCES ${SOURCES} src/cli/interactive/interactive.c )

# Command files
set( SOURCES ${SOURCES} src/cli/cmd/cmd.c src/cli/cmd/load.c src/cli/cmd/open.c src/cli/cmd/peek.c src/cli/cmd/peekpoke.c src/cli/cmd/poke.c src/cli/cmd/printset.c src/cli/cmd/probe.c src/cli/cmd/rxtx.c src/cli/cmd/version.c )

add_executable(bladerf-cli ${SOURCES})

#add_executable(bladerftest src/bladerftest.c)
target_link_libraries(bladerf-cli ${TECLA_LIB})
target_link_libraries(bladerf-cli bladerf)
#target_link_libraries(bladerftest libbladerf_static)
