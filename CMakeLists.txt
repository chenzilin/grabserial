project(grabserial)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. DIR_SRCS)
add_executable(grabserial ${DIR_SRCS})

set (SRC_VERSION_MAJOR 1)
set (SRC_VERSION_MINOR 0)
set (SRC_VERSION_PATCH 1)
set (SRC_VERSION ${SRC_VERSION_MAJOR}.${SRC_VERSION_MINOR}.${SRC_VERSION_PATCH})
set (PROJECT_VERSION "${SRC_VERSION}")

add_definitions(-D_GNU_SOURCE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")

TARGET_LINK_LIBRARIES(grabserial pthread)

install(TARGETS grabserial DESTINATION bin)
