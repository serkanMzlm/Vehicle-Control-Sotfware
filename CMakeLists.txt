cmake_minimum_required(VERSION 3.15...3.27)

project(Captain
    VERSION      1.0.0
    DESCRIPTION  "Captain Project"
    LANGUAGES    C CXX
)

find_package(Threads REQUIRED)

cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0075 NEW)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_C_STANDARD          11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS        OFF)

if(DEFINED CPP_STD)
    set(CMAKE_CXX_STANDARD ${CPP_STD})
else()
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

message(STATUS "Project      : ${PROJECT_NAME} v${PROJECT_VERSION}")
message(STATUS "Source       : ${CMAKE_SOURCE_DIR}")
message(STATUS "Build        : ${CMAKE_BINARY_DIR}")
message(STATUS "C++ Standard : ${CMAKE_CXX_STANDARD}")

add_compile_options(-Wall -Wextra -Wpedantic)

add_executable(${PROJECT_NAME} src/main.cpp)

target_include_directories(${PROJECT_NAME}
    PRIVATE ${CMAKE_SOURCE_DIR}/include
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
        PROJECT_NAME="${PROJECT_NAME}"
        PROJECT_VERSION="${PROJECT_VERSION}"
        $<$<CONFIG:Debug>:DEBUG_BUILD=1> 
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        Threads::Threads        # find_package(Threads) tarafından tanımlanır
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    OUTPUT_NAME              "captain"
    C_STANDARD               11
    C_STANDARD_REQUIRED      ON
    CXX_STANDARD             ${CMAKE_CXX_STANDARD}
    CXX_STANDARD_REQUIRED    ON
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)