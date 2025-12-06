cmake_minimum_required(VERSION 3.14)
project(DroneDigitalTwin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Includes
include_directories(cpp/include)

# Dependencies: using FetchContent for portability
include(FetchContent)

# 1. nlohmann_json
FetchContent_Declare(json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)
FetchContent_MakeAvailable(json)

# 2. cppzmq (ZeroMQ C++ Wrapper) & libzmq
# We need libzmq as well.
FetchContent_Declare(libzmq
    GIT_REPOSITORY https://github.com/zeromq/libzmq.git
    GIT_TAG v4.3.5
)
FetchContent_Declare(cppzmq
    GIT_REPOSITORY https://github.com/zeromq/cppzmq.git
    GIT_TAG v4.10.0
)
FetchContent_MakeAvailable(libzmq cppzmq)

# Source files
file(GLOB SOURCES "cpp/src/*.cpp")

# Executable
add_executable(drone_sim ${SOURCES})

# Limits dependencies to just what we need
target_link_libraries(drone_sim PRIVATE nlohmann_json::nlohmann_json cppzmq libzmq-static)
