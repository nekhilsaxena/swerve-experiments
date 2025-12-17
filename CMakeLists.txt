cmake_minimum_required(VERSION 3.20...4.0)
project(SwerveSim)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

file(GLOB SOURCES "src/*.cpp")

add_executable(SwerveSim ${SOURCES})

if(MSVC)
    target_link_libraries(SwerveSim user32.lib gdi32.lib opengl32.lib gdiplus.lib Shlwapi.lib dwmapi.lib)
endif()
