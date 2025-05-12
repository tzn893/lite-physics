cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)

project(lite-physics)

option(LITE_PHYSICS_ENABLE_RENDERING "enable building vulkan render graph samples" On)

add_subdirectory(code)


