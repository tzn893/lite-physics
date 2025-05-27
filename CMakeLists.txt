cmake_minimum_required(VERSION 3.24)
set(CMAKE_CXX_STANDARD 17)

project(lite-physics)

option(LITE_PHYSICS_ENABLE_RENDERING "enable building vulkan render graph samples" On)
option(LITE_PHYSICS_ENABLE_TESTING "enable testing physics" Off)

add_subdirectory(code)

if(LITE_PHYSICS_ENABLE_TESTING)
    add_subdirectory(test)
endif()

