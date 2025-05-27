file(GLOB LITE_PHYSICS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Physics/*.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/Physics/Constraints/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Physics/Shapes/*.cpp)

file(GLOB LITE_PHYSICS_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/Physics/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Physics/Constraints/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Physics/Shapes/*.h)
file(GLOB LITE_PHYSICS_MATH_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Math/*.cpp)
file(GLOB LITE_PHYSICS_MATH_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/Math/*.h)

add_library(lite-physics STATIC ${LITE_PHYSICS_SRC} ${LITE_PHYSICS_HEADER} 
    ${LITE_PHYSICS_MATH_HEADER} ${LITE_PHYSICS_MATH_HEADER} ${LITE_PHYSICS_MATH_SRC} )

target_include_directories(lite-physics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR} )


source_group("Constraints" FILES ${CMAKE_CURRENT_SOURCE_DIR}/Physics/Constraints/*.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/Physics/Constraints/*.h)
source_group("Shapes" FILES ${CMAKE_CURRENT_SOURCE_DIR}/Physics/Shapes/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Physics/Shapes/*.h)
source_group("Math" FILES ${CMAKE_CURRENT_SOURCE_DIR}/Math/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Math/*.h)



# 添加渲染库
if(LITE_PHYSICS_ENABLE_RENDERING)
    message(STATUS "lite-physics: Rendering enabled")
    
    file(GLOB LITE_PHYSICS_RENDERER_SRC ${CMAKE_CURRENT_SOURCE_DIR}/Renderer/*.cpp)
    file(GLOB LITE_PHYSICS_RENDERER_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/Renderer/*.h)

    add_library(lite-physics-render STATIC ${LITE_PHYSICS_RENDERER_SRC} ${LITE_PHYSICS_RENDERER_HEADER})

    target_include_directories(lite-physics-render PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} 
        ${CMAKE_SOURCE_DIR}/libs/vulkan_1.1.108.0/include
        ${CMAKE_SOURCE_DIR}/libs/glfw-3.2.1.bin.WIN64/include)
    
    target_link_libraries(lite-physics-render PUBLIC lite-physics 
        ${CMAKE_SOURCE_DIR}/libs/vulkan_1.1.108.0/Lib/vulkan-1.lib
        ${CMAKE_SOURCE_DIR}/libs/glfw-3.2.1.bin.WIN64/lib-vc2015/glfw3.lib)
    
    add_compile_definitions(LITE_PHYSICS_RENDERER_SHADER_DIR="${CMAKE_SOURCE_DIR}/data/shaders/spirv")
    

    set(apps basic ccd multi convex)

    foreach(app ${apps})
        message(STATUS "adding app ${app}...")

        add_executable(
            ${app}
            ${CMAKE_CURRENT_SOURCE_DIR}/Apps/${app}.cpp
        )
        
        target_link_libraries(
            ${app}
            lite-physics
            lite-physics-render
        )

        target_include_directories(
            ${app}
            PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
        )
        
        message(STATUS "app ${app} is added")
    endforeach()

endif()




