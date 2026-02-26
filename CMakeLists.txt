cmake_minimum_required(VERSION 3.24)
project(analyzer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(IMGUI_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui_demo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui_tables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/imgui_widgets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_sdl3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/backends/imgui_impl_sdlrenderer3.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${IMGUI_SOURCES})
add_executable(test_graph src/test_graph.cpp)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(SDL3_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libs/SDL/debug/libSDL3.so")
else()
    set(SDL3_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libs/SDL/release/libSDL3.so")
endif()

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${SDL3_LIB_PATH}
)

get_filename_component(SDL_LIB_DIR "${SDL3_LIB_PATH}" DIRECTORY)
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${SDL_LIB_DIR}"
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        $<$<CONFIG:Debug>:${CMAKE_CURRENT_SOURCE_DIR}/libs/SDL/debug/libSDL3.so>
        $<$<CONFIG:Release>:${CMAKE_CURRENT_SOURCE_DIR}/libs/SDL/release/libSDL3.so>
)

target_include_directories(${PROJECT_NAME}
    PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/SDL/include/
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/imgui/backends/
)
