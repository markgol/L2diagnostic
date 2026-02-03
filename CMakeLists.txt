cmake_minimum_required(VERSION 3.21)

project(L2diagnostic
    VERSION 0.3.11
    LANGUAGES CXX
)

# =============================
# Build options
# =============================
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(ENABLE_CLANG_TIDY "Enable clang-tidy checks" OFF)

# =============================
# Language standard
# =============================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# =============================
# Qt packages
# =============================
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Widgets
    Network
    Charts
    OpenGL
    OpenGLWidgets
)

# =============================
# Qt autogen
# =============================
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Tell AUTOUIC where .ui files live
set(CMAKE_AUTOUIC_SEARCH_PATHS
    ${CMAKE_SOURCE_DIR}/forms
)

# =============================
# Sources
# =============================
file(GLOB_RECURSE PROJECT_SOURCES
    CONFIGURE_DEPENDS
    src/*.cpp
    include/*.h
    forms/*.ui
)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE ${PROJECT_SOURCES}
)

# =============================
# Include directories
# =============================
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
)

# =============================
# Link libraries
# =============================
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::Widgets
        Qt6::Network
        Qt6::Charts
        Qt6::OpenGL
        Qt6::OpenGLWidgets
)

# =============================
# Compiler warnings
# =============================
target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/MP /W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)

# =============================
# MSVC debug info
# =============================
if(MSVC)
    target_link_options(${PROJECT_NAME} PRIVATE /DEBUG)
endif()

# =============================
# Sanitizers (GCC/Clang only)
# =============================
if(ENABLE_ASAN AND NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        -fsanitize=address
        -fno-omit-frame-pointer
    )
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address)
endif()

if(ENABLE_UBSAN AND NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=undefined)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=undefined)
endif()

# =============================
# clang-tidy
# =============================
if(ENABLE_CLANG_TIDY)
    set_target_properties(${PROJECT_NAME}
        PROPERTIES
            CXX_CLANG_TIDY "clang-tidy"
    )
endif()

# =============================
# Diagnostics
# =============================
message(STATUS "System: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Qt version: ${Qt6Core_VERSION}")
