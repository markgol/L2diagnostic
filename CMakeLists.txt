cmake_minimum_required(VERSION 3.21)

project(L2diagnostic
    VERSION 0.2.6
    LANGUAGES CXX
)

# =============================
# Language / build settings
# =============================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Optional sanitizers / static analysis
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(ENABLE_CLANG_TIDY "Enable clang-tidy checks" OFF)

if(ENABLE_ASAN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")
endif()

if(ENABLE_UBSAN)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined")
    set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=undefined")
endif()

# =============================
# Qt packages
# =============================
set(Qt6_DIR "" CACHE PATH "Path to Qt6 installation")
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Network Charts)

# Enable Qt autogen features
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# =============================
# Source files
# =============================
file(GLOB_RECURSE PROJECT_SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.h"
    "${CMAKE_SOURCE_DIR}/src/*.ui"
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

# =============================
# Include paths
# =============================
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)

# =============================
# Architecture-specific libraries
# =============================
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCH "x86_64")
else()
    set(ARCH "aarch64")
endif()

set(LIB_DIR "${CMAKE_SOURCE_DIR}/lib/${ARCH}")
link_directories(${LIB_DIR})
# Example: if you have libunitree.a in lib/x86_64 or lib/aarch64
# target_link_libraries(${PROJECT_NAME} PRIVATE unitree)

# =============================
# Link Qt libraries
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
# MSVC-specific options
# =============================
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /MP /Zi)
    target_link_options(${PROJECT_NAME} PRIVATE /DEBUG)
endif()

# =============================
# Optional clang-tidy
# =============================
if(ENABLE_CLANG_TIDY)
    set_target_properties(${PROJECT_NAME} PROPERTIES CXX_CLANG_TIDY "clang-tidy")
endif()
