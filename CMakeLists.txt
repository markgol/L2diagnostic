cmake_minimum_required(VERSION 3.21)

project(L2diagnostic
    VERSION 0.3.11
    LANGUAGES CXX
)

# =============================
# Language / build settings
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

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Tell AUTOUIC where .ui files live
set(CMAKE_AUTOUIC_SEARCH_PATHS
    ${CMAKE_SOURCE_DIR}/forms
)

# =============================
# Source files
# =============================
file(GLOB_RECURSE PROJECT_SOURCES
    CONFIGURE_DEPENDS
    src/*.cpp
    include/*.h
    forms/*.ui
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

# =============================
# Include directories
# =============================
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/include
)

# =============================
# Compiler-specific options
# =============================
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /MP /Zi)
    target_link_options(${PROJECT_NAME} PRIVATE /DEBUG)
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

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
# Install rules (for packaging)
# =============================
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Install UI files (optional, for debugging / completeness)
install(DIRECTORY forms/
    DESTINATION share/${PROJECT_NAME}/forms
    FILES_MATCHING PATTERN "*.ui"
)

# =============================
# CPack (optional archive package)
# =============================
include(InstallRequiredSystemLibraries)

set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_GENERATOR "TGZ")

include(CPack)
