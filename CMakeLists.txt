cmake_minimum_required(VERSION 3.21)

project(L2diagnostic
    VERSION 0.4.2
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

if(WIN32)
    add_executable(${PROJECT_NAME} WIN32 ${PROJECT_SOURCES})
else()
    add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
endif()

# =============================
# Include directories
# =============================
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/include
)

if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
endif()

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

# =========================================================
# AppImage packaging (Linux only, safe for Windows builds)
# =========================================================
if(UNIX AND NOT APPLE)

    set(APPDIR ${CMAKE_BINARY_DIR}/AppDir)

    # set QT_PLATFORM_PLUGIN_PATH
    # to source folder, aarch64-linux-gnu, /x86_64-linux-gnu
	# for the Qt6 platform plugins

    if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(QT_PLATFORM_PLUGIN_PATH /home/markstegall/Qt/6.10.2/gcc_64/plugins/platforms)
    elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(QT_PLATFORM_PLUGIN_PATH /usr/lib/aarch64-linux-gnu/qt6/plugins/platforms)
    elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm64")
        set(QT_PLATFORM_PLUGIN_PATH /usr/lib/aarch64-linux-gnu/qt6/plugins/platforms)
    endif()

    message(STATUS "QT_PLATFORM_PLUGIN_PATH: " ${QT_PLATFORM_PLUGIN_PATH})

    add_custom_target(appimage
        COMMAND ${CMAKE_COMMAND} -E rm -rf ${APPDIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${APPDIR}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${APPDIR}/usr/bin
        COMMAND ${CMAKE_COMMAND} -E make_directory ${APPDIR}/usr/lib

        # Copy executable
        COMMAND ${CMAKE_COMMAND} -E copy
                $<TARGET_FILE:${PROJECT_NAME}>
                ${APPDIR}/usr/bin/${PROJECT_NAME}

        #copy platforms files to usr/lib directory
        #linuxdeploy does not find these
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${QT_PLATFORM_PLUGIN_PATH} ${APPDIR}/usr/lib
        #COMMAND ${CMAKE_COMMAND} -E copy
        #        ${QT_PLATFORM_PLUGIN_PATH}/libqxcb.so
        #        ${APPDIR}/usr/lib
        #COMMAND ${CMAKE_COMMAND} -E copy
        #        ${QT_PLATFORM_PLUGIN_PATH}/libqvnc.so
        #       ${APPDIR}/usr/lib
        #COMMAND ${CMAKE_COMMAND} -E copy
        #        ${QT_PLATFORM_PLUGIN_PATH}/libqwayland.so
        #        ${APPDIR}/usr/lib

        # Run linuxdeploy with Qt plugin
        COMMAND linuxdeploy -v0
                --appdir ${APPDIR}
                --executable ${APPDIR}/usr/bin/${PROJECT_NAME}
                --desktop-file ${CMAKE_SOURCE_DIR}/packaging/${PROJECT_NAME}.desktop
                --create-desktop-file
                --icon-file ${CMAKE_SOURCE_DIR}/packaging/${PROJECT_NAME}.png
                --output appimage

        DEPENDS ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}

        #create bundle folder
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bundle
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bundle/docs

        #copy appimage to bundle folder
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_BINARY_DIR}/${PROJECT_NAME}-${CMAKE_HOST_SYSTEM_PROCESSOR}.AppImage
                ${CMAKE_BINARY_DIR}/bundle/${PROJECT_NAME}-${CMAKE_HOST_SYSTEM_PROCESSOR}.AppImage

        #copy docs folder to bundle folder
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_SOURCE_DIR}/docs ${CMAKE_BINARY_DIR}/bundle/docs

        #copy readme.md and readmore.md to bundle folder
        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/README.md
                ${CMAKE_BINARY_DIR}/bundle/README.md

        COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_SOURCE_DIR}/READMore.md
                ${CMAKE_BINARY_DIR}/bundle/READMore.md

        COMMENT "Building AppImage using linuxdeploy + Qt plugin"
    )

endif()
