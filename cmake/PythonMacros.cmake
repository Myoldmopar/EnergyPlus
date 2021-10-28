macro(SET_CPYTHON_VARIABLES)
    # sets the following variables:
    # - CPYTHON_DIR: the folder containing the clone of CPython itself
    # - CPYTHON_STDLIB_DIR: the folder containing the Python standard library implementation
    # - CPYTHON_LIBRARY: the name of the Python library, with version number
    # - CPYTHON_INCLUDE_DIR: the C include directories for compiling
    # - CPYTHON_LIBRARY_DIR: the folder containing the compiled Python shared object library file
    # - CPYTHON_BUILT_BIN: the full path to the Python shared object library file
    set(CPYTHON_DIR "${PROJECT_SOURCE_DIR}/third_party/CPython")
    set(CPYTHON_STDLIB_DIR ${CPYTHON_DIR}/Lib)
    set(CPYTHON_LIBRARY python3.11)
    if(MSVC)
        set(CPYTHON_INCLUDE_DIR ${CPYTHON_DIR}/Include ${CPYTHON_DIR}/PC)
        set(CPYTHON_LIBRARY_DIR ${CPYTHON_DIR}/PCBuild/${CPYTHON_BUILD_DIR})
        if(CMAKE_BUILD_TYPE MATCHES "Debug")
            set(CPYTHON_BUILT_BIN ${CPYTHON_DIR}/PCBuild/${CPYTHON_BUILD_DIR}/${CPYTHON_LIBRARY}_d.dll)
        else()
            set(CPYTHON_BUILT_BIN ${CPYTHON_DIR}/PCBuild/${CPYTHON_BUILD_DIR}/${CPYTHON_LIBRARY}.dll)
        endif()
    else()
        set(CPYTHON_INCLUDE_DIR ${CPYTHON_DIR}/Include ${CPYTHON_DIR})
        set(CPYTHON_LIBRARY_DIR ${CPYTHON_DIR})
        if (UNIX AND NOT APPLE)
            set(CPYTHON_BIN_NAME "lib${CPYTHON_LIBRARY}.so.1.0")
        elseif(APPLE)
            set(CPYTHON_BIN_NAME "lib${CPYTHON_LIBRARY}.dylib")
        endif()
        set(CPYTHON_BUILT_BIN "${CPYTHON_DIR}/${CPYTHON_BIN_NAME}")
    endif()
endmacro()

macro(CREATE_CPYTHON_PROJECT)
    # creates the external project for building CPython
    include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)
    set(CPYTHON_BUILD_TYPE_FLAG "")  # blank indicates release build
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CPYTHON_BUILD_TYPE_FLAG " -d ")  # override to debug mode if applicable
    endif()
    # Add cpython as an external project that will be included in the build
    if(MSVC)
        if(CMAKE_CL_64)
            set(CPYTHON_PLATFORM x64)
            set(CPYTHON_BUILD_DIR amd64)
        else()
            set(CPYTHON_PLATFORM x86)
            set(CPYTHON_BUILD_DIR win32)
        endif()
        ExternalProject_Add(CPYTHON
                DOWNLOAD_COMMAND ""
                SOURCE_DIR ${CPYTHON_DIR}
                CONFIGURE_COMMAND ""
                BUILD_COMMAND cd ${CPYTHON_DIR} && cmd /C ${CPYTHON_DIR}/PCbuild/build.bat ${CPYTHON_BUILD_TYPE_FLAG} -p ${CPYTHON_PLATFORM}
                BUILD_IN_SOURCE TRUE
                INSTALL_COMMAND ""
                TEST_COMMAND ""
                )
    else()
        ExternalProject_Add(CPYTHON
                DOWNLOAD_COMMAND ""
                # SOURCE_DIR ${CPYTHON_DIR}
                CONFIGURE_COMMAND cd ${CPYTHON_DIR} && ./configure --enable-shared # --enable-optimizations
                BUILD_COMMAND cd ${CPYTHON_DIR} && make -j 4
                INSTALL_COMMAND ""
                TEST_COMMAND ""
                )
    endif()
endmacro()

macro(CREATE_CPYTHON_LIBRARY)
    # Creates a library target for cpython that depends on the python shared object that is built via external project
    add_library(cpython_library SHARED IMPORTED)
    set_property(TARGET cpython_library PROPERTY IMPORTED_LOCATION ${CPYTHON_BUILT_BIN})
    add_dependencies(cpython_library CPYTHON)
endmacro()

macro(CPYTHON_POST_EXE_BUILD_OPERATIONS)
    add_custom_command(
            TARGET energyplus
            POST_BUILD
            COMMAND ${CMAKE_COMMAND}
            -E copy_directory ${CPYTHON_STDLIB_DIR} $<TARGET_FILE_DIR:energyplus>/python_standard_lib
    )
    # Then also copy python standard library built modules into the standard library folder
    file(GLOB MODULES ${PROJECT_SOURCE_DIR}/third_party/CPython/build/lib*/*)  # TODO: Verify this on Windows/Mac
    foreach(MODULE IN LISTS MODULES)
        message("Copying module: ${MODULE}")
        add_custom_command(
                TARGET energyplus
                POST_BUILD
                COMMAND ${CMAKE_COMMAND}
                -E copy "${MODULE}" $<TARGET_FILE_DIR:energyplus>/python_standard_lib
        )
    endforeach()
    if(APPLE)
        add_custom_command(
                TARGET energyplus
                POST_BUILD
                DEPENDS
                __ALWAYSRUNME
                COMMAND ${CMAKE_INSTALL_NAME_TOOL} -id "@executable_path/libpython3.11.dylib" "${CPYTHON_BUILT_BIN}"
        )
        add_custom_command(
                TARGET energyplus
                POST_BUILD
                DEPENDS
                __ALWAYSRUNME
                COMMAND ${CMAKE_COMMAND} -E copy "${CPYTHON_BUILT_BIN}" $<TARGET_FILE_DIR:energyplus>
        )
        add_custom_command(
                TARGET energyplusapi
                POST_BUILD
                DEPENDS
                __ALWAYSRUNME
                COMMAND ${CMAKE_INSTALL_NAME_TOOL} -change "/usr/local/lib/libpython3.11.dylib" "@loader_path/libpython3.11.dylib" $<TARGET_FILE:energyplusapi>
        )
        add_custom_command(
                TARGET energyplus
                POST_BUILD
                DEPENDS
                __ALWAYSRUNME
                COMMAND ${CMAKE_INSTALL_NAME_TOOL} -change "/usr/local/lib/libpython3.11.dylib" "@executable_path/libpython3.11.dylib" $<TARGET_FILE:energyplus>
        )
    endif()
endmacro()