cmake_minimum_required(VERSION 3.15)
project(orbbec_lidar_sdk VERSION 1.0.0 LANGUAGES CXX C)

# Set C++ and C standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set compile flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build (Debug or Release)")

# Define project options
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_EXAMPLES "Build examples" ON)
option(ENABLE_TESTING "Enable testing" ON)
option(ENABLE_SANITIZERS "Enable sanitizers" OFF)

# Set paths
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

# Add third-party libraries
set(BUILD_UVW_LIBS ON)
set(BUILD_TESTING OFF CACHE BOOL "Disable testing for uvw" FORCE)

add_subdirectory("${THIRD_PARTY_DIR}/spdlog" EXCLUDE_FROM_ALL)
add_subdirectory("${THIRD_PARTY_DIR}/uvw" EXCLUDE_FROM_ALL)
add_subdirectory("${THIRD_PARTY_DIR}/memory" EXCLUDE_FROM_ALL)

# Find dependencies
find_package(Threads REQUIRED)
find_package(LZ4 QUIET)
find_package(ZSTD QUIET)

# Gather source files
file(GLOB_RECURSE SRC_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/detail/*.cpp"
)

# Create library target
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})

# Set include directories
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/contrib>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/detail>
        $<BUILD_INTERFACE:${THIRD_PARTY_DIR}/mcap/cpp/mcap/include>
        $<INSTALL_INTERFACE:include>
)

# Set dependencies
set(DEPENDENCIES
        spdlog::spdlog
        uvw::uvw-static
        foonathan_memory
        Threads::Threads
)

# Handle optional dependencies
if(LZ4_FOUND)
    list(APPEND DEPENDENCIES LZ4::LZ4)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC MCAP_COMPRESSION_NO_LZ4)
    message(WARNING "LZ4 not found, disabling LZ4 support")
endif()

if(ZSTD_FOUND)
    list(APPEND DEPENDENCIES ZSTD::ZSTD)
else()
    target_compile_definitions(${PROJECT_NAME} PUBLIC MCAP_COMPRESSION_NO_ZSTD)
    message(WARNING "ZSTD not found, disabling ZSTD support")
endif()

# Link dependencies
target_link_libraries(${PROJECT_NAME} PRIVATE ${DEPENDENCIES})

# Enable sanitizers if option is set
if(ENABLE_SANITIZERS)
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address,undefined)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address,undefined)
endif()

# Add examples if enabled
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Add tests if enabled
if(ENABLE_TESTING)
    include(CTest)
    add_subdirectory("${THIRD_PARTY_DIR}/googletest" EXCLUDE_FROM_ALL)
    add_subdirectory(tests)
endif()

# Installation rules
include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}-targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install the project's public headers
install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}-targets.cmake
        DESTINATION ${INSTALL_CONFIGDIR}
)

install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}-targets.cmake
        DESTINATION ${INSTALL_CONFIGDIR}
)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# Install the config files
install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
        DESTINATION ${INSTALL_CONFIGDIR}
)
# include uninstall target
if (NOT TARGET uninstall)
    configure_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
            IMMEDIATE @ONLY)
    add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
endif ()
