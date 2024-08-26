cmake_minimum_required(VERSION 3.15)
project(OrbLidarSDK VERSION 1.0.0 LANGUAGES CXX C)

# Set the C++ and C standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# Set compile flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")
set(CMAKE_BUILD_TYPE "Debug")

# Set third-party library paths
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
set(MCAP_INCLUDE_DIR ${THIRD_PARTY_DIR}/mcap/cpp/mcap/include)

# Add third-party libraries
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
set(BUILD_UVW_LIBS ON)
set(BUILD_TESTING OFF CACHE BOOL "Disable testing for uvw" FORCE)

add_subdirectory(${THIRD_PARTY_DIR}/spdlog)
add_subdirectory(${THIRD_PARTY_DIR}/uvw)

# Options
option(BUILD_EXAMPLES "Build examples" ON)
option(ENABLE_TESTING "Enable testing" ON)
option(ENABLE_SANITIZERS "Enable sanitizers" OFF)

# Gather source files
file(GLOB SRC_FILES src/*.cpp src/detail/*.cpp)

add_library(${PROJECT_NAME} SHARED
        ${SRC_FILES}
        ${THIRD_PARTY_DIR}/FastCRC/src/FastCRCsw.cpp
)

# Find dependencies
find_package(lz4 QUIET)
find_package(zstd QUIET)
find_package(Threads REQUIRED)

set(DEPENDENCIES spdlog uvw::uvw-static)

# Handle optional dependencies
if (lz4_FOUND)
    list(APPEND DEPENDENCIES LZ4::lz4)
else()
    message(WARNING "Dependency LZ4 not found, disabling")
    target_compile_definitions(${PROJECT_NAME} PUBLIC "MCAP_COMPRESSION_NO_LZ4")
endif()

if (zstd_FOUND)
    list(APPEND DEPENDENCIES zstd::libzstd)
else()
    message(WARNING "Dependency zstd not found, disabling")
    target_compile_definitions(${PROJECT_NAME} PUBLIC "MCAP_COMPRESSION_NO_ZSTD")
endif()

# Enable sanitizers if the option is set
if (ENABLE_SANITIZERS)
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address,undefined)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address,undefined)
endif()

# Include directories
target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/contrib>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/detail>
        ${THIRD_PARTY_DIR}/FastCRC/src
        ${MCAP_INCLUDE_DIR}
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ${DEPENDENCIES}
        Threads::Threads
)

# Add examples if enabled
if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Add tests if enabled
if (ENABLE_TESTING)
    set(CMAKE_CTEST_ARGUMENTS "--verbose")
    add_subdirectory(${THIRD_PARTY_DIR}/googletest)
    enable_testing()
    add_subdirectory(tests)
endif()

# Install targets
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
