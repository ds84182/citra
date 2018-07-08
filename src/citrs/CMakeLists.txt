include(ExternalProject)

add_library(citrs STATIC dummy.cpp)

set_directory_properties(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/citrs)

set(SRCS
    src/lib.rs

    src/socu/errors.rs
    src/socu/hle_unpark.rs
    src/socu/mod.rs
)

# TODO: Build release
ExternalProject_Add(
    citrs-external
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND cargo build COMMAND cargo build
    # DEPENDS ${SRCS}
    BINARY_DIR "${CMAKE_CURRENT_LIST_DIR}"
    INSTALL_COMMAND ""
    LOG_BUILD ON
)

add_dependencies(citrs citrs-external)

if(WIN32)
    set(LIBS ws2_32 userenv advapi32)
endif()

target_link_libraries(citrs
    debug "${CMAKE_CURRENT_LIST_DIR}/target/debug/${CMAKE_STATIC_LIBRARY_PREFIX}citrs${CMAKE_STATIC_LIBRARY_SUFFIX}"
    optimized "${CMAKE_CURRENT_LIST_DIR}/target/release/${CMAKE_STATIC_LIBRARY_PREFIX}citrs${CMAKE_STATIC_LIBRARY_SUFFIX}"
    ${LIBS}
)
