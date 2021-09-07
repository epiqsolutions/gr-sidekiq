########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

list(APPEND sidekiq_sources
        sidekiq_rx_impl.cc
        sidekiq_tx_impl.cc
        sidekiq_base.cc
        tx_burst_test_impl.cc)

set(sidekiq_sources "${sidekiq_sources}" PARENT_SCOPE)
if (NOT sidekiq_sources)
    MESSAGE(STATUS "No C++ sources... skipping lib/")
    return()
endif (NOT sidekiq_sources)

add_library(gnuradio-sidekiq SHARED ${sidekiq_sources})
message(WARNING "Searching for libsidekiq in ${CMAKE_SOURCE_DIR}/lib...")
find_library(SIDEKIQ NAMES sidekiq PATHS ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(gnuradio-sidekiq gnuradio::gnuradio-runtime ${SIDEKIQ} glib-2.0 usb-1.0)

target_include_directories(gnuradio-sidekiq
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    PUBLIC $<INSTALL_INTERFACE:include>
  )
  
set_target_properties(gnuradio-sidekiq PROPERTIES DEFINE_SYMBOL "gnuradio_sidekiq_EXPORTS")

if (APPLE)
    set_target_properties(gnuradio-sidekiq PROPERTIES
            INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
            )
endif (APPLE)

########################################################################
# Install built library files
########################################################################
include(GrMiscUtils)
GR_LIBRARY_FOO(gnuradio-sidekiq)

########################################################################
# Build and register unit test
########################################################################
include(GrTest)

include_directories(
        sidekiq_sources
        $ENV{SIDEKIQ_SDK}/sidekiq_core/inc)

list(APPEND test_sidekiq_sources
        ${CMAKE_CURRENT_SOURCE_DIR}/test_sidekiq.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/qa_sidekiq.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/qa_tx_burst_test.cc
        )

add_executable(test-sidekiq ${test_sidekiq_sources})

target_link_libraries(
        test-sidekiq
        ${GNURADIO_RUNTIME_LIBRARIES}
        gnuradio-blocks
        gnuradio-pmt
        gnuradio-filter
        ${Boost_LIBRARIES}
        ${CPPUNIT_LIBRARIES}
        ${SIDEKIQ}
        gnuradio-sidekiq
)

GR_ADD_TEST(test_sidekiq test-sidekiq)

########################################################################
# Print summary
########################################################################
message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Building for version: ${VERSION} / ${LIBVER}")

