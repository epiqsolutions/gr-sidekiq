# SPDX-License-Identifier: GPL-3.0-or-later
find_package(Gnuradio "3.10" REQUIRED COMPONENTS blocks)
find_package(Boost REQUIRED COMPONENTS unit_test_framework)
find_package(Threads REQUIRED)
find_package(Volk REQUIRED)
set(SIDEKIQ_QA_SDK_INCLUDE_DIR "" CACHE PATH "Directory containing Sidekiq SDK headers (no library required)")
if(NOT EXISTS "${SIDEKIQ_QA_SDK_INCLUDE_DIR}/sidekiq_api.h")
    message(FATAL_ERROR "Set -DSIDEKIQ_QA_SDK_INCLUDE_DIR=/path/to/sdk/sidekiq_core/inc. QA never links libsidekiq.")
endif()
message(STATUS "QA ONLY: fake Sidekiq backend; no radio access and no install targets")
add_library(sidekiq_qa_blocks STATIC
    ../lib/sidekiq_session.cc
    ../lib/sidekiq_handle_utils.cc
    ../lib/sidekiq_tx_impl.cc
    ../lib/sidekiq_rx_impl.cc
    fake_sidekiq.cc)
target_compile_features(sidekiq_qa_blocks PUBLIC cxx_std_17)
target_include_directories(sidekiq_qa_blocks PUBLIC
    ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR} ${SIDEKIQ_QA_SDK_INCLUDE_DIR})
target_link_libraries(sidekiq_qa_blocks PUBLIC
    gnuradio::gnuradio-runtime gnuradio::gnuradio-blocks Volk::volk Threads::Threads)
include(GrTest)
set(GR_TEST_TARGET_DEPS sidekiq_qa_blocks)
set(GR_TEST_ENVIRONS "UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
GR_ADD_CPP_TEST(qa_sidekiq qa_sidekiq.cc)
set_tests_properties(qa_sidekiq PROPERTIES TIMEOUT 30 LABELS "qa;hardware-independent")
