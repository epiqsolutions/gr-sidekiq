# Hardware-independent GNU Radio QA

This suite builds the **actual repository RX/TX C++ blocks** with a test-only
Sidekiq implementation, then connects them to GNU Radio Vector Source, Head,
and Vector Sink blocks. It uses GNU Radio's `GR_ADD_CPP_TEST` integration with
Boost.Test and CTest. No Sidekiq library, driver, radio, GUI, installation, or
root access is needed.

The separate `SIDEKIQ_QA_ONLY` configuration does not build or install a fake
production module. The default configuration still builds the normal module.
Do not reuse a production build directory for QA.

## Prerequisites

- CMake, a C++17 compiler, GNU Radio 3.10+ development files including gr-blocks,
  VOLK development files, and Boost.Test development files.
- Sidekiq SDK headers compatible with this repository (SDK 4.26+). Supply the
  directory containing `sidekiq_api.h`, `sidekiq_types.h`, `sidekiq_params.h`, and
  `sidekiq_xport_types.h`. Headers are not copied into this repository.
- GNU Radio development packages provide the C++ QA framework; Python QA scripts
  and generated example Python files are not used in this first suite.

## Configure, build, and test

From the repository root, substitute your SDK header directory:

```bash
cmake -S . -B build/qa \
  -DSIDEKIQ_QA_ONLY=ON \
  -DSIDEKIQ_QA_SDK_INCLUDE_DIR=/path/to/sdk/sidekiq_core/inc
cmake --build build/qa -j4
ctest --test-dir build/qa --output-on-failure
```

On the development host used to add this suite, the available compatible headers
are in `/home/dhelm/sidekiq_sw/sdk_artifacts/common_files/inc`. The default
`~/sidekiq_sdk_current` points at an older SDK and cannot compile the current
repository's topology API calls.

Expected result: **1 CTest test passes**, containing **14 Boost.Test cases**.
CTest applies a 30-second timeout so a scheduler or callback deadlock fails the
run rather than hanging indefinitely. Nonzero exit status means failure.

For detailed output or one case:

```bash
ctest --test-dir build/qa -V
build/qa/tests/qa_sidekiq --list_content
build/qa/tests/qa_sidekiq --run_test=baseline/tx_immediate_sync --log_level=test_suite
build/qa/tests/qa_sidekiq --run_test=backend --log_level=test_suite
```

CTest also sets GNU Radio's test environment (`VOLK_GENERIC=1`, preferences
loading disabled). For manual execution use the same environment if comparing
results across machines:

```bash
VOLK_GENERIC=1 GR_DONT_LOAD_PREFS=1 GR_CONF_CONTROLPORT_ON=False \
  build/qa/tests/qa_sidekiq --log_level=test_suite
```

## What is covered now

- Immediate-mode, single-channel synchronous TX: finite waveform, exact sample
  count/order, IQ scaling/conversion, configured mode, and start/stop calls.
- The same flowgraph using async TX with immediate successful callbacks.
- Single-channel RX: known samples converted to GNU Radio complex output through
  a finite Head/Vector Sink flowgraph.
- TX setters forwarding frequency, attenuation, rate/bandwidth, and calibration.
- One-shot SDK error injection: a setter reports the failure, then can succeed.
- Handle normalization, invalid names, and parameter lookup independent of order.
- Fake backend contract: deferred callbacks, capacity-based queue-full rejection,
  original buffer lifetime, captured timestamps/data, and explicit completion.
- Fake RX script validation, channel order, timestamps, and packet sizes.

The TX safety regressions also cover deferred buffer ownership, queue-full retry
without a pending callback, cancellation while the pool is full, and complete
A2 dual-channel payload allocation. Pool-level tests cover late callback lifetime,
completion errors, and restart protection. A2 input occupies the secondary payload;
the paired primary payload contains zeros.

Burst offsets/partial tails, dual-RX, timestamp tags, and calibration defects are
reserved for subsequent branches. These tests do not validate timed transmission.

Run the TX safety cases alone with:

```bash
build/qa/tests/qa_sidekiq --run_test=tx_safety --log_level=test_suite
```

For hardware validation, exercise immediate synchronous and asynchronous TX on A1
and A2 with a known tone. Check the selected output and that the paired output has
no waveform, then repeat start/stop under sustained load. Observe sample integrity,
SDK errors, and hangs. Stop cancels pending transfers; it does not promise to drain
a final queued waveform. Hardware validation remains required before release.

## Extending the fake backend

`fake_sidekiq.h` exposes reset, call history, captured TX packets, one-shot error
injection, async capacity/auto-completion controls, and an RX packet script.
All state access is serialized with a mutex; callbacks execute outside that lock.

- Only card 0 is modeled, with m.2-style A1/A2 capability entries and 12-bit IQ.
- TX captures the configured payload size, doubled for dual-channel mode. Async
  deferred mode retains the caller's pointer until `complete_one()`: callers must
  keep buffers alive and unchanged. Stop cancels pending packets with completion
  status -2. Use explicit completion before stop when asserting transmitted data.
  An injected stop failure leaves pending packets available for late completion.
- RX scripts contain full unpacked packets and replay cyclically so a source can
  return from `work()` while a downstream Head stops the flowgraph. Supply enough
  distinct packets for the asserted output prefix. Timestamps repeat on replay;
  this is not a model of a continuously advancing hardware clock.
- Most configuration functions record calls and return success; only the modeled
  modes and selected streaming preconditions are enforced. This is not a complete
  simulator or authoritative hardware validator.
- No system libsidekiq fallback exists. A newly used SDK function without a fake
  implementation produces a link error. Add its documented behavior and tests.
- Reset only after all blocks and scheduler/callback threads have been destroyed.
  Each test gets fresh fake state. Do not run cases concurrently in one process.

For a bug fix, test the desired samples/tags/SDK actions through the actual block.
Use backend-only tests to verify new injection mechanisms, not as a substitute
for exercising the block. Keep hardware tests separate and opt-in.

## Memory checks

For buffer-related changes, use a separate AddressSanitizer/UBSan build:

```bash
cmake -S . -B build/qa-asan \
  -DSIDEKIQ_QA_ONLY=ON \
  -DSIDEKIQ_QA_SDK_INCLUDE_DIR=/path/to/sdk/sidekiq_core/inc \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-omit-frame-pointer" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address,undefined"
cmake --build build/qa-asan -j4
ctest --test-dir build/qa-asan --output-on-failure
```

These flags instrument this project's blocks and fake backend; installed GNU Radio
and VOLK libraries are not rebuilt with sanitizers. Passing this suite does not
measure RF accuracy, throughput, or hardware FIFO behavior.

If LeakSanitizer reports that it cannot run under `ptrace`, run CTest in a normal
terminal outside the debugger/sandbox. This is an instrumentation restriction,
not a failed sample assertion. The development run passed with leak checking
enabled outside the sandbox.
