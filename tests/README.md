# Hardware-independent GNU Radio QA

This suite builds the **actual repository RX/TX C++ blocks** with a test-only
Sidekiq implementation, then connects them to GNU Radio Vector Source, Head,
and Vector Sink blocks. It uses GNU Radio's `GR_ADD_CPP_TEST` integration with
Boost.Test and CTest. Sidekiq SDK headers are required to compile the suite,
but the compiled libsidekiq library is not linked. No driver, radio, GUI, module
installation, or root access is needed.

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

From the repository root, use a compatible SDK installed at
`~/sidekiq_sdk_current`, or substitute your SDK header directory:

```bash
cmake -S . -B build/qa \
  -DSIDEKIQ_QA_ONLY=ON \
  -DSIDEKIQ_QA_SDK_INCLUDE_DIR="$HOME/sidekiq_sdk_current/sidekiq_core/inc"
cmake --build build/qa -j4
ctest --test-dir build/qa --output-on-failure
```

Expected result: **1 CTest test passes**, containing **27 Boost.Test cases**.
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
- Fake TX stop preserves streaming state on an injected SDK failure and cancels
  only the selected handle's pending transfers on success.

The TX safety regressions also cover deferred buffer ownership, queue-full retry
without a pending callback, cancellation while the pool is full, and complete
A2 dual-channel payload allocation. Pool-level tests cover late callback lifetime,
completion errors, and restart protection. A2 input occupies the secondary payload;
the paired primary payload contains zeros.

The `tx_bursts` suite covers exact tag offsets, adjacent and separated bursts,
fragmentation across scheduler calls, input shorter than one SDK packet, zero
padding after buffer reuse, queue-full retry, and waiting for async completion.
It also covers stopping during that wait, signed integer lengths, unrelated tags,
and restarting after an incomplete burst. Assertions compare both I and Q samples
against the tagged input ranges and check packet/start/stop counts.

The `rx_correctness` suite exercises uneven handle arrivals (12 A1 packets then
12 A2 packets), compares every output I/Q sample, and checks each timestamp at
its packet's first sample. It also checks idle shutdown, discontinuity diagnostics
across work calls, restart with a new timestamp epoch, and retry after an SDK stop
failure. Calibration defects remain for a subsequent branch. These tests do not
validate timed transmission.

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
- RX scripts contain full unpacked packets and replay cyclically by default so a source can
  return from `work()` while a downstream Head stops the flowgraph. Supply enough
  distinct packets for the asserted output prefix. Timestamps repeat on replay;
  this is not a model of a continuously advancing hardware clock. Pass
  `repeat=false` to return no-data after the script ends. With no script, RX can
  start but returns no-data, modeling an idle receiver or pending trigger.
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
  -DSIDEKIQ_QA_SDK_INCLUDE_DIR="$HOME/sidekiq_sdk_current/sidekiq_core/inc" \
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

## Existing burst interface: hardware validation

Build and install the normal module with `SIDEKIQ_QA_ONLY=OFF` and your real SDK;
the QA build cannot access a card. Open `examples/bursting.grc` in GNU Radio
Companion and regenerate it from the GRC source. Select the correct card and A1,
a supported sample rate, and a suitable RF frequency/attenuation. Use a receiver
or analyzer connected through appropriate attenuation to capture the TX output.
The example's GUI sinks observe the input waveform, not the radio output.

The Tags Strobe key must match the TX sink's Bursting Tag Name (`tx_burst`). Its
value is the number of complex samples to transmit; its interval must be at least
that length. Start with Threads = 1, then repeat with Threads = 2 and 4.

With A1 Buffer Size = 4092, try lengths 4092, 4093, and 8201. Expect respectively
1, 2, and 3 SDK packets, with 0, 4091, and 4075 trailing zero IQ samples. Capture
the burst ending to check for truncation or stale waveform after the valid data.
Run repeated bursts and stop/restart while active; check for hangs and SDK errors.
For a stronger data-integrity check, use a changing known waveform and compare
captured samples after accounting for receiver delay, gain, and phase.

The stream tag offset selects input samples, not a wall-clock transmission time.
Input gaps are discarded, and each burst starts in immediate mode. The example's
Throttle paces input but cannot provide precise RF scheduling. Async completion
means the host buffer can be released; it does not prove the final sample has
aired. This branch waits for those callbacks before normal burst stop, but actual
RF tail delivery still requires hardware validation. Explicit flowgraph stop
remains an abort. If input ends before the declared burst length, any unfinished
packet is discarded; full packets already submitted cannot be recalled.

Zero, negative, noninteger, duplicate-at-one-offset, and overlapping burst tags
are rejected with an error rather than replacing an active burst silently.
No `tx_time`, SOB/EOB interface, or timestamp mode is introduced here.

## Normal burst completion versus explicit stop

The `burst_completion` suite verifies that normal length-tag burst completion
waits for all async transfer callbacks before stopping the SDK, that synchronous
TX releases its buffer reservation before the wait, and that explicit flowgraph
stop interrupts the wait and cancels pending packets. Both I/Q payloads are checked
for completed transfers. Run these cases with:

```bash
build/qa/tests/qa_sidekiq --run_test=burst_completion --log_level=test_suite
```

A completed callback permits reuse of the host buffer; it does not establish that
the final sample has aired. Check RF tail delivery on hardware. Timed TX is not
implemented in this branch.

## RX correctness: hardware validation

Use the normal build linked to libsidekiq. Regenerate `examples/source_test.grc`
for single-channel checks and `examples/dual_source.grc` for two supported RX
handles. Select the actual card, frequency, gain, and supported sample rate.
Feed known signals into the RX ports and check each output for the expected tone
and amplitude, then repeat start/stop and test at the intended operating rate.

Enable timestamp tags and connect a Tag Debug block to each output (filter key
`rf_timestamp`). Each tag describes the first sample of its SDK packet and uses
that output's absolute GNU Radio sample offset. With continuous reception, RF
timestamps advance by the packet's complex sample count. Deliberately interrupt
reception or cause a hardware overrun and verify that any timestamp gap is
reported; the source does not insert replacement samples or realign channels.

For dual RX, capture identifiable sample sequences from both handles and check
for missing or duplicated sections. Independent production preserves arrival
order within each handle; it does not promise that equal output offsets represent
the same RF time. Downstream processing must use the timestamps if alignment is
required. GNU Radio buffers remain finite, so a stalled downstream consumer can
still cause hardware overruns. This branch does not add an unlimited RX queue.

If using a triggered start, also verify that stopping while awaiting data/trigger
returns promptly. Start retains the existing SDK timestamp-reset behavior; verify
its effect in combined RX/TX flowgraphs. Hardware throughput, coherence, and SDK
blocking behavior are not established by the fake tests.
