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

Expected result: **1 CTest test passes**, containing **42 Boost.Test cases**.
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
failure. The `calibration` suite checks selected-handle execution, manual/auto/off
trigger gating, preservation of requested calibration subsets, per-handle
capabilities, unsupported requests, and SDK read/write/run errors. These tests
do not validate RF calibration quality or timed transmission.

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

## Calibration validation

Run only these cases with:

```bash
build/qa/tests/qa_sidekiq --run_test=calibration --log_level=test_suite
```

RX calibration masks are resolved independently for each selected handle as
`requested & available`. A request for DC offset alone never enables quadrature
calibration. Requesting Both on a handle that supports only one selects that
supported type with a warning. If no requested type is supported, or capability
lookup fails, the setter reports an error before writing any masks. A later SDK
write failure can leave an earlier handle configured; there is no transactional
rollback across handles. Correct the error and reapply the configuration.

For hardware validation after step 6, use single- and dual-source flowgraphs on
the M.2. Check Auto and Manual modes, select each supported calibration type,
and trigger Run Cal in Manual mode. Verify that both selected handles are
calibrated and that mask readback matches the supported requested subset using
the SDK. Check SDK errors and RF DC/image performance before and after calibration.
The fake tests verify calls and masks, not the analog effectiveness of calibration.
The existing Off option suppresses OOT-module calibration configuration/triggers;
this change does not redefine it as disabling all calibration inside the radio.

## C++ cleanup and SDK lifetime

The `sdk_lifetime` suite checks RX/TX destruction in both orders, cleanup after
constructor failure, sharing an externally initialized SDK, adding another card,
failure while enabling that card, concurrent lease acquisition, and a failed
constructor while another block remains alive. Card 1 is modeled only for the
session enable call; these are not full multi-card RF tests. `command_refactor`
checks pair/dictionary command forwarding through the actual blocks and that
invalid message shapes remain ignored.

The module now keeps libsidekiq alive until its last RX/TX block is destroyed.
Additional cards are enabled through `skiq_enable_cards`, rather than repeated
SDK initialization. Cards stay enabled until that shared lifetime ends. When
libsidekiq was initialized externally, the module borrows it and never calls
`skiq_exit`; the external owner must keep the SDK and required cards initialized.
Only module lease acquisition/release is serialized. This does not arbitrate
conflicting rate, topology, timestamp-reset, or channel settings among blocks.

Command keys and defaults, constructor signatures, GRC parameters, and sample
conversion rules are unchanged. Implementation constants and shared PMT helpers
are scoped, repeated RX configuration paths use handle loops, and constructor/
underrun output uses GNU Radio logging. The production target explicitly requires
C++17 and links its thread and VOLK dependencies. No generated example Python
files are part of this change.

After step 6, perform the TX, burst, RX, and calibration hardware checks above
before starting timed TX development. Also exercise combined RX/TX flowgraphs,
repeat construction/destruction in both orders, and confirm stopping/removing one
block leaves the other usable. Measure sustained throughput at the intended
sample rates. SDK shutdown failures are logged; cleanup cannot guarantee recovery
if the real SDK fails to release hardware.
