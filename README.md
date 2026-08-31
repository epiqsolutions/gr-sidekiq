# gr-sidekiq

GNU Radio out-of-tree module that provides Sidekiq source and sink blocks for
Epiq Solutions radios.

## Requirements

- GNU Radio 3.10 or newer
- CMake 3.8 or newer
- Sidekiq SDK v 4.26.0 or newer

The build locates the Sidekiq SDK in this order:

1. `-DSIDEKIQ_SDK_DIR=/path/to/sdk`
2. `SIDEKIQ_SDK_DIR=/path/to/sdk`
3. `$HOME/sidekiq_sdk_current`

If you are using the Python bindings or GNU Radio Companion blocks, build in an
environment where the GNU Radio Python components are available.

## Setup
Check the version of gnuradio available in your distro:
```bash
 apt-cache policy gnuradio
```

If it's not 3.10 or later, then add a PPA to get the latest version:
```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository ppa:gnuradio/gnuradio-releases
sudo apt update
```

Install build tools and gnuradio:
```bash
sudo apt install build-essential cmake gnuradio
```

## Build And Install

From the repository root:

```bash
cmake -S . -B build
cmake --build build
sudo cmake --install build
sudo ldconfig
```

## Examples

The [`examples/`](examples/) directory contains basic receive, transmit, dual
channel, timestamp, message-control, and burst-mode flowgraphs.
