The SIDEKIQ OOT Module
Basic implementation of source and sink blocks for Sidekiq radios
- tags: sdr, sidekiq, skiq, epiq
- copyright_owner: US Naval Research Lab, Epiq Solutions
- repo: https://github.com/epiqsolutions/gr-sidekiq
- website: https://epiqsolutions.com/ 

---
Provides receive and transmit functionality for Epiq Solution's Sidekiq radios via
GNU radio.  Note: availability of libsidekiq is required for use.

---
To Build and Install (from the gr-sidekiq directory)
  1) Configure the environment to build
      > source configure_env.sh
  2) Create a build directory
      > mkdir build
  3) Change to the build directory
      > cd build
  4) Run cmake 
     (NOTE: if you wish to install to a specific directory, 
          run with -DCMAKE_INSTALL_PREFIX)
      > cmake ../
           OR
      > cmake -DCMAKE_INSTALL_PREFIX=/my_install_directory ../
  5) Build the code
      > make
  6) Install it
      > sudo make install
  7) Update dynamic linker for new library (Linux only)
      > sudo ldconfig
  8) Refer to examples of sink/source blocks located in examples
