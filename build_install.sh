
#!/bin/bash

set -x
set -e

#gr_modtool bind sidekiq_tx
#gr_modtool bind sidekiq_rx

sudo rm -r -f build

mkdir build
cd build

cmake ../

make -j8

sudo make install

sudo make uninstall

sudo make install

sudo ldconfig
