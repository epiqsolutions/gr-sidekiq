
#!/bin/bash

set -x
set -e

#gr_modtool bind sidekiq_tx
#gr_modtool bind sidekiq_rx

sudo rm -r -f build

mkdir build
cd build

if [ -n "$1" ]; then
    cmake ../ -D$1
else
    cmake ../
fi

make -j8

sudo make install

sudo make uninstall

sudo make install

sudo ldconfig
