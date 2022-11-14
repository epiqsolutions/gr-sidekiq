
#!/bin/bash

set -x

gr_modtool bind sidekiq_tx
sudo rm -r -f build

mkdir build
cd build

cmake ../

make 

sudo make install

sudo make uninstall

sudo make install

