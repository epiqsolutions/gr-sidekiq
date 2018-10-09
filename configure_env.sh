#! /bin/sh
# This script configures the environment to build gr-sidekiq

# If no SDK directory specified, use the default
if [ "$SIDEKIQ_SDK" = "" ]
then
    SIDEKIQ_SDK=~/sidekiq_sdk_current
fi

export SIDEKIQ_SDK

# Assuming standard path for Epiq libraries
export EPIQ_LIBS=/usr/lib/epiq
# Copy the Sidekiq library to lib/ (assuming the x86_64 BUILD_CONFIG)
if [ -f lib/libsidekiq.a ]
then
   rm lib/libsidekiq.a
fi
ln -s $SIDEKIQ_SDK/lib/libsidekiq__x86_64.gcc.a lib/libsidekiq.a

