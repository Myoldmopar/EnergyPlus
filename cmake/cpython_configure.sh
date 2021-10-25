#!/bin/bash

# shellcheck disable=SC2164
cd "$1"
./configure --enable-shared --enable-optimizations # LDFLAGS="-static" --disable-shared
