#!/bin/bash

# shellcheck disable=SC2164
cd "$1"
make # LDFLAGS="-static" LINKFORSHARED=" " -j 4
