#!/bin/bash

set -ex

cd /workspace/build/kortex_hardware/
#Check whether files match format
make check-format