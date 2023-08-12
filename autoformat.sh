#!/bin/bash

cd "$(dirname "$0")"
set -e

source_files=`find libminiscope/ src/ py/ -type f \( -name '*.c' -o -name '*.h' -o -name '*.cpp' -o -name '*.hpp' \)`

clang-format -i $source_files
