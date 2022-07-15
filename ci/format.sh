#!/bin/bash

files=$(find $1 -type f -name "*.cpp" -or -name "*.h" -or -name "*.hpp" -or -name "*.cxx")

clang-format -Werror --verbose -i --style=file ${files}

git diff --exit-code --stat
