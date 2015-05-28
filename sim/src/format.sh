#!/bin/sh
exec clang-format -style=file -i $(find . -name "*.h" -or -name "*.cpp")
