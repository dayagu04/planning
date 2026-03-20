#!/bin/bash

cppcheck --xml --xml-version=2 --enable=all -isrc/thirdparty -ithirdparty ./src 2> cppcheck-report.xml
