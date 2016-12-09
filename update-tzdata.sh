#!/bin/bash
#Stop script if something is broken
set -e

gpg --recv-keys ED97E90E62AA7E34
python2 bionic/libc/tools/zoneinfo/update-tzdata.py