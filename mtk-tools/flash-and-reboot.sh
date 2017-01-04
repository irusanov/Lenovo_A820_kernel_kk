#!/bin/bash
#Stop script if something is broken
set -e

adb kill-server

adb push boot.img /sdcard/proton/boot.img
adb shell busybox dd if=/sdcard/proton/boot.img of=/dev/bootimg
adb reboot
