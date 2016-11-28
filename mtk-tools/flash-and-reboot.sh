#!/bin/bash
#Stop script if something is broken
set -e

adb kill-server
adb push boot.img /sdcard/a820/
adb shell busybox dd if=/sdcard/a820/boot.img of=/dev/bootimg
adb reboot
