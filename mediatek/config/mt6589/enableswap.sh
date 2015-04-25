#!/bin/sh
echo 268435456 > /sys/block/zram0/disksize
/system/bin/tiny_mkswap /dev/block/zram0
/system/bin/tiny_swapon /dev/block/zram0
echo 60 > /proc/sys/vm/swappiness
