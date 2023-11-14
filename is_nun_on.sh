#!/bin/sh

i2cdetect -y -r 1 | grep 52 > /dev/null && echo "yes!" || echo "nope"
