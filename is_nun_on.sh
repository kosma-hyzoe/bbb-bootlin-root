#!/bin/sh

i2cdetect -y -r 1 | grep -E '52|UU' > /dev/null && echo "yes!" || echo "nope"
