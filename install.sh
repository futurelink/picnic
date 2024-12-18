#!/bin/sh

#addgroup gpio
#usermod -aG gpio linuxcnc

cp -R ./etc /
udevadm control --reload-rules && udevadm trigger

cp build/picnic.so /usr/lib/linuxcnc/modules
