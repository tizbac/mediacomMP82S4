#!/bin/bash
  
export ARCH=arm
export CROSS_COMPILE=/home/tiziano/arm-eabi-4.6/bin/arm-eabi-
export LOCALVERSION=
export INSTALL_MOD_PATH=$HOME/src/rk3188/mod_fw
MAKE="make -j6"
  
$MAKE

$MAKE bzImage

rm -rf $INSTALL_MOD_PATH
$MAKE modules
$MAKE modules_install


