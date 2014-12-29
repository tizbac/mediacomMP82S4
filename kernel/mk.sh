#!/bin/bash
if [ "$1" == "" ]
then
  CFG_B=mp18
else
  CFG_B=$1
fi
INPUT=arch/arm/boot/Image
CFG_E=config
CFG_T=arch/arm/mach-rk3188/config/$CFG_B.txt
CFG_C=arch/arm/mach-rk3188/config/config.c

OUTPUT=$CFG_B.img

if [ ${CFG_B:0:2} = mk ]
then
  OUTDIR=../out68k
  CFG_KN=rk3168_yf_defconfig
else
  OUTDIR=../out88k
  axp=`grep -c "axp22_supproted = 1" $CFG_T`;
  if [ $axp == 1 ]
  then
    CFG_KN=rk3188_yf_axp_defconfig
  else
    CFG_KN=rk3188_yf_defconfig
  fi

  if [ -f $OUTDIR/.config ]; then
    AXP=`grep -c "CONFIG_KP_AXP=y" $OUTDIR/.config`;
    if [ $axp != $AXP ]; then
        rm -rf $OUTDIR/.config
    fi
  fi
fi
CFG_K=arch/arm/configs/$CFG_KN

if [ ! -d $OUTDIR ]; then
  mkdir $OUTDIR
  mkdir --parent $OUTDIR/arch/arm/mach-rk30
  mkdir --parent $OUTDIR/arch/arm/mach-rk3188
  mkdir --parent $OUTDIR/arch/arm/plat-rk
  mkdir --parent $OUTDIR/drivers/net/wireless/wifi_sys
fi
if [ ! -f $CFG_T ]; then
  echo "no config file found for $CFG_B"
  exit -1
fi
if [ $CFG_C -nt $CFG_E ]
then
  echo generating $CFG_E
  gcc $CFG_C -o $CFG_E
fi
cpu_num=`cat /proc/cpuinfo | grep "processor" | wc -l`
if [ $CFG_K -nt $OUTDIR/.config ]
then
  echo generating kernel config
  #cp $CFG_K .config
  make -j$cpu_num $CFG_KN O=$OUTDIR
fi

echo generating kernel
make Image -j$cpu_num  O=$OUTDIR
if [ $? -eq 0 ]
then
  if [[ $OUTDIR/$INPUT -nt $OUTPUT || $CFG_T -nt $OUTPUT || $CFG_E -nt $OUTPUT ]]
  then
    echo generating image
    ./$CFG_E $CFG_T $OUTDIR/$INPUT $CFG_B.kmg
    if [ ! $? -eq 0 ]; then
      echo failed to apply config
      exit -1
    fi
    cp $CFG_B.kmg $OUTDIR/$INPUT
    #build android need this
    mv $CFG_B.kmg $INPUT
    ./mkkrnlimg $INPUT $OUTPUT
    size=`stat -c%s $OUTPUT`
    if [ $size -gt 8388608 ]
    then
        echo image is too big, generating zImage
        make -j$cpu_num zImage O=$OUTDIR
        ./mkkrnlimg $OUTDIR/arch/arm/boot/zImage $OUTPUT
    fi
  fi
  rm  kernel.img >/dev/null 2>&1
  ln -s $OUTPUT kernel.img
fi
