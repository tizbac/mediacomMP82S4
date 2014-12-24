mediacomMP82S4
==============

My work to compile a working linux kernel for various chinese rebranded tablets , like Mediacom Smartpad S4 8.0 , Prestigio PMP5785c and Yarvik Noble 7.85"

To compile:
-----------

Fetch Google's ARM Toolchain
git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/arm/arm-eabi-4.6

Put the correct path to the toolchain in build.sh

Run ./build.sh

And finally

rkutils/rkcrc -k kernel/arch/arm/boot/Image kernel.img

Now you are ready to flash the kernel image using rkflashkit, and enjoy the black screen (i'll get it working i hope)

