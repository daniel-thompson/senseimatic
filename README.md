senseimatic - protothreaded sensor drivers
==========================================

This library contains I2C sensor drivers that work well with the
protothread implementation found in [librfn][1].

The library is provided with a build system that allows the sensor code
to be developed and tested on a GNU/Linux workstation (or any other
Unix-like OS that supports the Linux i2c-dev interface. Testing is
performed primarily using an [i2c-star USB to I2C bridge][2], and
more rarely using the I2C adapters built into SBCs.

Note that the supplied build system is intended **only** for testing; the 
kernel typically already provides drivers for I2C sensor peripherals
(mostly in the iio sub-system) and these drivers would normally be
preferred to the ones in the library. Instead the main use of the
library is for integrating into protothreaded micro-controller
firmwares.

[1] https://github.com/daniel-thompson/librfn
[2] https://github.com/daniel-thompson/i2c-star
