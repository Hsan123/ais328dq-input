Drivers files expected to be placed at:
<KERNEL>/drivers/input/misc/ais328dq_acc.c
<KERNEL>/include/linux/input/ais328dq.h

Add following line to the end of <KERNEL>/drivers/input/misc/Makefile

obj-$(CONFIG_INPUT_AIS328DQ)		+= ais328dq_acc.o


Add a configuration entry into in <KERNEL>/drivers/input/misc/Kconfig
file like the following:

......
config INPUT_AIS328DQ
	tristate "STM MEMS Accelerometer AIS328DQ"
	depends on INPUT
	depends on I2C && SYSFS
	help
	  Say Y here if you have STM ais328dq acc connected to I2C bus;

	  To compile this driver as a module, choose M here: the
	  module will be called ultra.

.......