# mmc-mailbox-driver

This is a driver for the DMMC-STAMP "mailbox" acting as a data interface between the MMC and the payload FPGA/SoC.

The "mailbox" is a 2 KByte dual port memory inside the STAMP's CPLD acting as a `at24`-like EEPROM. This driver is based on `at24.c` from the mainline Linux kernel.

## Locking

To avoid race conditions, the uppermost byte has a "lock" flag preventing the STAMP from updating the memory. This driver implements a critical section, setting the lock flag when more than one byte is read or written.

## Power off

To notify the STAMP that the Linux system on FPGA/SoC is ready to be powered off, this driver uses the Linux kernel's `pm_power_off` callback to set a "shutdown finished" flag in the mailbox.

This requires that
* No other driver is using `pm_power_off`
* The I2C driver providing access to the I2C bus towards the DMMC-STAMP mailbox supports the `master_xfer_atomic()` method (see also [i2c-xiic-atomic](https://github.com/MicroTCA-Tech-Lab/i2c-xiic-atomic))
