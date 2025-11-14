.. _at_start_f437_board:

AT-START-F437
################

Overview
********

The AT START F405 board features an ARM Cortex-M4 based AT32F405 MCU
with a wide range of connectivity support and configurations. 

Hardware
********

- ARM Cortex-M4F Processor
- Core clock up to 288 MHz
- 4032KB Flash memory
- 384 KB SRAM
- 3x12-bit 5MSPS ADC
- Up to 6x USART and 2x UART
- Up to 3x I2C
- Up to 3x SPI
- 1x QSPI interface
- 2x CAN interface(2.0B Active)
- 2x OTGFS Support usb2.0 Full speed
- Up to 14 times

Supported Features
==================

The following features are supported:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| ADC       | on-chip    | adc controller                      |
+-----------+------------+-------------------------------------+
| CLOCK     | on-chip    | reset and clock control             |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+
| I2C       | on-chip    | i2c port/controller                 |
+-----------+------------+-------------------------------------+
| SPI       | on-chip    | SPI port/controller                 |
+-----------+------------+-------------------------------------+
| PINMUX    | on-chip    | pinmux                              |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port-polling;                |
|           |            | serial port-interrupt               |
+-----------+------------+-------------------------------------+
| WDT       | on-chip    | watchdog                            |
+-----------+------------+-------------------------------------+

The default configuration can be found in the defconfig file:
:zephyr_file:`boards/at/at_start_f405/at_start_f405_defconfig`


Programming and Debugging
*************************


Flashing
========


Debugging
=========

Use SWD with a AT-LINK/J-Link

References
**********

.. _microbit website: https://www.arterychip.com/en/product/AT32F405.jsp