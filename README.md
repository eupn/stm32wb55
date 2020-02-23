# `stm32wb55`

[![Build
Status](https://travis-ci.org/eupn/stm32wb55.svg?branch=master)](https://travis-ci.org/eupn/stm32wb55)


This is an implementation of [bluetooth-hci] for [STM32WB5x], based on [BlueNRG]
reference implementation of [bluetooth-hci]. 

It's using an [`embedded-hal` implementation] for low-level interaction.

This crate provides the vendor-specific Bluetooth HCI for STMicro's [STM32WB5x]
family of wireless SoC. It extends [bluetooth-hci] with vendor-specific commands and events (and associated errors).

## Work in Progress

This crate is a work-in-progress. Thanks to [STM32WB5x] BLE stack sharing majority of HCI command set with
the existing [BlueNRG], this crate in its current state is already quite usable.

The essence of the work that should be done is to find differences in details of HCI command set implementation
and fixing them.

[`embedded-hal` implementation]: https://github.com/eupn/stm32wb-hal
[STM32WB5x]: https://www.st.com/en/microcontrollers-microprocessors/stm32wbx5.html
[BlueNRG]: https://github.com/danielgallagher0/bluenrg
[bluetooth-hci]: https://github.com/danielgallagher0/bluetooth-hci