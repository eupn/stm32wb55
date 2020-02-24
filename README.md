# `stm32wb55`

[![Build
Status](https://travis-ci.org/eupn/stm32wb55.svg?branch=master)](https://travis-ci.org/eupn/stm32wb55)


This is an implementation of [bluetooth-hci] for [STM32WB5x], based on [BlueNRG]
reference implementation of [bluetooth-hci]. 

It's using an [`embedded-hal` implementation] for low-level interaction.

This crate provides the vendor-specific Bluetooth HCI for STMicro's [STM32WB5x]
family of wireless SoC. It extends [bluetooth-hci] with vendor-specific commands and events (and associated errors).

## Examples

* [transparent_mode.rs] - a firmware for STM32WB55 MCU that can be uploaded to USB dongle and used with [STM32CubeMon-RF] software.
* [eddystone_beacon.rs] - an implementation of [Eddystone URL beacon] that broadcasts
[https://www.rust-lang.com](https://www.rust-lang.com) URL.
* [ibeacon.rs] - an implementation of [Apple iBeacon] that broadcasts custom UUID.

## Work in Progress

This crate is a work-in-progress. Thanks to [STM32WB5x] BLE stack sharing majority of HCI command set with
the existing [BlueNRG], this crate in its current state is already quite usable.

The essence of the work that should be done is to find differences in details of HCI command set implementation
and fixing them.

[transparent_mode.rs]: examples/transparent_mode.rs
[ibeacon.rs]: examples/ibeacon.rs
[eddystone_beacon.rs]: examples/eddystone_beacon.rs
[`embedded-hal` implementation]: https://github.com/eupn/stm32wb-hal
[STM32WB5x]: https://www.st.com/en/microcontrollers-microprocessors/stm32wbx5.html
[BlueNRG]: https://github.com/danielgallagher0/bluenrg
[bluetooth-hci]: https://github.com/danielgallagher0/bluetooth-hci
[Eddystone beacon]: https://developers.google.com/beacons/eddystone
[Apple iBeacon]: https://developer.apple.com/ibeacon/
[STM32CubeMon-RF]: https://www.st.com/en/development-tools/stm32cubemonrf.html
