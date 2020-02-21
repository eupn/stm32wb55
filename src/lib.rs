//! Bluetooth HCI for STMicro's STM32WB5x Bluetooth controllers.
//!
//! # Design
//!
//! The STM32WB55 is a dual-core SoC that contains application controller (Cortex-M4F) and
//! radio coprocessor (Cortex-M0+). This crate is intended to run on application controller and
//! communicates with the BLE stack that is running on radio coprocessor. The communication is
//! performed through mailbox interface based on shared SRAM area and IPCC peripheral interrupts.
//!
//! This crate defines a public struct, [`RadioCoprocessor`] that owns the IPCC peripheral,
//! implements IPCC IRQ handlers and and implements [`bluetooth_hci::Controller`],
//! which provides access to the full Bluetooth HCI.
//!
//! STM32WB55 BLE stack implements 4.x and 5.x versions of the Bluetooth [specification].
//!
//!
//! # Vendor-Specific Commands
//!
//! STM32WB5x provides several vendor-specific commands that control the behavior of the
//! controller.
//!
//! # Vendor-Specific Events
//!
//! STM32WB5x provides several vendor-specific events that provide data related to the
//! controller. Many of these events are forwarded from the link layer, and these are documented
//! with a reference to the appropriate section of the Bluetooth specification.
//!
//! # Example
//!
//! TODO
//!
//! [specification]: https://www.bluetooth.com/specifications/bluetooth-core-specification

#![no_std]
#![deny(missing_docs)]

#[macro_use]
extern crate bitflags;
#[macro_use]
extern crate bluetooth_hci as hci;
extern crate byteorder;
extern crate embedded_hal as emhal;
#[macro_use(block)]
extern crate nb;

use byteorder::{ByteOrder, LittleEndian};
use core::cmp::min;
use core::convert::TryFrom;
use core::marker::PhantomData;
use hci::host::HciHeader;
use hci::Controller;

mod cb;
mod command;
pub mod event;
mod opcode;

pub use command::gap;
pub use command::gatt;
pub use command::hal;
pub use command::l2cap;

pub use hci::host::{AdvertisingFilterPolicy, AdvertisingType, OwnAddressType};

use stm32wb_hal::tl_mbox;
use stm32wb_hal::ipcc;
use stm32wb_hal::tl_mbox::shci::ShciBleInitCmdParam;

/// Handle for interfacing with the STM32WB5x radio coprocessor.
pub struct RadioCoprocessor {
    mbox: tl_mbox::TlMbox,
    ipcc: ipcc::Ipcc,
    config: ShciBleInitCmdParam,
}

impl RadioCoprocessor {
    /// Creates a new RadioCoprocessor instance to send commands to and receive events from.
    pub fn new(mbox: tl_mbox::TlMbox, ipcc: ipcc::Ipcc, config: ShciBleInitCmdParam) -> RadioCoprocessor {
        RadioCoprocessor {
            mbox,
            ipcc,
            config
        }
    }

    fn write_command(&mut self, opcode: opcode::Opcode, params: &[u8]) -> nb::Result<(), ()> {
        const HEADER_LEN: usize = 4;
        let mut header = [0; HEADER_LEN];
        hci::host::uart::CommandHeader::new(opcode, params.len()).copy_into_slice(&mut header);

        self.write(&header, &params)
    }

    /// Call this function from `IPCC_C1_RX_IT` interrupt context.
    pub fn handle_ipcc_rx(&mut self) {
        self.mbox.interrupt_ipcc_rx_handler(&mut self.ipcc);
    }

    /// Call this function from `IPCC_C1_TX_IT` interrupt context.
    pub fn handle_ipcc_tx(&mut self) {
        self.mbox.interrupt_ipcc_tx_handler(&mut self.ipcc);
    }

    /// Call this function out of interrupt context, for example in `main()` loop.
    pub fn process_event(&mut self) {
        while let Some(evt) = self.mbox.dequeue_event() {
            let event = evt.evt();
            if event.kind() == 18 {
                tl_mbox::shci::shci_ble_init(&mut self.ipcc, self.config);
            }
        }
    }
}

impl hci::Controller for RadioCoprocessor {
    type Error = ();
    type Header = bluetooth_hci::host::uart::CommandHeader;
    type Vendor = Stm32Wb5xTypes;

    fn write(&mut self, header: &[u8], payload: &[u8]) -> nb::Result<(), Self::Error> {
        todo!()
    }

    fn read_into(&mut self, buffer: &mut [u8]) -> nb::Result<(), Self::Error> {
        todo!()
    }

    fn peek(&mut self, n: usize) -> nb::Result<u8, Self::Error> {
        todo!()
    }
}

/// Specify vendor-specific extensions for the BlueNRG.
pub struct Stm32Wb5xTypes;
impl hci::Vendor for Stm32Wb5xTypes {
    type Status = event::Status;
    type Event = event::Stm32Wb5xEvent;
}

/// Master trait that encompasses all commands, and communicates over UART.
pub trait UartController<E>:
    crate::gap::Commands<Error = E>
    + crate::gatt::Commands<Error = E>
    + crate::hal::Commands<Error = E>
    + crate::l2cap::Commands<Error = E>
    + bluetooth_hci::host::uart::Hci<E, crate::event::Stm32Wb5xEvent, crate::event::Stm32Wb5xError>
{
}

impl<T, E> UartController<E> for T where
    T: crate::gap::Commands<Error = E>
        + crate::gatt::Commands<Error = E>
        + crate::hal::Commands<Error = E>
        + crate::l2cap::Commands<Error = E>
        + bluetooth_hci::host::uart::Hci<E, crate::event::Stm32Wb5xEvent, crate::event::Stm32Wb5xError>
{
}

/// Vendor-specific interpretation of the local version information from the controller.
#[derive(Clone)]
pub struct Version {
    /// Major version of the controller firmware
    pub major: u8,

    /// Minor version of the controller firmware
    pub minor: u8,

    /// Patch version of the controller firmware
    pub patch: u8,
}

/// Extension trait to convert [`hci::event::command::LocalVersionInfo`] into the BLE stack-specific
/// [`Version`] struct.
pub trait LocalVersionInfoExt {
    /// Converts LocalVersionInfo as returned by the controller into a BLE stack-specific [`Version`]
    /// struct.
    fn wireless_fw_version(&self) -> Version;
}

impl<VS> LocalVersionInfoExt for hci::event::command::LocalVersionInfo<VS> {
    fn wireless_fw_version(&self) -> Version {
        // TODO
        Version {
            major: (self.hci_revision & 0xFF) as u8,
            minor: ((self.lmp_subversion >> 4) & 0xF) as u8,
            patch: (self.lmp_subversion & 0xF) as u8,
        }
    }
}

