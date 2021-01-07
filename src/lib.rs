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

use core::convert::TryFrom;
use hci::host::HciHeader;
use hci::Controller;

mod command;
pub mod event;
mod opcode;

pub use command::gap;
pub use command::gatt;
pub use command::hal;
pub use command::l2cap;

pub use hci::host::{AdvertisingFilterPolicy, AdvertisingType, OwnAddressType};

use stm32wb_hal::ipcc;
use stm32wb_hal::tl_mbox;
use stm32wb_hal::tl_mbox::cmd::CmdSerial;
use stm32wb_hal::tl_mbox::{consts::TlPacketType, shci::ShciBleInitCmdParam};

use bbqueue::{ArrayLength, Consumer, Producer};

const TX_BUF_SIZE: usize = core::mem::size_of::<CmdSerial>();

/// Handle for interfacing with the STM32WB5x radio coprocessor.
pub struct RadioCoprocessor<'buf, N: ArrayLength<u8>> {
    mbox: tl_mbox::TlMbox,
    ipcc: ipcc::Ipcc,
    config: ShciBleInitCmdParam,
    buff_producer: Producer<'buf, N>,
    buff_consumer: Consumer<'buf, N>,
    tx_buf: [u8; TX_BUF_SIZE],
    is_ble_ready: bool,
}

impl<'buf, N: ArrayLength<u8>> RadioCoprocessor<'buf, N> {
    /// Creates a new RadioCoprocessor instance to send commands to and receive events from.
    pub fn new(
        producer: Producer<'buf, N>,
        consumer: Consumer<'buf, N>,
        mbox: tl_mbox::TlMbox,
        ipcc: ipcc::Ipcc,
        config: ShciBleInitCmdParam,
    ) -> RadioCoprocessor<'buf, N> {
        RadioCoprocessor {
            mbox,
            ipcc,
            config,
            buff_consumer: consumer,
            buff_producer: producer,
            tx_buf: [0u8; TX_BUF_SIZE],
            is_ble_ready: false,
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

    /// Call this function outside of interrupt context, for example in `main()` loop.
    /// Returns `true` if events was written and can be read with HCI `read()` function.
    /// Returns `false` if no HCI events was written.
    pub fn process_events(&mut self) -> bool {
        while let Some(evt) = self.mbox.dequeue_event() {
            let event = evt.evt();

            let mut buf = self
                .buff_producer
                .grant_exact(evt.size().expect("Known packet kind"))
                .expect("No space in buffer");

            evt.write(buf.buf()).expect("EVT_BUF_SIZE is too small");

            if event.kind() == 18 {
                tl_mbox::shci::shci_ble_init(&mut self.ipcc, self.config);
                self.is_ble_ready = true;
                buf.buf()[0] = 0x04; // Replace event code with one that is supported by HCI
            }

            buf.commit(evt.size().unwrap());
        }

        // Ignore SYS-channel "command complete" events
        if self.mbox.pop_last_cc_evt().is_some() {
            return false;
        }

        return true;
    }
}

impl<'buf, N: ArrayLength<u8>> hci::Controller for RadioCoprocessor<'buf, N> {
    type Error = ();
    type Header = bluetooth_hci::host::uart::CommandHeader;
    type Vendor = Stm32Wb5xTypes;

    fn write(&mut self, header: &[u8], payload: &[u8]) -> nb::Result<(), Self::Error> {
        let cmd_code = header[0];
        let cmd = TlPacketType::try_from(cmd_code).map_err(|_| ())?;

        self.tx_buf = [0; TX_BUF_SIZE];
        self.tx_buf[..header.len()].copy_from_slice(header);
        self.tx_buf[header.len()..(header.len() + payload.len())].copy_from_slice(payload);

        match &cmd {
            TlPacketType::AclData => {
                // Destination buffer: ble table, phci_acl_data_buffer, acldataserial field
                todo!()
            }

            TlPacketType::SysCmd => {
                // Destination buffer: SYS table, pcmdbuffer, cmdserial field
                todo!()
            }

            _ => {
                tl_mbox::ble::ble_send_cmd(&mut self.ipcc, &self.tx_buf[..]);
            }
        }

        Ok(())
    }

    fn read_into(&mut self, buffer: &mut [u8]) -> nb::Result<(), Self::Error> {
        match self.buff_consumer.read() {
            Ok(grant) => {
                if buffer.len() <= grant.buf().len() {
                    buffer.copy_from_slice(&grant.buf()[..buffer.len()]);

                    grant.release(buffer.len());

                    Ok(())
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }
            Err(bbqueue::Error::InsufficientSize) => Err(nb::Error::WouldBlock),
            Err(_other) => Err(nb::Error::Other(())),
        }
    }

    fn peek(&mut self, n: usize) -> nb::Result<u8, Self::Error> {
        match self.buff_consumer.read() {
            Ok(grant) => {
                if n >= grant.buf().len() {
                    return Err(nb::Error::WouldBlock);
                } else {
                    Ok(grant.buf()[n])
                }
            }
            Err(bbqueue::Error::InsufficientSize) => Err(nb::Error::WouldBlock),
            Err(_other) => Err(nb::Error::Other(())),
        }
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
        + bluetooth_hci::host::uart::Hci<
            E,
            crate::event::Stm32Wb5xEvent,
            crate::event::Stm32Wb5xError,
        >
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
