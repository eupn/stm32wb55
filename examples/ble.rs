use bbqueue::{consts::U514, BBBuffer, ConstBBBuffer};
use bluetooth_hci::{
    event::command::{CommandComplete, ReturnParameters},
    host::uart::{Hci as UartHci, Packet},
    Event, Status,
};
use core::fmt::Debug;
use nb::block;
use rtt_target::rprintln;
use stm32wb55::{
    event::{command::GattCharacteristicDescriptor, AttributeHandle, Stm32Wb5xEvent},
    gatt::{
        AccessPermission, AddCharacteristicParameters, AddDescriptorParameters,
        AddServiceParameters, CharacteristicEvent, CharacteristicHandle, CharacteristicPermission,
        CharacteristicProperty, Commands as GattCommands, DescriptorHandle, DescriptorPermission,
        EncryptionKeySize, ServiceHandle, ServiceType, UpdateCharacteristicValueParameters, Uuid,
    },
    RadioCoprocessor,
};
use stm32wb_hal::{
    interrupt,
    ipcc::Ipcc,
    tl_mbox::{shci::ShciBleInitCmdParam, TlMbox},
};

pub type RadioCopro = RadioCoprocessor<'static, U514>;

static BB: BBBuffer<U514> = BBBuffer(ConstBBBuffer::new());

static mut RADIO_COPROCESSOR: Option<RadioCopro> = None;

pub fn setup_coprocessor(config: ShciBleInitCmdParam, ipcc: Ipcc, mbox: TlMbox) {
    let (producer, consumer) = BB.try_split().unwrap();
    let rc = RadioCoprocessor::new(producer, consumer, mbox, ipcc, config);

    unsafe {
        RADIO_COPROCESSOR = Some(rc);
    }
}

#[derive(Debug)]
pub struct Service {
    handle: ServiceHandle,

    max_num_attributes: u8,
}

impl Service {
    pub fn new(
        service_type: ServiceType,
        uuid: Uuid,
        max_attribute_records: u8,
    ) -> Result<Self, ()> {
        rprintln!("Adding service {:x?}", uuid);

        let protocol_handle = perform_command(|rc: &mut RadioCopro| {
            let service = AddServiceParameters {
                service_type,
                uuid,
                max_attribute_records,
            };
            rc.add_service(&service)
        })?;

        if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddService(
                stm32wb55::event::command::GattService {
                    service_handle,
                    status,
                },
            ),
        ) = protocol_handle
        {
            rprintln!("Handle {:?}, status {:?}", service_handle, status);
            check_status(&status).expect("Failed to add service");
            Ok(Service {
                handle: service_handle,
                max_num_attributes: max_attribute_records,
            })
        } else {
            //writeln!(serial, "Unexpected response to init_gap command");
            Err(())
        }
    }

    pub fn add_characteristic(
        &self,
        uuid: &Uuid,
        properties: CharacteristicProperty,
        event_mask: CharacteristicEvent,
        value_len: usize,
        is_variable: bool,
    ) -> Result<Characteristic, ()> {
        rprintln!("Adding characteristic {:x?}", uuid);
        rprintln!(" Properties: {:?}", properties);

        let response = perform_command(|rc: &mut RadioCopro| {
            rc.add_characteristic(&AddCharacteristicParameters {
                service_handle: self.handle,
                characteristic_uuid: *uuid,
                characteristic_properties: properties,
                characteristic_value_len: value_len,

                is_variable,

                // Initially hardcoded
                gatt_event_mask: event_mask,
                encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
                fw_version_before_v72: false,
                security_permissions: CharacteristicPermission::empty(),
            })
        })?;

        if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattAddCharacteristic(
                stm32wb55::event::command::GattCharacteristic {
                    characteristic_handle,
                    status,
                },
            ),
        ) = response
        {
            check_status(&status).expect("Failed to add characteristic");
            rprintln!("Handle (declaration): {:?}", characteristic_handle);
            rprintln!("Handle (value): {:?}", characteristic_handle.0 + 1);

            // If the notify or indicate properties are set,
            // a CCCD (Client characteristic configuration descriptor) is allocated as well.
            if properties
                .intersects(CharacteristicProperty::NOTIFY | CharacteristicProperty::INDICATE)
            {
                rprintln!(
                    "Client characteristic configuration: Handle={}",
                    characteristic_handle.0 + 1,
                );
            }

            Ok(Characteristic {
                service: self.handle,
                characteristic: characteristic_handle,
                max_len: value_len,
            })
        } else {
            Err(())
        }
    }

    /// Check if this service contains the given handle.
    ///
    /// The check is done based on the maximum number of handles
    /// reserved for this service, as given in the `Service::new`
    /// function. A value of `true` does not guarantee that
    /// the given handle has actually been created, however the
    /// given handle cannot exist in any other service.
    pub fn contains_handle(&self, handle: AttributeHandle) -> bool {
        let value = handle.0;

        let service_handle = self.handle.0;

        service_handle <= value && value < (service_handle + self.max_num_attributes as u16)
    }
}

#[derive(Debug)]
pub struct Characteristic {
    pub service: ServiceHandle,
    pub characteristic: CharacteristicHandle,

    pub max_len: usize,
}

impl Characteristic {
    pub fn set_value(&self, value: &[u8]) -> Result<(), ()> {
        if value.len() > self.max_len {
            return Err(());
        }

        let response = perform_command(|rc: &mut RadioCopro| {
            rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
                service_handle: self.service,
                characteristic_handle: self.characteristic,
                offset: 0,
                value,
            })
            .map_err(|_| nb::Error::Other(()))
        })?;

        if let ReturnParameters::Vendor(
            stm32wb55::event::command::ReturnParameters::GattUpdateCharacteristicValue(status),
        ) = response
        {
            rprintln!("Update value: {:?}", status);
            check_status(&status)?;
        } else {
            // Unexpected response;
            rprintln!("Unexpected reponse to UpdateValue: {:?}", response);
        };

        Ok(())
    }

    pub fn add_descriptor(&self, uuid: Uuid, length: usize) -> Result<DescriptorHandle, ()> {
        let dummy_slice = [0u8; 10];

        assert!(length <= 10, "Hack: Not implemented for length > 10");

        let descriptor = perform_command(|rc: &mut RadioCopro| {
            rc.add_characteristic_descriptor(&mut AddDescriptorParameters {
                service_handle: self.service,
                characteristic_handle: self.characteristic,
                descriptor_uuid: uuid,
                descriptor_value_max_len: length,
                descriptor_value: &dummy_slice[..length],
                security_permissions: DescriptorPermission::empty(),
                access_permissions: AccessPermission::READ,
                gatt_event_mask: CharacteristicEvent::ATTRIBUTE_WRITE
                    | CharacteristicEvent::CONFIRM_READ,
                encryption_key_size: EncryptionKeySize::with_value(16).unwrap(),
                is_variable: false,
            })
            .map_err(|_| nb::Error::Other(()))
        })?;

        let descriptor_handle = match descriptor {
            ReturnParameters::Vendor(
                stm32wb55::event::command::ReturnParameters::GattAddCharacteristicDescriptor(
                    GattCharacteristicDescriptor {
                        status,
                        descriptor_handle,
                    },
                ),
            ) => {
                check_status(&status)?;
                descriptor_handle
            }
            _ => {
                rprintln!("Unexpected response to init_gap command");
                return Err(());
            }
        };

        rprintln!("Descriptor {:?} - {:?}", uuid, descriptor_handle);

        Ok(descriptor_handle)
    }
}

fn check_status<S: Debug>(status: &Status<S>) -> Result<(), ()> {
    if let Status::Success = status {
        Ok(())
    } else {
        rprintln!("Status not succesfull: {:?}", status);
        Err(())
    }
}

pub fn perform_command(
    command: impl Fn(&mut RadioCopro) -> nb::Result<(), ()>,
) -> Result<ReturnParameters<Stm32Wb5xEvent>, ()> {
    // Send command (blocking)
    block!(cortex_m::interrupt::free(|_| {
        let rc = unsafe { RADIO_COPROCESSOR.as_mut().unwrap() };
        command(rc)
    }))?;

    let response = block!(receive_event()).unwrap(); // .map_err(|_| Err(()))?;

    if let Packet::Event(Event::CommandComplete(CommandComplete {
        return_params,
        num_hci_command_packets: _,
    })) = response
    {
        Ok(return_params)
    } else {
        Err(())
    }
}

pub fn receive_event() -> nb::Result<
    Packet<Stm32Wb5xEvent>,
    bluetooth_hci::host::uart::Error<(), stm32wb55::event::Stm32Wb5xError>,
> {
    cortex_m::interrupt::free(|_| {
        let rc = unsafe { RADIO_COPROCESSOR.as_mut().unwrap() };
        if rc.process_events() {
            rc.read()
        } else {
            Err(nb::Error::WouldBlock)
        }
    })
}

// Handle IPCC_C1_RX_IT interrupt
#[interrupt]
fn IPCC_C1_RX_IT() {
    unsafe {
        RADIO_COPROCESSOR.as_mut().unwrap().handle_ipcc_rx();
    }
}

// Handle IPCC_C1_TX_IT interrupt
#[interrupt]
fn IPCC_C1_TX_IT() {
    // TODO: Critical section?
    unsafe {
        RADIO_COPROCESSOR.as_mut().unwrap().handle_ipcc_tx();
    }
}
