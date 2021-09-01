//! BLE Apple iBeacon example.
#![no_main]
#![no_std]
#![allow(non_snake_case)]

use panic_rtt_target as _;
// use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

extern crate stm32wb_hal as hal;

use core::{convert::TryFrom, future::Pending, time::Duration};

use cortex_m_rt::{entry, exception};
use nb::block;

use hal::{
    flash::FlashExt,
    prelude::*,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, RfWakeupClock, RtcClkSrc,
        StopWakeupClock, SysClkSrc,
    },
    tl_mbox::{lhci::LhciC1DeviceInformationCcrp, shci::ShciBleInitCmdParam, TlMbox},
};

use bluetooth_hci::{
    event::{command::ReturnParameters, Event},
    host::{uart::Packet, AdvertisingFilterPolicy, EncryptionKey, Hci, OwnAddressType},
    BdAddr,
};

use ble::{perform_command, receive_event, setup_coprocessor, Characteristic, RadioCopro};
use stm32wb55::{
    event::{AttReadPermitRequest, AttributeHandle, GattAttributeModified, Stm32Wb5xEvent},
    gap::{
        AdvertisingDataType, AdvertisingType, AuthenticationRequirements, Commands as GapCommands,
        DiscoverableParameters, LocalName, OutOfBandAuthentication, Pin, Role,
    },
    gatt::{CharacteristicProperty, Commands as GattCommads, UpdateCharacteristicValueParameters},
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
};

mod ble;

// Apple iBeacon UUID specific for your application.
// You can use https://yupana-engineering.com/online-uuid-to-c-array-converter to convert
// UUID string into byte array
const IBEACON_UUID: [u8; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
const IBEACON_MAJOR: [u8; 2] = [0, 1]; // Two-byte iBeacon major
const IBEACON_MINOR: [u8; 2] = [0, 1]; // Two-byte iBeacon minor

/// Advertisement interval in milliseconds.
const ADV_INTERVAL_MS: u64 = 250;

/// TX power at 0 m range. Used for range approximation.
const CALIBRATED_TX_POWER_AT_0_M: u8 = 193;

// Required to be recognised as iBeacon
const BLE_BEACON_NAME: &[u8] = b"BEACON";


#[entry]
fn entry() -> ! {
    //rtt_init_print!(BlockIfFull, 4096);
    rtt_init_print!(NoBlockSkip, 4096);
    run();

    loop {
        continue;
    }
}

fn run() {
    let dp = hal::device::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    rcc.set_stop_wakeup_clock(StopWakeupClock::HSI16);

    // Fastest clock configuration.
    // * External low-speed crystal is used (LSE)
    // * 32 MHz HSE with PLL
    // * 64 MHz CPU1, 32 MHz CPU2
    // * 64 MHz for APB1, APB2
    // * HSI as a clock source after wake-up from low-power mode
    let clock_config = Config::new(SysClkSrc::Pll(PllSrc::Hse(HseDivider::NotDivided)))
        .with_lse()
        .cpu1_hdiv(HDivider::NotDivided)
        .cpu2_hdiv(HDivider::Div2)
        .apb1_div(ApbDivider::NotDivided)
        .apb2_div(ApbDivider::NotDivided)
        .pll_cfg(PllConfig {
            m: 2,
            n: 12,
            r: 3,
            q: Some(4),
            p: Some(3),
        })
        .rtc_src(RtcClkSrc::Lse)
        .rf_wkp_sel(RfWakeupClock::Lse);

    let mut rcc = rcc.apply_clock_config(clock_config, &mut dp.FLASH.constrain().acr);

    rprintln!("Boot");

    // RTC is required for proper operation of BLE stack
    let _rtc = hal::rtc::Rtc::rtc(dp.RTC, &mut rcc);

    let mut ipcc = dp.IPCC.constrain();
    let mbox = TlMbox::tl_init(&mut rcc, &mut ipcc);

    let config = ShciBleInitCmdParam {
        p_ble_buffer_address: 0,
        ble_buffer_size: 0,
        num_attr_record: 100,
        num_attr_serv: 10,
        attr_value_arr_size: 1344,
        num_of_links: 8,
        extended_packet_length_enable: 1,
        pr_write_list_size: 0x3A,
        mb_lock_count: 0x79,
        att_mtu: 156,
        slave_sca: 500,
        master_sca: 0,
        ls_source: 1,
        max_conn_event_length: 0xFFFFFFFF,
        hs_startup_time: 0x148,
        viterbi_enable: 1,
        ll_only: 0,
        hw_version: 0,
    };

    setup_coprocessor(config, ipcc, mbox);

    // enable interrupts -> interrupts are enabled in Ipcc::init(), which is called TlMbox::tl_init

    // Boot CPU2
    hal::pwr::set_cpu2(true);

    let ready_event = block!(receive_event());

    rprintln!("Received packet: {:?}", ready_event);

    rprintln!("Resetting processor...");

    let reset_response = perform_command(|rc| rc.reset()).expect("Failed to reset processor");

    rprintln!("Received packet: {:?}", reset_response);

    init_gap_and_gatt().expect("Failed to initialize GAP and GATT");

    rprintln!("Succesfully initialized GAP and GATT");

    init_ibeacon().expect("Failed to initialize iBeacon");

    rprintln!("Succesfully initialized iBeacon");

    loop {
        let response = block!(receive_event());

        rprintln!("Received event: {:x?}", response);

        if let Ok(Packet::Event(event)) = response {
            match event {
                other => rprintln!("ignoring event {:?}", other),
            }
        }
    }
}

#[exception]
unsafe fn DefaultHandler(irqn: i16) -> ! {
    panic!("Unhandled IRQ: {}", irqn);
}

fn get_bd_addr() -> BdAddr {
    let mut bytes = [0u8; 6];

    let lhci_info = LhciC1DeviceInformationCcrp::new();
    bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    bytes[3] = lhci_info.device_type_id;
    bytes[4] = (lhci_info.st_company_id & 0xff) as u8;
    bytes[5] = (lhci_info.st_company_id >> 8 & 0xff) as u8;

    BdAddr(bytes)
}

fn init_gap_and_gatt() -> Result<(), ()> {
    let response = perform_command(|rc: &mut RadioCopro| {
        rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
    })?;

    rprintln!("Response to write_config_data: {:?}", response);

    perform_command(|rc| {
        rc.write_config_data(&ConfigData::random_address(get_random_addr()).build())
    })?;

    perform_command(|rc| rc.write_config_data(&ConfigData::identity_root(&get_irk()).build()))?;

    perform_command(|rc| rc.write_config_data(&ConfigData::encryption_root(&get_erk()).build()))?;

    perform_command(|rc| rc.set_tx_power_level(PowerLevel::ZerodBm))?;

    perform_command(|rc| rc.init_gatt())?;

    let return_params =
        perform_command(|rc| rc.init_gap(Role::PERIPHERAL, false,
                                         BLE_BEACON_NAME.len() as u8))?;

    let (service_handle, dev_name_handle, appearence_handle) = if let ReturnParameters::Vendor(
        stm32wb55::event::command::ReturnParameters::GapInit(stm32wb55::event::command::GapInit {
            service_handle,
            dev_name_handle,
            appearance_handle,
            ..
        }),
    ) = return_params
    {
        (service_handle, dev_name_handle, appearance_handle)
    } else {
        rprintln!("Unexpected response to init_gap command");
        return Err(());
    };

    perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: dev_name_handle,
            offset: 0,
            value: BLE_BEACON_NAME,
        })
        .map_err(|_| nb::Error::Other(()))
    })?;

    let appearance_characteristic = Characteristic {
        service: service_handle,
        characteristic: appearence_handle,
        max_len: 4,
    };

    appearance_characteristic.set_value(&[0x80, 0x00])?;
    return Ok(());
}

fn init_ibeacon() -> Result<(), ()> {
    // disable scan response
    perform_command(|rc: &mut RadioCopro| {
        rc.le_set_scan_response_data(&[])
            .map_err(|_| nb::Error::Other(()))
    })?;

    // non-connectable mode...
    perform_command(|rc| {
        rc.set_discoverable(&DISCOVERY_PARAMS)
            .map_err(|_| nb::Error::Other(()))
    })?;

    // remove some advertisements to decrease packet size
    perform_command(|rc| rc.delete_ad_type(AdvertisingDataType::TxPowerLevel))?;
    perform_command(|rc| rc.delete_ad_type(AdvertisingDataType::PeripheralConnectionInterval))?;

    perform_command(|rc| {
        let mut service_data = [0u8; 27];
        service_data[0] = 26;
        service_data[1] = AdvertisingDataType::ManufacturerSpecificData as u8;

        // iBeacon specific 32-bit data blob
        service_data[2] = 0x4C;
        service_data[3] = 0x00;
        service_data[4] = 0x02;
        service_data[5] = 0x15;

        // Uuid bytes
        let mut index = 6;
        service_data[index..(index + IBEACON_UUID.len())].copy_from_slice(&IBEACON_UUID[..]);
        index += IBEACON_UUID.len();

        // Major bytes
        service_data[index..(index + IBEACON_MAJOR.len())].copy_from_slice(&IBEACON_MAJOR[..]);
        index += IBEACON_MAJOR.len();

        // Minor bytes
        service_data[index..(index + IBEACON_MINOR.len())].copy_from_slice(&IBEACON_MAJOR[..]);
        index += IBEACON_MINOR.len();

        // TX power at 0 meters for ranging
        service_data[index] = CALIBRATED_TX_POWER_AT_0_M;

        rc.update_advertising_data(&service_data[..])
            .map_err(|_| nb::Error::Other(()))
    })?;

    perform_command(|rc| {
        let mut service_uuid_list = [0u8; 18];
        service_uuid_list[0] = 17;
        service_uuid_list[1] = AdvertisingDataType::UuidCompleteList16 as u8;
        service_uuid_list[2..(2 + IBEACON_UUID.len())].copy_from_slice(&IBEACON_UUID[..]);

        rc.update_advertising_data(&service_uuid_list[..])
            .map_err(|_| nb::Error::Other(()))
    })?;

    perform_command(|rc| {
        let flags = [
            2,
            AdvertisingDataType::Flags as u8,
            (0x02 | 0x04) as u8, // BLE general discoverable, without BR/EDR support.
        ];

        rc.update_advertising_data(&flags[..])
            .map_err(|_| nb::Error::Other(()))
    })?;

    return Ok(());
}

fn get_random_addr() -> BdAddr {
    let mut bytes = [0u8; 6];

    let lhci_info = LhciC1DeviceInformationCcrp::new();
    bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    bytes[3] = 0;
    bytes[4] = 0x6E;
    bytes[5] = 0xED;

    BdAddr(bytes)
}

const BLE_CFG_IRK: [u8; 16] = [
    0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
];
const BLE_CFG_ERK: [u8; 16] = [
    0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21, 0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21,
];

fn get_irk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_IRK)
}

fn get_erk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_ERK)
}

const DISCOVERY_PARAMS: DiscoverableParameters = DiscoverableParameters {
    advertising_type: AdvertisingType::NonConnectableUndirected,
    advertising_interval: Some((
        Duration::from_millis(ADV_INTERVAL_MS),
        Duration::from_millis(ADV_INTERVAL_MS),
    )),
    address_type: OwnAddressType::Public,
    filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
    // Local name should be empty for the device to be recognized as an Eddystone beacon
    local_name: None,
    advertising_data: &[],
    conn_interval: (None, None),
};
