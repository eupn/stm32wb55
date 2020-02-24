//! BLE Apple iBeacon example.
#![no_main]
#![no_std]
#![allow(non_snake_case)]

extern crate panic_reset;
extern crate stm32wb_hal as hal;

use core::time::Duration;

use cortex_m_rt::exception;
use heapless::spsc::{MultiCore, Queue};
use nb::block;
use rtfm::app;

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
    event::{
        command::{CommandComplete, ReturnParameters},
        Event,
    },
    host::{
        uart::{Hci as UartHci, Packet},
        AdvertisingFilterPolicy, EncryptionKey, Hci, OwnAddressType,
    },
    BdAddr,
};

use stm32wb55::{
    gap::{
        AdvertisingDataType, AdvertisingType, Commands as GapCommands, DiscoverableParameters, Role,
    },
    gatt::{
        CharacteristicHandle, Commands as GattCommads, ServiceHandle,
        UpdateCharacteristicValueParameters,
    },
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
    RadioCoprocessor,
};

pub type HciCommandsQueue =
    Queue<fn(&mut RadioCoprocessor<'static>, &BleContext), heapless::consts::U32, u8, MultiCore>;

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

// Need to be at least 257 bytes to hold biggest possible HCI BLE event + header
const BLE_DATA_BUF_SIZE: usize = 257 * 2;
const BLE_GAP_DEVICE_NAME_LENGTH: u8 = 7;

#[derive(Debug, Default)]
pub struct BleContext {
    service_handle: Option<ServiceHandle>,
    dev_name_handle: Option<CharacteristicHandle>,
    appearence_handle: Option<CharacteristicHandle>,
}

#[app(device = stm32wb_hal::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        rc: RadioCoprocessor<'static>,
        hci_commands_queue: HciCommandsQueue,
        ble_context: BleContext,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        static mut BLE_DATA_BUF: [u8; BLE_DATA_BUF_SIZE] = [0u8; BLE_DATA_BUF_SIZE];

        let dp = cx.device;
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

        // RTC is required for proper operation of BLE stack
        let _rtc = hal::rtc::Rtc::rtc(dp.RTC, &mut rcc);

        let mut ipcc = dp.IPCC.constrain();
        let mbox = TlMbox::tl_init(&mut rcc, &mut ipcc);

        // Boot CPU2
        hal::pwr::set_cpu2(true);

        let config = ShciBleInitCmdParam {
            p_ble_buffer_address: 0,
            ble_buffer_size: 0,
            num_attr_record: 68,
            num_attr_serv: 8,
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
        let rc = RadioCoprocessor::new(&mut BLE_DATA_BUF[..], mbox, ipcc, config);

        init::LateResources {
            rc,
            hci_commands_queue: HciCommandsQueue::u8(),
            ble_context: BleContext::default(),
        }
    }

    #[idle(resources = [rc, ble_context], spawn = [setup, exec_hci, event])]
    fn idle(mut cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();

            // At this point, an interrupt was received.
            // Radio co-processor talks to the app via IPCC interrupts, so this interrupt
            // may be one of the IPCC interrupts and the app can start processing events from
            // radio co-processor here.
            let evt = cx.resources.rc.lock(|rc| {
                if rc.process_events() {
                    Some(block!(rc.read()))
                } else {
                    None
                }
            });

            if let Some(Ok(Packet::Event(evt))) = evt {
                if let Event::Vendor(stm32wb55::event::Stm32Wb5xEvent::CoprocessorReady(_)) = evt {
                    // Setup BLE service when BLE co-processor is ready
                    cx.spawn.setup().unwrap();
                } else {
                    cx.spawn.event(evt).unwrap();
                    cx.spawn.exec_hci().unwrap();
                }
            }
        }
    }

    /// Sets up Eddystone BLE beacon service.
    #[task(resources = [rc, hci_commands_queue], spawn = [exec_hci])]
    fn setup(mut cx: setup::Context) {
        cx.resources
            .hci_commands_queue
            .enqueue(|rc, _| rc.reset().unwrap())
            .ok();

        init_gap_and_gatt(&mut cx.resources.hci_commands_queue);
        init_ibeacon(&mut cx.resources.hci_commands_queue);

        // Execute first HCI command from the queue
        cx.spawn.exec_hci().unwrap();
    }

    /// Executes HCI command from the queue.
    #[task(resources = [rc, hci_commands_queue, ble_context])]
    fn exec_hci(mut cx: exec_hci::Context) {
        if let Some(cmd) = cx.resources.hci_commands_queue.dequeue() {
            cmd(&mut cx.resources.rc, &cx.resources.ble_context);
        }
    }

    /// Processes BLE events.
    #[task(resources = [ble_context])]
    fn event(mut cx: event::Context, event: Event<stm32wb55::event::Stm32Wb5xEvent>) {
        if let Event::CommandComplete(CommandComplete { return_params, .. }) = event {
            match return_params {
                ReturnParameters::Vendor(stm32wb55::event::command::ReturnParameters::GapInit(
                    stm32wb55::event::command::GapInit {
                        service_handle,
                        dev_name_handle,
                        appearance_handle,
                        ..
                    },
                )) => {
                    cx.resources.ble_context.service_handle = Some(service_handle);
                    cx.resources.ble_context.dev_name_handle = Some(dev_name_handle);
                    cx.resources.ble_context.appearence_handle = Some(appearance_handle);
                }

                _ => (),
            }
        }
    }

    /// Handles IPCC interrupt and notifies `RadioCoprocessor` code about it.
    #[task(binds = IPCC_C1_RX_IT, resources = [rc])]
    fn mbox_rx(cx: mbox_rx::Context) {
        cx.resources.rc.handle_ipcc_rx();
    }

    /// Handles IPCC interrupt and notifies `RadioCoprocessor` code about it.
    #[task(binds = IPCC_C1_TX_IT, resources = [rc])]
    fn mbox_tx(cx: mbox_tx::Context) {
        cx.resources.rc.handle_ipcc_tx();
    }

    // Interrupt handlers used to dispatch software tasks.
    // One per priority.
    extern "C" {
        fn USART1();
    }
};

#[exception]
fn DefaultHandler(irqn: i16) -> ! {
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

fn init_gap_and_gatt(hci_commands_queue: &mut HciCommandsQueue) {
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
                .expect("set public address");
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::random_address(get_random_addr()).build())
                .expect("set random address");
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::identity_root(&get_irk()).build())
                .expect("set IRK address");
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::encryption_root(&get_erk()).build())
                .expect("set ERK address");
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.set_tx_power_level(PowerLevel::ZerodBm)
                .expect("set TX power level")
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| rc.init_gatt().expect("GATT init"))
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME_LENGTH)
                .expect("GAP init")
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, cx| {
            rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
                service_handle: cx.service_handle.expect("service handle to be set"),
                characteristic_handle: cx.dev_name_handle.expect("dev name handle to be set"),
                offset: 0,
                value: b"BEACON",
            })
            .unwrap()
        })
        .ok();
}

fn init_ibeacon(hci_commands_queue: &mut HciCommandsQueue) {
    // Disable scan response
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.le_set_scan_response_data(&[])
                .expect("set scan response data")
        })
        .ok();

    // Put the device in a non-connectable mode
    hci_commands_queue
        .enqueue(|rc, _| {
            let params = DiscoverableParameters {
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

            rc.set_discoverable(&params)
                .expect("set discoverable params")
        })
        .ok();

    // Remove some advertisements (this is done to decrease the packet size)
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.delete_ad_type(AdvertisingDataType::TxPowerLevel)
                .expect("delete tx power ad type")
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.delete_ad_type(AdvertisingDataType::PeripheralConnectionInterval)
                .expect("delete conn interval ad type")
        })
        .ok();

    hci_commands_queue
        .enqueue(|rc, _| {
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
                .expect("update service data")
        })
        .ok();
    hci_commands_queue
        .enqueue(|rc, _| {
            let mut service_uuid_list = [0u8; 18];
            service_uuid_list[0] = 17;
            service_uuid_list[1] = AdvertisingDataType::UuidCompleteList16 as u8;
            service_uuid_list[2..(2 + IBEACON_UUID.len())].copy_from_slice(&IBEACON_UUID[..]);

            rc.update_advertising_data(&service_uuid_list[..])
                .expect("update service uuid list data")
        })
        .ok();

    hci_commands_queue
        .enqueue(|rc, _| {
            let flags = [
                2,
                AdvertisingDataType::Flags as u8,
                (0x02 | 0x04) as u8, // BLE general discoverable, without BR/EDR support.
            ];

            rc.update_advertising_data(&flags[..])
                .expect("update flags data")
        })
        .ok();
}
