target extended-remote :3333
break HardFault
#break IPCC_C1_RX_IT
#break IPCC_C1_TX_IT
monitor arm semihosting enable
monitor reset
monitor reset halt
load
si
