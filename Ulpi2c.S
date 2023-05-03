#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_i2c_reg.h"


/*
.bss

/.global gas_adc

gas_adc: 
    .long 0

.global gas_range

gas_range:
    .long 0

.global flag

flag:
    .long 0

 
.text

.global entry
.global boucle

.macro I2C_Read addr
WAIT 65000
WAIT 65000
I2C_RD \addr,7,0,0
WAIT 65000
WAIT 65000
.endm

.macro calc_gas_adc
I2C_Read 0x2C //fetch value in register gas_adc
ST r0,r1,0    
I2C_Read 0x2D //fetch value in register gas_range
ST r0,r2,0 
LSH r1,r1,2
RSH r2,r2,6
OR r1,r1,r2
.endm


//entry:
MOVE r1,gas_adc
MOVE r2,gas_range
MOVE r3,flag
I2C_Read 0x2C
//RSH r0,r0,17
ST r0,r2,0 


//WAKE
jump boucle
HALT
//JUMP calc_gas

boucle:
NOP
jump boucle



calc_gas:
//calc_gas_adc 
I2C_Read 0x2D
//AND r0,r0,15
ST r0,r2,0 
WAIT 65000
I2C_WR 0x74,0x01,7,0,0
//JUMP calc_gas
WAKE
HALT


*/