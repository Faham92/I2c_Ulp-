#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_i2c_reg.h"



.bss

.global resu 

resu: 
    .long 0

.data


.text

.global entry


entry:
//WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 12, 1, 1)
WAIT 65000
WAIT 65000
WAIT 65000
I2C_WR 0x77,0xEE,7,0,0
I2C_WR 0xF4,0x2E,7,0,0
WAIT 65000
WAIT 65000
WAIT 65000
I2C_RD 0xF7,7,0,0
//ST r0,r1,0
//MOVE r1,resu
//I2C_RD 0xF4,7,0,0
//ST r0,r2,0
//MOVE r2,resu
WAIT 65000
WAIT 65000
WAIT 65000
JUMP entry
