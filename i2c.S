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
.global boucle

entry:
MOVE r1,resu
jump boucle


boucle:
WAIT 65000
I2C_WR 0x77,0xEE,7,0,0
I2C_WR 0xF4,0x2E,7,0,0
WAIT 65000
I2C_WR 0x77,0xEE,7,0,0
I2C_WR 0x77,0xF6,7,0,0
WAIT 65000
I2C_WR 0x77,0xEF,7,0,0
I2C_RD 0xF7,8,0,0
WAIT 65000
ST r0,r1,0
JUMP boucle

