#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_i2c_reg.h"



.bss


.data


.text

.global entry


entry:
WAIT 65000
WAIT 65000
WAIT 65000
I2C_WR 0xF4,0x2E,7,0,0
WAIT 65000
WAIT 65000
WAIT 65000
I2C_RD 0xF6,7,0,0
WAIT 65000
WAIT 65000
WAIT 65000
jump entry
