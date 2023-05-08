#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"
#include "soc/rtc_i2c_reg.h"


#include "stack.S"


// unluckily bitbanging I2C trashes all registers except r3 which must contain the stack.
.macro i2crd regnum addr
    push  r2
    move r1,\addr
    push r1
    move r1,\regnum
    push r1
    psr
    jump read8
    add r3,r3,2 // remove 2 arguments from stack
    pop r2
.endm

.macro i2cwr regnum,val,addr
    push r2
    move r1,\addr
    push r1
    move r1,\regnum
    push r1
    move r1,\val
    push r1
    psr
    jump write8
    add r3,r3,3 // remove 3 arguments from stack
    pop r2
.endm

.macro saut reg 
    jump \reg
.endm
.macro send_flag val
    move r2,flag
    move r0,\val
    st r0,r2,0
.endm
/* Define variables, which go into .bss section (zero-initialized data) */
    .bss

   .global flag
flag:
   .long 0
// stack
  .global stack
stack:
  .fill 100
  .global stackEnd
stackEnd:
  .long 0


/* Code goes into .text section */
    .text

    .global entry
    .global entry2

entry:   
    // Main code  
    move r3,stackEnd

calc_gas:
    delai
    i2crd 0x2C 0x77//fetch value in register gas_adc
    lsh r0,r0,2
    move r2,r0
    i2crd 0x2D 0x77//fetch value in register gas_range
    rsh r0,r0,6
    or r0,r2,r0  //r0 contains results of gas_adc
    move r2,800
    sub r0,r2,r0   // if gas_adc > 800 then go check gas_range
    jump range,ov
    i2cwr 0x74 0x01 0x77
    delai
    delai
    saut calc_gas


range:
    i2crd 0x2D 0x77
    and r0,r0,0x0F   //r0 contains results of gas_range
    jumpr wakeup,10,ge // if gas_range >= 10 wake the esp32 
    i2cwr 0x74 0x01 0x77
    delai
    saut calc_gas

 wakeup:
   send_flag 0
   WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 9, 1, 0)
   delai
   WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 9, 1, 1)
   halt







// Entry2 to wait that GSR goes above the threshold 
              
 
entry2:
    delai
    i2crd 0x2D 0x77
    and r0,r0,0x0F   //r0 contains results of gas_range
    jumpr wakeup2,9,le// if gas_range=9 wake the esp32 
    i2cwr 0x74 0x01 0x77
    delai
    saut entry2

wakeup2:
   send_flag 1
   WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 9, 1, 0)
   delai
   WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 9, 1, 1)
   halt
    