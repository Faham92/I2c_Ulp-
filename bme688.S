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

/* Define variables, which go into .bss section (zero-initialized data) */
    .bss

   .global gas_adc
gas_adc:
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

    .global calc_gas_adc
    .global entry
entry:   
    // Main code
    
    move r3,stackEnd
    jump calc_gas_adc

calc_gas_adc:
    i2crd 0x2C 0x77//fetch value in register gas_adc
    st r0,r2,00
    ld r2,r2,0    
    i2crd 0x2D 0x77//fetch value in register gas_range
    //st r0,r2,0 
    rsh r0,r0,6
    lsh r2,r2,2
    or r0,r2,r0
    move r2, gas_adc
    st r0,r2,0
    wait 30000
    i2cwr 0x74 0x01 0x77
    wait 30000
    jump calc_gas_adc


    
    // Wake up ESP32
is_rdy_for_wakeup:
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP) // Read RTC_CNTL_RDY_FOR_WAKEUP bit
    AND r0, r0, 1
    JUMP is_rdy_for_wakeup, eq    // Retry until the bit is set
    WAKE                    // Trigger wake up    

.global exit
exit:   /* end the program */
    halt                   // on halt the ulp will start sleeping and it will automatically restart after ULP_SENSOR_PERIOD has passed
    jump  entry          // shouldn't be necessary 

