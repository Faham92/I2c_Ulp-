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

   .global gas_range
gas_range:
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

    .global calc_gas
    .global entry
    .global wakeup
entry:   
    // Main code  
    move r3,stackEnd
    jump calc_gas

calc_gas:
   
    i2crd 0x2C 0x77//fetch value in register gas_adc
    lsh r0,r0,2
    move r2,r0
    i2crd 0x2D 0x77//fetch value in register gas_range
    rsh r0,r0,6
    or r0,r2,r0
    move r2, gas_adc
    st r0,r2,0
    wait 50000
    move r2,gas_range
    i2crd 0x2D 0x77
    and r0,r0,0x0F
    st r0,r2,0
  

    i2cwr 0x74 0x01 0x77
    wait 50000
    wait 50000
    wait 50000
    wait 50000
    wait 50000
    wait 50000
     wait 50000
    wait 50000
    wait 50000
    wait 50000
    wait 50000
    wait 50000
     wait 50000
    wait 50000
    wait 50000
    wait 50000
    wait 50000
    wait 50000
     wait 50000
    wait 50000
    wait 50000
    wait 50000
    wait 50000
    wait 50000
      wait 50000
    wait 50000
    wait 50000
    wait 50000
      wait 50000
    wait 50000
    wait 50000
    wait 50000
      wait 50000
    wait 50000
    wait 50000
    wait 50000
      wait 50000
    wait 50000
    wait 50000
    wait 50000
      wait 50000
    wait 50000
    wait 50000
    wait 50000
    jump calc_gas
    
    
    // Wake up ESP32
  /*wakeup:
    WAKE          
    HALT             

.global exit
exit:   
    halt                   // on halt the ulp will start sleeping and it will automatically restart after ULP_SENSOR_PERIOD has passed
    jump  entry          // shouldn't be necessary 

*/