// I2C.s
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_i2c_reg.h"


#include "stack.S"

/*
 * =============================== I2C code ==========================================
 * Implementation of pseudo code from
 * https://en.wikipedia.org/wiki/I%C2%B2C#Example_of_bit-banging_the_I.C2.B2C_master_protocol
 */



  .global i2c_started

i2c_started:
  .long 0

  .global i2c_didInit

i2c_didInit:
  .long 0
  
  .text 
.global i2c_start_cond
.global i2c_stop_cond
.global i2c_write_bit
.global i2c_read_bit
.global i2c_write_byte
.global i2c_read_byte


// TOUCH0 = GPIO4 = SCL = RTC_GPIO10
// TOUCH1 = GPIO0 = SDA = RTC_GPIO11

.macro read_SCL // Return current level of SCL line, 0 or 1
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 10, 1) // RTC_GPIO_10 == GPIO_04
.endm

.macro read_SDA // Return current level of SDA line, 0 or 1
  READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 11, 1) // RTC_GPIO_11 == GPIO_00
.endm

.macro set_SCL // Do not drive SCL (set pin high-impedance)
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + 10, 1, 1)
.endm

.macro clear_SCL // Actively drive SCL signal low
  // Output mode
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + 10, 1, 1)
.endm

.macro set_SDA // Do not drive SDA (set pin high-impedance)
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TC_REG, RTC_GPIO_ENABLE_W1TC_S + 11, 1, 1)
.endm

.macro clear_SDA // Actively drive SDA signal low
  // Output mode
  WRITE_RTC_REG(RTC_GPIO_ENABLE_W1TS_REG, RTC_GPIO_ENABLE_W1TS_S + 11, 1, 1)
.endm
.macro I2C_delay
  wait 10 // 38 // minimal 4.7us
.endm


i2c_start_cond:
  move r1,i2c_didInit
  ld r0,r1,0
  jumpr didInit,1,ge
  move r0,1
  st r0,r1,0
// set GPIO to pull low when activated
  WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 11 ,1, 0)
  WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 10, 1, 0)
didInit:
  move r2,i2c_started
  ld r0,r2,0
  jumpr not_started,1,lt
// if started, do a restart condition
// set SDA to 1
  set_SDA
  I2C_delay
  set_SCL
clock_stretch: // TODO: Add timeout?
  read_SCL
  jumpr clock_stretch,1,lt

// Repeated start setup time, minimum 4.7us
  I2C_delay

not_started:
  // if (read_SDA() == 0) {
  //    arbitration_lost();
  // }

// SCL is high, set SDA from 1 to 0.
  clear_SDA
  I2C_delay
  clear_SCL
  move r0,1
  st r0,r2,0

  ret


i2c_stop_cond:
// set SDA to 0
  clear_SDA
  I2C_delay

  set_SCL
clock_stretch_stop:
  read_SCL
  jumpr clock_stretch_stop,1,lt

// Stop bit setup time, minimum 4us
  I2C_delay

// SCL is high, set SDA from 0 to 1
  set_SDA
  I2C_delay
  // if (read_SDA() == 0) {
  //    arbitration_lost();
  // }

  move r2,i2c_started
  move r0,0
  st r0,r2,0

  ret


// Write a bit to I2C bus
i2c_write_bit:
  jumpr bit0,1,lt
  set_SDA
  jump bit1
bit0:
  clear_SDA
bit1:

// SDA change propagation delay
  I2C_delay
// Set SCL high to indicate a new valid SDA value is available
  set_SCL
// Wait for SDA value to be read by slave, minimum of 4us for standard mode
  I2C_delay

clock_stretch_write:
  read_SCL
  jumpr clock_stretch_write,1,lt

  // SCL is high, now data is valid
  // If SDA is high, check that nobody else is driving SDA
  // if (bit && (read_SDA() == 0)) {
  //    arbitration_lost();
  // }

  // Clear the SCL to low in preparation for next change
  clear_SCL

  ret


// Read a bit from I2C bus
i2c_read_bit:
// Let the slave drive data
  set_SDA
// Wait for SDA value to be written by slave, minimum of 4us for standard mode
  I2C_delay
// Set SCL high to indicate a new valid SDA value is available
  set_SCL

clock_stretch_read:
  read_SCL
  jumpr clock_stretch_read,1,lt

// Wait for SDA value to be written by slave, minimum of 4us for standard mode
  I2C_delay
// SCL is high, read out bit
  read_SDA
// Set SCL low in preparation for next operation
  clear_SCL

  ret // bit in r0

// Write a byte to I2C bus. Return 0 if ack by the slave.
i2c_write_byte:
  stage_rst
next_bit:
  and r0,r2,0x80
  psr
  jump i2c_write_bit
  lsh r2,r2,1
  stage_inc 1
  jumps next_bit,8,lt

  psr
  jump i2c_read_bit
  ret // nack


// Read a byte from I2C bus
i2c_read_byte:
  push r2
  move r2,0
  stage_rst
next_bit_read:
  psr
  jump i2c_read_bit
  lsh r2,r2,1
  or r2,r2,r0
  stage_inc 1
  jumps next_bit_read,8,lt

  pop r0
  psr
  jump i2c_write_bit

  move r0,r2

  ret


/*
 * I2C ULP utility routines
 */

write_intro:
  psr
  jump i2c_start_cond

  ld r2,r3,20 // Address
  lsh r2,r2,1
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge

  ld r2,r3,16 // Register
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge
  ret


.global write8
write8:
  psr
  jump write_intro

write_b:
  ld r2,r3,8 // data byte
  psr
  jump i2c_write_byte
  jumpr fail,1,ge

  psr
  jump i2c_stop_cond

  move r2,0 // Ok
  ret


read_intro:
  psr
  jump i2c_start_cond

  ld r2,r3,16 // Address
  lsh r2,r2,1
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge

  ld r2,r3,12 // Register
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge

  psr
  jump i2c_start_cond

  ld r2,r3,16
  lsh r2,r2,1
  or r2,r2,1 // Address Read
  psr
  jump i2c_write_byte
  jumpr popfail,1,ge

  ret
popfail:
  pop r1 // pop caller return address
  move r2,1
  ret

.global read8
read8:
  psr
  jump read_intro

  move r2,1 // last byte
  psr
  jump i2c_read_byte
  push r0

  psr
  jump i2c_stop_cond

  pop r0

  move r2,0 // OK
  ret
fail:
  move r2,1
  ret