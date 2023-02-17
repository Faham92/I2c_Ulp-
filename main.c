#include <stdio.h>
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "soc/soc.h"
#include "soc/sens_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/rtc_i2c_reg.h"
#include "soc/rtc_io_periph.h"

#include "ulp_main.h"


// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

typedef struct {
    uint32_t scl_low;       /*!< FAST_CLK cycles for SCL to be low [19b] */
    uint32_t scl_high;      /*!< FAST_CLK cycles for SCL to be high [20b] */
    uint32_t sda_duty;      /*!< FAST_CLK cycles SDA will switch after falling edge of SCL [20b] */
    uint32_t scl_start;     /*!< FAST_CLK cycles to wait before generating start condition [20b] */
    uint32_t scl_stop;      /*!< FAST_CLK cycles to wait before generating stop condition [20b] */
    uint32_t timeout;       /*!< Maximum number of FAST_CLK cycles that the transmission can take [20b] */
    bool scl_pushpull;      /*!< SCL is push-pull (true) or open-drain (false) */
    bool sda_pushpull;      /*!< SDA is push-pull (true) or open-drain (false) */
    bool rx_lsbfirst;       /*!< Receive LSB first */
    bool tx_lsbfirst;       /*!< Send LSB first */
}hulp_i2c_controller_config_t;

void hulp_configure_i2c_pins(gpio_num_t , gpio_num_t );
esp_err_t hulp_configure_i2c_controller( hulp_i2c_controller_config_t* );
void register_i2c_slave(uint8_t );
static void start_ulp_program();
void ini_config(hulp_i2c_controller_config_t *);

RTC_DATA_ATTR hulp_i2c_controller_config_t* config;
gpio_num_t sda_pin = 4;
gpio_num_t scl_pin = 0;

void app_main() 
{
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  esp_sleep_enable_ulp_wakeup();                       //Prévient le microcontroleur qu'il peut etre reveille par l'ulp
  if (cause != ESP_SLEEP_WAKEUP_ULP)
  {
    hulp_configure_i2c_pins(scl_pin, sda_pin);
    ini_config(config);
    hulp_configure_i2c_controller(config);
    start_ulp_program();
  }
  ulp_run((&ulp_entry-RTC_SLOW_MEM));
  register_i2c_slave(0x77);
 
}


// Use this function for all your first time init like setup() used to be
/*static void init_run_ulp() 
{
  
}
*/
static void start_ulp_program()
{
    vTaskDelay(100/portTICK_PERIOD_MS);
    ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
}

void hulp_configure_i2c_pins(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
  /*Initialisation des pins RTC*/
    rtc_gpio_init(sda_pin);
    rtc_gpio_init(scl_pin);
    rtc_gpio_set_direction(sda_pin,RTC_GPIO_MODE_INPUT_OUTPUT);
    rtc_gpio_set_direction(scl_pin,RTC_GPIO_MODE_INPUT_OUTPUT);

    const int scl_rtcio_num = rtc_io_number_get(scl_pin);
    SET_PERI_REG_BITS(rtc_io_desc[scl_rtcio_num].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x3, rtc_io_desc[scl_rtcio_num].func);
    const int sda_rtcio_num = rtc_io_number_get(sda_pin);
    SET_PERI_REG_BITS(rtc_io_desc[sda_rtcio_num].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x3, rtc_io_desc[sda_rtcio_num].func);

    REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SCL_SEL, 0); // Touch pad 0 GPIO4 RTC_GPIO10
    REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SDA_SEL, 0);// Touch pad 1  GPIO0 RTC_GPIO11
}
 

esp_err_t hulp_configure_i2c_controller(hulp_i2c_controller_config_t *config)
{
    if(!config)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if(
        (config->scl_low > RTC_I2C_SCL_LOW_PERIOD_V) ||
        (config->scl_high > RTC_I2C_SCL_HIGH_PERIOD_V) ||
        (config->sda_duty > RTC_I2C_SDA_DUTY_V) ||
        (config->scl_start > RTC_I2C_SCL_START_PERIOD_V) ||
        (config->scl_stop > RTC_I2C_SCL_STOP_PERIOD_V) ||
        (config->timeout > RTC_I2C_TIMEOUT_V)
      )
    {
        return ESP_ERR_INVALID_ARG;
    }

    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_RX_LSB_FIRST, config->rx_lsbfirst);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_TX_LSB_FIRST, config->tx_lsbfirst);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SCL_FORCE_OUT, config->scl_pushpull);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SDA_FORCE_OUT, config->sda_pushpull);

    REG_SET_FIELD(RTC_I2C_SCL_LOW_PERIOD_REG, RTC_I2C_SCL_LOW_PERIOD, config->scl_low);
    REG_SET_FIELD(RTC_I2C_SCL_HIGH_PERIOD_REG, RTC_I2C_SCL_HIGH_PERIOD, config->scl_high);
    REG_SET_FIELD(RTC_I2C_SDA_DUTY_REG, RTC_I2C_SDA_DUTY, config->sda_duty);
    REG_SET_FIELD(RTC_I2C_SCL_START_PERIOD_REG, RTC_I2C_SCL_START_PERIOD, config->scl_start);
    REG_SET_FIELD(RTC_I2C_SCL_STOP_PERIOD_REG, RTC_I2C_SCL_STOP_PERIOD, config->scl_stop);
    REG_SET_FIELD(RTC_I2C_TIMEOUT_REG, RTC_I2C_TIMEOUT, config->timeout);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_MS_MODE, 1);
    return ESP_OK;
}

void register_i2c_slave(uint8_t reg)
{
  REG_SET_FIELD(SENS_SAR_SLAVE_ADDR1_REG,SENS_I2C_SLAVE_ADDR0, reg);
}

void ini_config(hulp_i2c_controller_config_t *config)
{
    
    config->scl_low = 40;                //initialisation fait à la page 664 de la technical reference de l'esp32            
    config->scl_high = 40;                         
    config->sda_duty = 16;                         
    config->scl_start = 30;                        
    config->scl_stop = 44;                         
    config->timeout = 200;                         
    config->scl_pushpull = false;                 
    config->sda_pushpull = false;                 
    config->rx_lsbfirst = false;                  
    config->tx_lsbfirst = false;
}
