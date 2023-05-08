#include <stdio.h>
#include <math.h>
#include "esp_sleep.h"
#include "sdkconfig.h"
#include "driver/rtc_io.h"
#include "esp32/ulp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#include "ulp_main.h"


// Unlike the esp-idf always use these binary blob names
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

#define ACK_CHECK_EN    0x1     // I2C master will check ack from slave
#define ACK_CHECK_DIS   0x0     // I2C master will not check ack from slave
#define ACK_VAL         0x0     // I2C ack value
#define NACK_VAL        0x1     // I2C nack value



#define BME688       0x77
#define BME688_W     0x00
#define BME688_R     0x01


void ulp_configure_i2c_pins();
void init_ulp();
void start_ulp_program();
static void init_i2c(void);
void init_bme688(void);
static esp_err_t i2c_write(i2c_port_t i2c_num, uint8_t* data_wr, size_t size);
static esp_err_t i2c_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t cmd);
static esp_err_t i2c_read(i2c_port_t i2c_num, uint8_t* data_rd, size_t size);
static esp_err_t i2c_read_int16(i2c_port_t i2c_num, uint8_t reg, int16_t* value);
static esp_err_t i2c_read_gas_adc(i2c_port_t i2c_num, uint8_t reg, uint16_t* value);
static esp_err_t i2c_read_uint8(i2c_port_t i2c_num, uint8_t reg, uint8_t* value);
static esp_err_t i2c_read_int8(i2c_port_t i2c_num, uint8_t reg, int8_t* value);
uint8_t calc_rest_heat(uint16_t amb_temp,int8_t par_g1,int16_t par_g2,int8_t par_g3,int16_t target_temp,uint8_t res_heat_range,int8_t res_heat_val);
static float calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range);


void app_main() 
{
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_34,0);          //Prévient le microcontroleur qu'il peut etre reveille par une pin rtc
  init_i2c();
  uint8_t gas_range,i=0;
  uint16_t gas_adc;
  float gas_resistance;
  init_i2c();
  vTaskDelay(3000/portTICK_PERIOD_MS);
   if ((ulp_flag&0xFFFF)!=0)
   {
    init_bme688(); 
    while(i<180)
      { 
        i++;
        vTaskDelay(1000/portTICK_PERIOD_MS);
        i2c_write_reg(I2C_NUM_0,0x74,0x01);//set mode to forced mode
      }
   }
  if (cause != ESP_SLEEP_WAKEUP_EXT0)
  {
   init_ulp();
  }
   else
  {
    i2c_read_gas_adc(I2C_NUM_0,0x2C,&gas_adc); //fetch value in register gas_adc    
    i2c_read_uint8(I2C_NUM_0,0x2D,&gas_range); //fetch value in register gas_range   
    gas_range &= 0x0F;
    ESP_LOGE("[Main]","gas_range= %d gas_adc=%d",gas_range,gas_adc);
    gas_resistance = calc_gas_resistance(gas_adc,gas_range);
    ESP_LOGE("[Main]","gas_resistance =%f kOhm --> Alarme sent",gas_resistance/1000);
    ulp_run((&ulp_entry2-RTC_SLOW_MEM));
    vTaskDelay(1000/portTICK_PERIOD_MS);
    esp_deep_sleep_start();
  }
   ulp_run((&ulp_entry-RTC_SLOW_MEM)); 
   ESP_LOGE("[Main]","Going to Deep Sleep, everything is fine");
   esp_deep_sleep_start(); 
}


void init_bme688(void)
{
  uint8_t heat_range;
  int16_t g2;
  int8_t g1,g3,heat_val;
  i2c_write_reg(I2C_NUM_0,0xE0,0xB6);
  vTaskDelay(1000/portTICK_PERIOD_MS);
  i2c_write_reg(I2C_NUM_0,0x72,0x01); //set humidity
  i2c_write_reg(I2C_NUM_0,0x74,0x54); //set pressure and temperature
  i2c_write_reg(I2C_NUM_0,0x75,0x02<<3); // set filter
  i2c_write_reg(I2C_NUM_0,0x71,0x20); //set run_gas to 1 and nb_conv to 0
  i2c_read_int8(I2C_NUM_0,0xED,&g1);
  i2c_read_int16(I2C_NUM_0,0xEB,&g2);
  i2c_read_int8(I2C_NUM_0,0xEE,&g3);
  //ESP_LOGE("[Main]","g1=%d g2=%d g3 =%d ",g1,g2,g3);
  i2c_read_uint8(I2C_NUM_0,0x02,&heat_range);
  i2c_read_int8(I2C_NUM_0,0x00,&heat_val);
  heat_range= heat_range>>4;
  heat_range &= 0x03;
  uint8_t res_heat = calc_rest_heat((uint16_t)25,g1,g2,g3,(int16_t)312,heat_range,heat_val); //calculate res_heat
  //ESP_LOGE("[Main]","res =%d ",res_heat);
  i2c_write_reg(I2C_NUM_0,0x5A,res_heat); //set heater temperature
  i2c_write_reg(I2C_NUM_0,0x64,0x59); //duration 100ms
  i2c_write_reg(I2C_NUM_0,0x74,0x01);//set mode to forced mode
  
}

void start_ulp_program()
{

    vTaskDelay(100/portTICK_PERIOD_MS);
    ulp_load_binary(0, ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
}

void ulp_configure_i2c_pins()
{
    
  /*Initialisation des pins RTC*/
   rtc_gpio_init(GPIO_NUM_4); //scl
   rtc_gpio_init(GPIO_NUM_0);//sda
   rtc_gpio_init(GPIO_NUM_34);
   rtc_gpio_init(GPIO_NUM_32);

   rtc_gpio_pullup_en(GPIO_NUM_4);
   rtc_gpio_pullup_en(GPIO_NUM_0);

   rtc_gpio_set_level(GPIO_NUM_32,1);
   

   rtc_gpio_set_direction(GPIO_NUM_4,RTC_GPIO_MODE_INPUT_ONLY);
   rtc_gpio_set_direction(GPIO_NUM_0,RTC_GPIO_MODE_INPUT_ONLY);
   rtc_gpio_set_direction(GPIO_NUM_34,RTC_GPIO_MODE_INPUT_ONLY);
   rtc_gpio_set_direction(GPIO_NUM_32,RTC_GPIO_MODE_OUTPUT_ONLY);
}
 

static esp_err_t i2c_write(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();  
    i2c_master_start(cmd);  
    i2c_master_write_byte(cmd,BME688<<1|BME688_W, ACK_CHECK_EN);
    i2c_master_write(cmd,data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t i2c_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t cmd)
{
    uint8_t data_wr[] = {reg, cmd};
    esp_err_t err = i2c_write(i2c_num, data_wr, 2);
    if (err != ESP_OK) {
        ESP_LOGI("[TAG]", "Write [0x%02x] = 0x%02x failed, err = %d", data_wr[0], data_wr[1], err);
    }
    return err;
}


static esp_err_t i2c_read(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,BME688<<1|BME688_R,ACK_VAL);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size -1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_uint8(i2c_port_t i2c_num, uint8_t reg, uint8_t* value)
{
    esp_err_t err = i2c_write(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[1] = {0};
        err = i2c_read(i2c_num, data_rd, 1);
        if (err == ESP_OK) {
            *value = data_rd[0];
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE("[ESP]", "Read [0x%02x] int16 failed, err = %d", reg, err);
    }
    return err;
}
static esp_err_t i2c_read_int8(i2c_port_t i2c_num, uint8_t reg, int8_t* value)
{
    esp_err_t err = i2c_write(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[1] = {0};
        err = i2c_read(i2c_num, data_rd, 1);
        if (err == ESP_OK) {
            *value = data_rd[0];
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE("[ESP]", "Read [0x%02x] int16 failed, err = %d", reg, err);
    }
    return err;
}


static esp_err_t i2c_read_int16(i2c_port_t i2c_num, uint8_t reg, int16_t* value)
{
    esp_err_t err = i2c_write(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[2] = {0};
        err = i2c_read(i2c_num, data_rd, 2);
        if (err == ESP_OK) {
            *value = (data_rd[1]<<8|data_rd[0]);
           // ESP_LOGE("[Read]","EC= %d EB=%d",data_rd[1],data_rd[0]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE("[ESP]", "Read [0x%02x] int16 failed, err = %d", reg, err);
    }
    return err;
}
static esp_err_t i2c_read_gas_adc(i2c_port_t i2c_num, uint8_t reg, uint16_t* value)
{
      esp_err_t err = i2c_write(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[2] = {0};
        err = i2c_read(i2c_num, data_rd, 2);
        if (err == ESP_OK) {
           // ESP_LOGE("[Read]","2C= %d 2D=%d",data_rd[0],data_rd[1]);
           // data_rd[1] &= 0xC0;
            data_rd[1] >>= 6;
            *value = (data_rd[0]<<2|data_rd[1]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE("[ESP]", "Read [0x%02x] int16 failed, err = %d", reg, err);
    }
    return err; 
}
static void init_i2c()
{
  esp_err_t err;

    i2c_config_t conf = {                                    //Configuration des pins i2c
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = GPIO_NUM_22,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0,
    };
    err = i2c_param_config(I2C_NUM_0, &conf);             
    if (err != ESP_OK) {
        ESP_LOGE("[INI_ERROR]", "I2C driver configuration failed with error = %d", err);
        
    }
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);  //installation du driver i2c
    if (err != ESP_OK) {
        ESP_LOGE("[INI_ERROR]", "I2C driver installation failed with error = %d", err);
       
    }else {
    ESP_LOGI("[INI_SUCCESS]", "I2C master driver has been installed.");
    }
}

/* This internal API is used to calculate the heater resistance value using float*/
uint8_t calc_rest_heat(uint16_t amb_temp,int8_t par_g1,int16_t par_g2,int8_t par_g3,int16_t target_temp,uint8_t res_heat_range,int8_t res_heat_val)
{

    double var1;
    double var2;
    double var3;
    double var4;
    double var5;
    uint8_t res_heat;

    if (target_temp > 400) /* Cap temperature */
    {
        target_temp = 400;
    }

        var1 = ((double)par_g1 / 16.0) + 49.0;
        var2 = (((double)par_g2 / 32768.0) * 0.0005) + 0.00235;
        var3 = (double)par_g3 / 1024.0;
        var4 = var1 * (1.0 + (var2 * (double) target_temp));
        var5 = var4 + (var3 * (double)amb_temp);
        res_heat = (uint8_t)(3.4 * ((var5 * (4.0 / (4.0 + (double)res_heat_range)) * (1.0/(1.0 +
        ((double)res_heat_val * 0.002)))) - 25));

    return res_heat;

}

/* This internal API is used to calculate the gas resistance */
static float calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range)
{
        uint32_t var1 = UINT32_C(262144) >> gas_range;
        int32_t var2 = (int32_t) gas_res_adc - INT32_C(512);
        var2 *= INT32_C(3);
        var2 = INT32_C(4096) + var2;
        float gas_res = 1000000.0f * (float)var1 / (float)var2;

    return gas_res;
}

void init_ulp (void)
{
  ulp_configure_i2c_pins();
  start_ulp_program();
  //ulp_set_wakeup_period(0,30 * 1000000);
}