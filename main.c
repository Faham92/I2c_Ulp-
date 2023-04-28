#include <stdio.h>
#include <math.h>
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

typedef struct {
    uint32_t scl_low;       /*!< FAST_CLK cycles for SCL to be low [19b] */
    uint32_t scl_high;      /*!< FAST_CLK cycles for SCL to be high [20b] */
    uint32_t sda_duty;      /*!< FAST_CLK cycles SDA will switch after falling edge of SCL [20b] */
    uint32_t scl_start;     /*!< FAST_CLK cycles to wait before generating start condition [20b] */
    uint32_t scl_stop;      /*!< FAST_CLK cycles to wait before generating stop condition [20b] */
    uint32_t timeout;       /*!< Maximum number of FAST_CLK cycles that the transmission can take [20b] */
}hulp_i2c_controller_config_t;

void ulp_configure_i2c_pins();
void init_ulp();
esp_err_t ulp_configure_i2c_controller( hulp_i2c_controller_config_t* );
void register_i2c_slave(uint8_t );
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

RTC_DATA_ATTR hulp_i2c_controller_config_t config = 
{   
    .scl_low = 16,           //initialisation fait à la page 664 de la technical reference de l'esp32            
    .scl_high = 15,                         
    .sda_duty = 13,                         
    .scl_start = 21,                        
    .scl_stop = 25,                         
    .timeout = 1000,                 };

void app_main() 
{
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  //uint8_t gas_range,new_data;
 // uint16_t gas_adc;
  int i=0;
  float gas_resistance;
  esp_sleep_enable_ulp_wakeup();                       //Prévient le microcontroleur qu'il peut etre reveille par l'ulp
  init_i2c();
  vTaskDelay(3000/portTICK_PERIOD_MS);
  init_bme688();
 // vTaskDelay(1000*60/portTICK_PERIOD_MS);
  i2c_write_reg(I2C_NUM_0,0x74,0x01); //set forced mode
  vTaskDelay(2000/portTICK_PERIOD_MS);
  if (cause != ESP_SLEEP_WAKEUP_ULP)
  {
   init_ulp();
   vTaskDelay(1000/portTICK_PERIOD_MS);
  }
   ulp_run((&ulp_entry-RTC_SLOW_MEM)); 
   vTaskDelay(5000/portTICK_PERIOD_MS);
  
  while (1)
  {
   // i2c_read_uint8(I2C_NUM_0,0x2D,&heat_stab);
   // heat_stab=0x10;
   // heat_stab >>=4;
   // i2c_read_int8(I2C_NUM_0,0x1D,&heat_val);
   // heat_val &= 0x40;
   // heat_val >>= 6;
    ///i2c_read_uint8(I2C_NUM_0,0x1D,&new_data);
   // if(new_data==0x80)
   // {
        //i2c_read_gas_adc(I2C_NUM_0,0x2C,&gas_adc); //fetch value in register gas_adc    
        //i2c_read_uint8(I2C_NUM_0,0x2D,&gas_range); //fetch value in register gas_range   
        //gas_range &= 0x0F;
        i++;
        // uint16_t flag = ulp_flag & 0xffff;
        // uint16_t gas_adc = ulp_gas_adc & 0xffff ;
       // ESP_LOGE("[Main]","flag= 0x%X",flag);
        //ESP_LOGE("[Main]","gas_range =0x%lX",ulp_gas_range&0xFFFF);
        //ESP_LOGE("[Main]","flag= 0x%X",flag);
         ESP_LOGE("[Main]","gas_adc =%lu",ulp_gas_adc&0xFFFF);
       // gas_resistance = calc_gas_resistance(gas_adc,gas_range);
      //  ESP_LOGE("[Main]","gas_resistance =%f kOhm",gas_resistance/1000);
         vTaskDelay(1000/portTICK_PERIOD_MS);
        //i2c_write_reg(I2C_NUM_0,0x74,0x01);
   //}
  }
  //uint8_t res = (uint8_t)ulp_resu & (uint8_t)0xFF;
 /* while(1){
  uint16_t res = (uint16_t)ulp_resu & (uint16_t)0xFF;*/
   //ulp_run((&ulp_entry-RTC_SLOW_MEM)); 
   //vTaskDelay(100/portTICK_PERIOD_MS);
   //ESP_LOGE("[Main]","Going to Deep Sleep");
   //esp_deep_sleep_start(); 
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
  ESP_LOGE("[Main]","g1=%d g2=%d g3 =%d ",g1,g2,g3);
  i2c_read_uint8(I2C_NUM_0,0x02,&heat_range);
  i2c_read_int8(I2C_NUM_0,0x00,&heat_val);
  heat_range= heat_range>>4;
  heat_range &= 0x03;
  uint8_t res_heat = calc_rest_heat((uint16_t)20,g1,g2,g3,(int16_t)312,heat_range,heat_val); //calculate res_heat
  ESP_LOGE("[Main]","res =%d ",res_heat);
  i2c_write_reg(I2C_NUM_0,0x5A,res_heat); //set heater temperature
  i2c_write_reg(I2C_NUM_0,0x64,0x59); //duration 100ms
  //i2c_write_reg(I2C_NUM_0,0x74,0x01);//set mode to forced mode
  

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

   rtc_gpio_pullup_en(GPIO_NUM_4);
   rtc_gpio_pullup_en(GPIO_NUM_0);

    rtc_gpio_set_direction(GPIO_NUM_4,RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_direction(GPIO_NUM_0,RTC_GPIO_MODE_INPUT_ONLY);

   /* const int scl_rtcio_num = rtc_io_number_get(GPIO_NUM_4);
    SET_PERI_REG_BITS(rtc_io_desc[scl_rtcio_num].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x3, rtc_io_desc[scl_rtcio_num].func);
    const int sda_rtcio_num = rtc_io_number_get(GPIO_NUM_0);
    SET_PERI_REG_BITS(rtc_io_desc[sda_rtcio_num].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x3, rtc_io_desc[sda_rtcio_num].func);

   REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SCL_SEL, 0); // Touch pad 0 GPIO4 RTC_GPIO_10
   REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SDA_SEL, 0);// Touch pad 1 GPIO0 RTC_GPIO_11*/
}
 

esp_err_t ulp_configure_i2c_controller(hulp_i2c_controller_config_t *conf)
{
    //REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_RX_LSB_FIRST, 0);
    //REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_TX_LSB_FIRST, 0);
    //REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SCL_FORCE_OUT, 0);
    //REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SDA_FORCE_OUT, 0);
     WRITE_PERI_REG(SENS_SAR_I2C_CTRL_REG, 0);
    REG_SET_FIELD(RTC_I2C_SCL_LOW_PERIOD_REG, RTC_I2C_SCL_LOW_PERIOD, conf->scl_low);
    REG_SET_FIELD(RTC_I2C_SCL_HIGH_PERIOD_REG, RTC_I2C_SCL_HIGH_PERIOD, conf->scl_high);
    REG_SET_FIELD(RTC_I2C_SDA_DUTY_REG, RTC_I2C_SDA_DUTY, conf->sda_duty);
    REG_SET_FIELD(RTC_I2C_SCL_START_PERIOD_REG, RTC_I2C_SCL_START_PERIOD, conf->scl_start);
    REG_SET_FIELD(RTC_I2C_SCL_STOP_PERIOD_REG, RTC_I2C_SCL_STOP_PERIOD, conf->scl_stop);
    REG_SET_FIELD(RTC_I2C_TIMEOUT_REG, RTC_I2C_TIMEOUT, conf->timeout);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_MS_MODE, 1);
    return ESP_OK;
}

void register_i2c_slave(uint8_t reg)
{
  SET_PERI_REG_BITS(SENS_SAR_SLAVE_ADDR1_REG,SENS_I2C_SLAVE_ADDR0, reg, SENS_I2C_SLAVE_ADDR0_S);
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
            ESP_LOGE("[Read]","2C= %d 2D=%d",data_rd[0],data_rd[1]);
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
  //ulp_configure_i2c_controller(&config);
  //register_i2c_slave(0x77);
  start_ulp_program();
}