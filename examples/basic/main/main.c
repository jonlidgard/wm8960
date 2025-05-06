//
// Created by Jon Lidgard on 06/05/2025.
//

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "wm8960.h"

#define I2C_MASTER_SCL_IO           CONFIG_WM8960_I2C_SCL_PIN    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_WM8960_I2C_SDA_PIN     /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_FREQ_HZ          CONFIG_WM8960_I2C_MASTER_FREQUENCY                 /*!< I2C master clock frequency */

/*!< I2C port number for master dev */
#ifdef CONFIG_WM89690_I2C_MASTER_PORT_NUM_0
    #define I2C_MASTER_NUM              I2C_NUM_0
#else
    #define I2C_MASTER_NUM              I2C_NUM_1
#endif

static const char *TAG = "WM8960_DEMO";

i2c_master_bus_handle_t bus_handle;
wm8960_t wm8960;

static esp_err_t i2c_master_init(i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_config, bus_handle);
}


/*******************************
 * MAIN ENTRY POINT
 ******************************/


void app_main(void)
{

  wm8960.i2c_config.dev_addr_length = I2C_ADDR_BIT_LEN_7,
  wm8960.i2c_config.device_address = WM8960_ADDR,
  wm8960.i2c_config.scl_speed_hz = I2C_MASTER_FREQ_HZ,

  i2c_master_init(&bus_handle);
  ESP_LOGI(TAG, "I2C Bus initialized successfully");

  i2c_master_bus_add_device(bus_handle, &wm8960.i2c_config, &wm8960.handle);
  ESP_LOGI(TAG, "I2C WM8960 initialized successfully");


  // General setup needed
  wm8960_reset(&wm8960); // Must be called first
  wm8960_enableVREF(&wm8960);
  wm8960_enableVMID(&wm8960);

  // Connect from DAC outputs to output mixer
  wm8960_enableLD2LO(&wm8960);
  wm8960_enableRD2RO(&wm8960);

  // Set gainstage between booster mixer and output mixer
  // For this loopback example, we are going to keep these as low as they go
  wm8960_setLB2LOVOL(&wm8960, WM8960_OUTPUT_MIXER_GAIN_NEG_21DB);
  wm8960_setRB2ROVOL(&wm8960, WM8960_OUTPUT_MIXER_GAIN_NEG_21DB);

  // Enable output mixers
  wm8960_enableLOMIX(&wm8960);
  wm8960_enableROMIX(&wm8960);

  // CLOCK STUFF, These settings will get you 44.1KHz sample rate, and class-d
  // freq at 705.6kHz
  wm8960_enablePLL(&wm8960); // Needed for class-d amp clock
  wm8960_setPLLPRESCALE(&wm8960, WM8960_PLLPRESCALE_DIV_2);
  wm8960_setSMD(&wm8960, WM8960_PLL_MODE_FRACTIONAL);
  wm8960_setCLKSEL(&wm8960, WM8960_CLKSEL_PLL);
  wm8960_setSYSCLKDIV(&wm8960, WM8960_SYSCLK_DIV_BY_2);
  wm8960_setBCLKDIV(&wm8960, 4);
  wm8960_setDCLKDIV(&wm8960, WM8960_DCLKDIV_16);
  wm8960_setPLLN(&wm8960, 7);
  wm8960_setPLLK(&wm8960, 0x86, 0xC2, 0x26); // PLLK=86C226h
  //wm8960_setADCDIV(&wm8960, 0); // Default is 000 (what we need for 44.1KHz)
  //wm8960_setDACDIV(&wm8960, 0); // Default is 000 (what we need for 44.1KHz)
  wm8960_setWL(&wm8960, WM8960_WL_16BIT);

  wm8960_enablePeripheralMode(&wm8960);
  //wm8960_enableMasterMode(&wm8960);
  //wm8960_setALRCGPIO(&wm8960); // Note, should not be changed while ADC is enabled.

  wm8960_enableDacLeft(&wm8960);
  wm8960_enableDacRight(&wm8960);

  //wm8960_enableLoopBack(&wm8960); // Loopback sends ADC data directly into DAC
  wm8960_disableLoopBack(&wm8960);

  // Default is "soft mute" on, so we must disable mute to make channels active
  wm8960_disableDacMute(&wm8960);

//  wm8960_enableHeadphones(&wm8960);
 // wm8960_enableOUT3MIX(&wm8960); // Provides VMID as buffer for headphone ground

  wm8960_enableSpeakers(&wm8960);

  ESP_LOGI(TAG, "Volume set to +0dB");
//  wm8960_setHeadphoneVolumeDB(&wm8960, 0.00);
  wm8960_setSpeakerVolumeDB(&wm8960, 0);
}