/**
 * @brief ADC testing with calibration and noise filtering using ESP-IDF.
 *
 * This program demonstrates ADC sampling with full calibration and noise filtering
 * for precise voltage measurement on an ESP32. The following features are implemented:
 * 
 * - ADC calibration using eFuse values (Two Point or Vref) or default Vref if not available.
 * - Noise reduction through multi-sampling with an adjustable sample count (`NO_OF_SAMPLES`).
 * - Full attenuation (11 dB) for maximum input voltage range.
 * - Voltage conversion from raw ADC readings.
 *
 * Hardware and Configuration:
 * - ADC1 Channel 5 is used (corresponds to GPIO34).
 * - ADC is configured for 12-bit resolution and 11 dB attenuation.
 * - Default reference voltage (`DEFAULT_VREF`) is set to 1100 mV, 
 *   and calibration adjusts it for higher accuracy.
 *
 * Software Requirements:
 * - FreeRTOS is used for task management.
 * - ESP-IDF ADC and calibration libraries are utilized for hardware interfacing.
 *
 * Notes:
 * - This example can be adapted for ADC2 by modifying the `unit` and GPIO configuration.
 * - Adjust the `NO_OF_SAMPLES` value for balancing noise filtering and sampling speed.
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"


#define DEFAULT_VREF 1100  //Use adc_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64   //Multisampling
#define VREF_SHADOW_PIN GPIO_NUM_25 //VREF is output to this pin for clibration

static esp_adc_cal_characteristics_t *adc_chars;
static const adc1_channel_t channel = ADC1_CHANNEL_5;  //GPIO34 if ADC1, GPIO14 if ADC2

static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;


static void check_efuse(void) {
  //Check TP is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
    Serial.printf("eFuse Two Point: Supported\n");
  } else {
    Serial.printf("eFuse Two Point: NOT supported\n");
  }

  //Check Vref is burned into eFuse
  if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
    Serial.printf("eFuse Vref: Supported\n");
  } else {
    Serial.printf("eFuse Vref: NOT supported\n");
  }
}

static void print_char_val_type(esp_adc_cal_value_t val_type) {
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    Serial.printf("Characterized using Two Point Value\n");
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    Serial.printf("Characterized using eFuse Vref\n");
  } else {
    Serial.printf("Characterized using Default Vref\n");
  }
}

void setup() {

  Serial.begin(115200);

  vTaskDelay(pdMS_TO_TICKS(3000));

  //Check if Two Point or Vref are burned into eFuse
  check_efuse();

  //Configure ADC
  if (unit == ADC_UNIT_1) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);
  } else {
    adc2_config_channel_atten((adc2_channel_t)channel, atten);
  }

  //Characterize ADC
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
  print_char_val_type(val_type);


  esp_err_t status = adc_vref_to_gpio(ADC_UNIT_2, VREF_SHADOW_PIN);
  if (status == ESP_OK) {
    Serial.printf("v_ref routed to GPIO\n");
  } else {
    Serial.printf("failed to route v_ref\n");
  }
}


void loop() {
  //Continuously sample ADC1
  uint32_t adc_reading = 0;
  //Multisampling
  for (int i = 0; i < NO_OF_SAMPLES; i++) {
    if (unit == ADC_UNIT_1) {
      adc_reading += adc1_get_raw((adc1_channel_t)channel);
    } else {
      int raw;
      adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
      adc_reading += raw;
    }
  }
  adc_reading /= NO_OF_SAMPLES;

  //Convert adc_reading to voltage in mV
  uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
  Serial.printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);

  vTaskDelay(pdMS_TO_TICKS(1000));
}
