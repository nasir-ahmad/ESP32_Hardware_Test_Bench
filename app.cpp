#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_log.h"

#define MCPWM_FREQ_HZ 100000  // 100 kHz frequency
#define PWM_CH1 12
#define PWM_CH2 13

void config_mcpwm(void) {
  // Initialize MCPWM unit
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_CH1);  // Set MCPWM0A pin
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, PWM_CH2);  // Set MCPWM0B pin

  // Initialize MCPWM configuration
  mcpwm_config_t pwm_config = {
    .frequency = MCPWM_FREQ_HZ,  // Frequency of PWM
    .cmpr_a = 50.0,              // Duty cycle of PWMxA = 50%
    .cmpr_b = 50.0,              // Duty cycle of PWMxB = 50%
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER
  };

  // Initialize MCPWM unit with the configuration
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

  // Enable complementary mode
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 0, 0);
}

void setup(void) {  
  config_mcpwm();
}

void loop() {
  static float duty_cycle = 0.0;
  static int increment = 1;  // Increment direction

  // Set the duty cycle for PWM0A and PWM0B
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, duty_cycle);

  // Increment or decrement duty cycle
  duty_cycle += increment;

  // Reverse direction at the edge
  if (duty_cycle >= 100.0 || duty_cycle <= 0.0) {
    increment = -increment;  // Reverse direction
  }

  // Delay for smooth transition
  vTaskDelay(pdMS_TO_TICKS(10));
}
