/**
 * @brief Configures MCPWM (Motor Control PWM) for complementary push-pull configuration.
 *        This implementation uses two PWM channels with complementary outputs and dead-time
 *        control for applications like power conversion, H-Bridge or other power electronics circuits.
 *        The frequency is set to 30 kHz, and the duty cycle will oscillate between 0% and 50%.
 * 
 * @details
 * This code configures two PWM channels (MCPWM0A and MCPWM1A) for a complementary push-pull output.
 * A 30 kHz PWM frequency is used, and acheve higher resolution the duty cycle will smoothly transition from 0% to 50% with a delay of 50 ms.
 * 
 * The configuration includes:
 * - PWM frequency set to 30 kHz.
 * - Dead-time insertion between complementary channels for proper switching.
 * - Timer synchronization to ensure accurate and synchronized operation between the two PWM channels.
 * - Smooth duty cycle modulation.
 * 
 * @note
 * Ensure that the microcontroller's GPIO pins are correctly connected to the power stage for your application.
 * The dead-time value should be adjusted based on your specific switching requirements.
 * 
 * 
 * @author Nasir Ahmad
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_log.h"

#define MCPWM_FREQ_HZ 30000  // 100 kHz frequency
#define PWM_CH1 12
#define PWM_CH2 13
#define SOC_MCPWM_BASE_CLK_HZ 160000000


void config_mcpwm(void) {

  // Initialize MCPWM unit for two channels
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_CH1);  // Set MCPWM0A pin
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_CH2);  // Set MCPWM1A pin


  mcpwm_group_set_resolution(MCPWM_UNIT_0, SOC_MCPWM_BASE_CLK_HZ);

  mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_0, SOC_MCPWM_BASE_CLK_HZ);
  mcpwm_timer_set_resolution(MCPWM_UNIT_0, MCPWM_TIMER_1, SOC_MCPWM_BASE_CLK_HZ);


  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_FREQ_HZ);
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_FREQ_HZ);


  // Initialize MCPWM configuration
  mcpwm_config_t pwm_config = {
    .frequency = MCPWM_FREQ_HZ,  // Frequency of PWM
    .cmpr_a = 0.0,               // Duty cycle of PWM0A = 50%
    .cmpr_b = 0.0,               // Duty cycle of PWM1A = 50%
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER
  };


  // Initialize MCPWM unit for both timers with the configuration
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);


  // Enable synchronization for both timers
  mcpwm_sync_config_t sync_config = {
    .sync_sig = MCPWM_SELECT_TIMER0_SYNC,
    .timer_val = 500,
    .count_direction = MCPWM_TIMER_DIRECTION_UP
  };

  mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);  // TIMER0: Send Sync Signal on counting to 0
  mcpwm_sync_configure(MCPWM_UNIT_0, MCPWM_TIMER_1, &sync_config);



  // Enable complementary mode with a 180-degree phase shift
  mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);
  //mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE, 10, 10);
}

void setup(void) {
  config_mcpwm();
}

void loop() {
  static float duty_cycle = 0.0;  // Initial duty cycle = 0%
  static float increment = 0.1;   // Increment direction (1 for increase, -1 for decrease)

  // Gradually increase or decrease the duty cycle
  duty_cycle += increment;

  // Reverse direction when limits are reached
  if (duty_cycle >= 50.0) {
    increment = -1;  // Start decreasing the duty cycle
  } else if (duty_cycle <= 0.0) {
    increment = 1;  // Start increasing the duty cycle
  }

  // Set the duty cycle for both PWM signals
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
  mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle);

  // Delay for smooth transition
  vTaskDelay(pdMS_TO_TICKS(50));  // Adjust the delay for smoother transitions
}
