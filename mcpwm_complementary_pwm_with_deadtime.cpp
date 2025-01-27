/**
 * @brief Example code for configuring and controlling PWM signals using the MCPWM peripheral on ESP32.
 * 
 * This example demonstrates how to set up two PWM signals with dead time and a sweeping duty cycle.
 * The code uses the MCPWM (Motor Control Pulse Width Modulation) module to generate PWM signals
 * on two GPIO pins. The duty cycle of the PWM signals is swept from 0% to 100% and back in a loop.
 * 
 * Key Features:
 * - Configures MCPWM timer, operator, comparator, and generators.
 * - Sets up dead time to prevent shoot-through in motor control applications.
 * - Sweeps the duty cycle of the PWM signals to demonstrate dynamic control.
 * 
 * Hardware Setup:
 * - Connect GPIO_PWM0A and GPIO_PWM0B to the PWM input of your device (e.g., motor driver).
 * - Replace GPIO_PWM0A and GPIO_PWM0B with the appropriate GPIO pins for your application.
 * 
 * Dependencies:
 * - FreeRTOS for task management.
 * - ESP-IDF MCPWM driver for PWM configuration.
 * 
 * Author: Nasir Ahmad
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"

// Define GPIOs for PWM outputs
#define GPIO_PWM0A 12  // Replace with your GPIO for PWM output A
#define GPIO_PWM0B 13  // Replace with your GPIO for PWM output B

// Global handle for the MCPWM comparator
static mcpwm_cmpr_handle_t comparator = NULL;

/**
 * @brief Setup function for initializing MCPWM components.
 * 
 * This function configures the MCPWM timer, operator, comparator, and generators.
 * It also sets up the dead time for the PWM signals and starts the timer.
 */
void setup() {
  // Configure MCPWM timer
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = 80 * 1000 * 1000,  // 80 MHz
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = 800,  // 100 kHz period (10 us / 12.5 ns = 800 ticks)
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  // Create operator
  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t oper_config = { .group_id = 0 };
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper));

  // Connect timer to operator
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

  // Create comparator
  mcpwm_comparator_config_t comparator_config = {};
  comparator_config.flags.update_cmp_on_tez = true;
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));

  // Create generators for PWM A and B
  mcpwm_gen_handle_t gen_a = NULL, gen_b = NULL;
  mcpwm_generator_config_t gen_config = { .gen_gpio_num = GPIO_PWM0A };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen_a));
  gen_config.gen_gpio_num = GPIO_PWM0B;
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config, &gen_b));

  // Configure generator actions
  // PWM A: High on timer start, Low on compare
  // Both generators start with same phase - dead time will invert one
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_a,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_a,
                                                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

  // Configure gen_b with SAME actions initially
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_b,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_b,
                                                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

  // Configure dead time (500 ns = 40 ticks @ 80 MHz)
  mcpwm_dead_time_config_t dead_time_config_a = {
    .posedge_delay_ticks = 30,
  };
  ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_a, gen_a, &dead_time_config_a));

  mcpwm_dead_time_config_t dead_time_config_b = {
    .negedge_delay_ticks = 30,
  };
  dead_time_config_b.flags.invert_output = true;
  ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_b, gen_b, &dead_time_config_b));

  // Start timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

/**
 * @brief Main loop for sweeping the PWM duty cycle.
 * 
 * This function continuously adjusts the duty cycle of the PWM signals,
 * sweeping from 0% to 100% and back. The duty cycle is updated every 10 ms.
 */
void loop(void) {
  // Duty cycle sweep variables 0 to 800 and vice versa
  uint32_t duty = 50;
  int8_t dir = 1;

  while (1) {
    duty += dir;
    if (duty >= 750) dir = -1;     // Max duty (100%)
    else if (duty <= 50) dir = 1;  // Min duty (0%)

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty));
    vTaskDelay(pdMS_TO_TICKS(10));  // Adjust delay to change sweep speed
  }
}
