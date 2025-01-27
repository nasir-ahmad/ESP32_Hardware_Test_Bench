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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"

// Define GPIOs for PWM outputs
#define GPIO_PWM0A 12  // Replace with your GPIO for PWM output A
#define GPIO_PWM0B 13  // Replace with your GPIO for PWM output B

// Global handle for the MCPWM comparator
static mcpwm_cmpr_handle_t comparator0 = NULL;
static mcpwm_cmpr_handle_t comparator1 = NULL;

/**
 * @brief Setup function for initializing MCPWM components.
 * 
 * This function configures the MCPWM timer, operator, comparator, and generators.
 * It also sets up the dead time for the PWM signals and starts the timer.
 */
void setup() {
  // Configure MCPWM timer
  mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = 80 * 1000 * 1000,  // 80 MHz
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = 800,  // 100 kHz period (10 us / 12.5 ns = 800 ticks)
  };
  mcpwm_timer_handle_t timer0 = NULL;
  mcpwm_timer_handle_t timer1 = NULL;

  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer0));
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer1));


  //Create timer sync with TEZ
  mcpwm_sync_handle_t timer_sync_source = NULL;
  mcpwm_timer_sync_src_config_t timer_sync_config = {
    .timer_event = MCPWM_TIMER_EVENT_EMPTY,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer_sync_src(timer0, &timer_sync_config, &timer_sync_source));


  //Timer sync phase
  mcpwm_timer_sync_phase_config_t sync_phase_config = {
    .sync_src = timer_sync_source,
    .count_value = 400,
    .direction = MCPWM_TIMER_DIRECTION_UP,
  };
  ESP_ERROR_CHECK(mcpwm_timer_set_phase_on_sync(timer1, &sync_phase_config));


  // Create operator
  mcpwm_oper_handle_t oper0 = NULL;
  mcpwm_oper_handle_t oper1 = NULL;
  mcpwm_operator_config_t oper_config = { .group_id = 0 };
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper0));
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &oper1));

  // Connect timer to operator
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper0, timer0));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, timer1));

  // Create comparator
  mcpwm_comparator_config_t comparator_config = {};
  comparator_config.flags.update_cmp_on_tez = true;
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper0, &comparator_config, &comparator0));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator0, 0));

  ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &comparator_config, &comparator1));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, 0));


  // Create generators for PWM A and B
  mcpwm_gen_handle_t gen_a = NULL, gen_b = NULL;
  mcpwm_generator_config_t gen_config = { .gen_gpio_num = GPIO_PWM0A };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper0, &gen_config, &gen_a));
  gen_config.gen_gpio_num = GPIO_PWM0B;
  ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &gen_config, &gen_b));


  // Configure generator actions
  // PWM A: High on timer start, Low on compare
  // Both generators start with same phase - dead time will invert one
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_a,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_a,
                                                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator0, MCPWM_GEN_ACTION_LOW)));

  // Configure gen_b with SAME actions initially
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_b,
                                                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_b,
                                                              MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator1, MCPWM_GEN_ACTION_HIGH)));


  // Configure dead time (500 ns = 40 ticks @ 80 MHz)
  mcpwm_dead_time_config_t dead_time_config_a = {
    .posedge_delay_ticks = 30,
    //.negedge_delay_ticks = 30,
  };
  ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_a, gen_a, &dead_time_config_a));

  mcpwm_dead_time_config_t dead_time_config_b = {
    //.posedge_delay_ticks = 30,
    .negedge_delay_ticks = 30,
  };
  dead_time_config_b.flags.invert_output = true;
  ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_b, gen_b, &dead_time_config_b));


  // Start timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer0));
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer1));

  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP));
}

/**
 * @brief Main loop for sweeping the PWM duty cycle.
 * 
 * This function continuously adjusts the duty cycle of the PWM signals,
 * sweeping from 0% to 100% and back. The duty cycle is updated every 10 ms.
 */
void loop(void) {
  // Duty cycle sweep variables 0 to 800 and vice versa
  int duty = 0;
  int dir = 1;

  while (1) {
    duty += dir;
    if (duty >= 400) dir = -1;    // Max duty (100%)
    else if (duty <= 0) dir = 1;  // Min duty (0%)

    int dutyCapped = min(400, max(20, duty));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator0, dutyCapped));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, dutyCapped));
    vTaskDelay(pdMS_TO_TICKS(10));  // Adjust delay to change sweep speed
  }
}
