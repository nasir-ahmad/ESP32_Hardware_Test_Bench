/**
 * @brief Example code for generating SPWM signals using the MCPWM peripheral on ESP32.
 * 
 * This example demonstrates how to set up two PWM signals with dead time.
 * The code uses the MCPWM (Motor Control Pulse Width Modulation) module to generate PWM signals
 * on two GPIO pins.
 * 
 * 
 * Author: Nasir Ahmad
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"

// Define GPIOs for PWM outputs
#define GPIO_PWM0A 12  // SPWM GPIO_A
#define GPIO_PWM0B 13  // SPWM GPIO_B
#define TIMER_RES_HZ (80 * 1000 * 1000)
#define SPWM_CARRIER_FREQ_HZ 20000.0                                //SPWM carrier 20Khz
#define PERIOD_TICK (TIMER_RES_HZ / SPWM_CARRIER_FREQ_HZ)           //20 kHz period (50 us / 12.5 ns = 4000 ticks at 100% duty cycle)
#define SINEWAVE_FREQ_HZ 50.0                                       //AC sinewave 50Hz
#define SPWM_TOTAL_PULSE (SPWM_CARRIER_FREQ_HZ / SINEWAVE_FREQ_HZ)  //Number of spwm pulse in a sinewave cycle (20Khz/50 = 400)
#define SINEWAVE_AMPLITUDE 3800                                     //maximum allowed comparator value for timer i.e. amplitude of the sine wave


// Global handle for the MCPWM comparator
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_gen_handle_t gen_a = NULL, gen_b = NULL;

volatile int spwm_pulse_index = 0;

// Timer event callback function
static bool IRAM_ATTR timer_on_empty(
  mcpwm_timer_handle_t timer,
  const mcpwm_timer_event_data_t *event_data,
  void *user_data) {

  float spwm_cmp_value_raw = SINEWAVE_AMPLITUDE * sin(2 * PI * SINEWAVE_FREQ_HZ * (spwm_pulse_index / SPWM_CARRIER_FREQ_HZ));
  int spwm_cmp_value = int(spwm_cmp_value_raw + 0.5);

  if (spwm_cmp_value > 0) {  // Positive half cycle of sine wave
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, spwm_cmp_value));
    //digitalWrite(LO2, 1);
  }
  if (spwm_cmp_value < 0) {  // Negative half cycle of sine wave
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, PERIOD_TICK + spwm_cmp_value));
    //digitalWrite(HO2, 1);
  }

  // Zero cross (0 deg)
  if (spwm_pulse_index == 0) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));
    //digitalWrite(HO2, 0);
  }

  // Zero Cross (180 deg)
  if (spwm_pulse_index == (SPWM_TOTAL_PULSE / 2)) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, PERIOD_TICK));
    //digitalWrite(LO2, 0);
  }

  spwm_pulse_index++;
  if (spwm_pulse_index > SPWM_TOTAL_PULSE) {
    spwm_pulse_index = 0;  // reset i if > 1 cycle
  }

  return pdFALSE;  // Return whether a task was woken
}


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
    .resolution_hz = TIMER_RES_HZ,  // 80 MHz
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = PERIOD_TICK,  // 20 kHz period (50 us / 12.5 ns = 4000 ticks)
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  // Create operator
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

  // Define the event callbacks
  mcpwm_timer_event_callbacks_t cbs = {
    .on_full = NULL,             // Called at peak (UP direction)
    .on_empty = timer_on_empty,  // Called at trough (DOWN direction)
    .on_stop = NULL,             // Optional: Timer stop event
  };

  // Register the callbacks with the timer
  ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer, &cbs, NULL));

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


  Serial.begin(115200);
  pinMode(14, OUTPUT);
}

/**
 * @brief Main loop for sweeping the PWM duty cycle.
 * 
 * This function continuously adjusts the duty cycle of the PWM signals,
 * sweeping from 0% to 100% and back. The duty cycle is updated every 10 ms.
 */
void loop(void) {
  /*
  // Duty cycle sweep variables 0 to 800 and vice versa
  uint32_t duty = 0;
  int8_t dir = 1;

  while (1) {
    duty += dir;
    if (duty >= PERIOD_TICK) dir = -1;  // Max duty (100%)
    else if (duty <= 0) dir = 1;        // Min duty (0%)

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty));
    vTaskDelay(pdMS_TO_TICKS(2));  // Adjust delay to change sweep speed
  }
  */
}
