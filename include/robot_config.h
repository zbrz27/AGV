#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include <WebServer.h>
#include "driver/gpio.h"

/* ================= CONFIG ================= */
extern const char* AP_SSID;
extern const char* AP_PASS;

/* ================= PIN DEFINITIONS ================= */
constexpr gpio_num_t STEP_PIN = GPIO_NUM_18;
constexpr gpio_num_t DIR_PIN  = GPIO_NUM_19;
constexpr gpio_num_t MS1_PIN  = GPIO_NUM_21;
constexpr gpio_num_t MS2_PIN  = GPIO_NUM_22;
constexpr gpio_num_t MS3_PIN  = GPIO_NUM_23;

constexpr int LPWM_PIN = 25;
constexpr int RPWM_PIN = 26;
constexpr gpio_num_t LDIR_PIN   = GPIO_NUM_27;
constexpr gpio_num_t RDIR_PIN   = GPIO_NUM_14;
constexpr gpio_num_t LBRAKE_PIN = GPIO_NUM_32;
constexpr gpio_num_t RBRAKE_PIN = GPIO_NUM_33;
constexpr gpio_num_t DRIVE_POWER_EN_PIN = GPIO_NUM_4;

/* ================= PWM & TIMING ================= */
constexpr uint8_t  LPWM_CH        = 0;
constexpr uint8_t  RPWM_CH        = 1;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;
constexpr uint8_t  MOTOR_PWM_RES  = 8;

constexpr uint32_t WATCHDOG_TIMEOUT_MS = 800;
constexpr uint32_t DRIVE_RAMP_INTERVAL_MS = 20;

/* ================= ENUMS & STATES ================= */
enum DriveState { DRIVE_IDLE = 0, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_LEFT, DRIVE_RIGHT };
enum StepperState { STEPPER_IDLE = 0, STEPPER_CW, STEPPER_CCW };

/* ================= FUNCTION PROTOTYPES ================= */
void robot_init();
void service_robot();
void stopAllMotion();
void apply_command(char cmd);
void update_drive_speed(uint8_t received_percent);
void refresh_watchdog();
bool is_watchdog_tripped();
String build_telemetry_string();

#endif