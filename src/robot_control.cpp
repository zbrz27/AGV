#include "robot_config.h"

/* Internal Logic Constants */
constexpr uint8_t DRIVE_ACCEL_STEP = 2;
constexpr uint8_t DRIVE_DECEL_STEP = 4;
constexpr uint32_t DRIVE_DIR_CHANGE_DELAY_MS = 150;
constexpr uint32_t DRIVE_POWER_OFF_DELAY_MS = 150;

/* Globals */
portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;
volatile DriveState drive_state = DRIVE_IDLE;
volatile StepperState stepper_state = STEPPER_IDLE;
DriveState commanded_drive_state = DRIVE_IDLE;
DriveState pending_drive_state = DRIVE_IDLE;

extern uint8_t current_drive_duty = 0;
extern uint8_t target_drive_duty = 0;
extern uint8_t drive_duty_pref = 20; 

extern uint32_t last_drive_ramp_ms = 0;
extern uint32_t last_step_toggle_us = 0;
extern uint32_t lastPacketTime = 0;
extern uint32_t drive_interlock_start_ms = 0;
extern uint32_t drive_power_off_start_ms = 0;
extern uint32_t stepper_half_period_us = 2500;

extern bool watchdog_tripped = false;
extern bool drive_interlock_active = false;
extern bool drive_power_enabled = false;
extern bool drive_power_off_pending = false;
volatile bool stepper_enabled = false;
volatile bool step_pin_state = false;

/* Helper functions (Internal only) */
static inline void pin_write(gpio_num_t pin, bool value) { gpio_set_level(pin, value ? 1 : 0); }

void set_drive_power(bool enabled) {
    pin_write(DRIVE_POWER_EN_PIN, enabled);
    drive_power_enabled = enabled;
    if (enabled) drive_power_off_pending = false;
}

void apply_drive_output(DriveState state, uint8_t duty) {
    drive_state = state;
    if (state == DRIVE_IDLE || duty == 0) {
        ledcWrite(LPWM_CH, 0); ledcWrite(RPWM_CH, 0);
        pin_write(LBRAKE_PIN, true); pin_write(RBRAKE_PIN, true);
        return;
    }
    set_drive_power(true);
    bool ldir = (state == DRIVE_FORWARD || state == DRIVE_RIGHT);
    bool rdir = (state == DRIVE_FORWARD || state == DRIVE_LEFT); // Note: Right is inverted in original logic
    
    pin_write(LDIR_PIN, ldir);
    pin_write(RDIR_PIN, !rdir); // Assuming RIGHT_DIR_INVERT = true
    pin_write(LBRAKE_PIN, false);
    pin_write(RBRAKE_PIN, false);
    ledcWrite(LPWM_CH, duty);
    ledcWrite(RPWM_CH, duty);
}

/* Public API Functions */
void robot_init() {
    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
    // ... Initialize other GPIOs similarly ...
    ledcSetup(LPWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(LPWM_PIN, LPWM_CH);
    ledcSetup(RPWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(RPWM_PIN, RPWM_CH);
    
    lastPacketTime = millis();
}

void service_robot() {
    uint32_t now_ms = millis();
    
    // Ramp Logic
    if ((now_ms - last_drive_ramp_ms) >= DRIVE_RAMP_INTERVAL_MS) {
        if (current_drive_duty < target_drive_duty) current_drive_duty += DRIVE_ACCEL_STEP;
        else if (current_drive_duty > target_drive_duty) current_drive_duty -= DRIVE_DECEL_STEP;
        last_drive_ramp_ms = now_ms;
        apply_drive_output(commanded_drive_state, current_drive_duty);
    }

    // Watchdog
    if (!watchdog_tripped && (now_ms - lastPacketTime > WATCHDOG_TIMEOUT_MS)) {
        stopAllMotion();
        watchdog_tripped = true;
    }
}

void stopAllMotion() {
    target_drive_duty = 0;
    current_drive_duty = 0;
    apply_drive_output(DRIVE_IDLE, 0);
    stepper_enabled = false;
}

void refresh_watchdog() { lastPacketTime = millis(); watchdog_tripped = false; }
bool is_watchdog_tripped() { return watchdog_tripped; }

void update_drive_speed(uint8_t received_percent) {
    uint8_t effective = 100 - received_percent;
    target_drive_duty = (uint8_t)((255UL * effective) / 100UL);
}

void apply_command(char cmd) {
    refresh_watchdog();
    switch (cmd) {
        case 'W': commanded_drive_state = DRIVE_FORWARD; break;
        case 'S': commanded_drive_state = DRIVE_BACKWARD; break;
        case 'X': stopAllMotion(); break;
        // Add A, D, U, J, K cases as per original
    }
}

String build_telemetry_string() {
    return "PWR:" + String(drive_power_enabled ? "ON" : "OFF") + "|DUTY:" + String(current_drive_duty);
}