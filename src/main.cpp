#include <WiFi.h>
#include <WebServer.h>
#include "driver/gpio.h"

/* ================= CONFIG ================= */

const char* AP_SSID = "ESP32_ROBOT";
const char* AP_PASS = "12345678";

WebServer server(80);

/* ================= PIN DEFINITIONS ================= */

// Stepper driver
constexpr gpio_num_t STEP_PIN = GPIO_NUM_18;
constexpr gpio_num_t DIR_PIN  = GPIO_NUM_19;
constexpr gpio_num_t MS1_PIN  = GPIO_NUM_21;
constexpr gpio_num_t MS2_PIN  = GPIO_NUM_22;
constexpr gpio_num_t MS3_PIN  = GPIO_NUM_23;

// Drive motors
constexpr int LPWM_PIN = 25;
constexpr int RPWM_PIN = 26;

constexpr gpio_num_t LDIR_PIN   = GPIO_NUM_27;
constexpr gpio_num_t RDIR_PIN   = GPIO_NUM_14;
constexpr gpio_num_t LBRAKE_PIN = GPIO_NUM_32;
constexpr gpio_num_t RBRAKE_PIN = GPIO_NUM_33;

// Main drive power MOSFET gate
constexpr gpio_num_t DRIVE_POWER_EN_PIN = GPIO_NUM_4;

/* ================= PWM CONFIG ================= */

constexpr uint8_t  LPWM_CH        = 0;
constexpr uint8_t  RPWM_CH        = 1;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;
constexpr uint8_t  MOTOR_PWM_RES  = 8;

/* ================= POLARITY CONFIG ================= */

constexpr bool LEFT_DIR_INVERT  = false;
constexpr bool RIGHT_DIR_INVERT = true;

constexpr bool DRIVE_POWER_ACTIVE_HIGH = true;

// Flipped from previous logic.
// false means LOW = brake active, HIGH = brake released.
constexpr bool BRAKE_ACTIVE_HIGH = false;

/* ================= STEPPER CONFIG ================= */

constexpr uint32_t STEPPER_PULSE_HZ = 200;

constexpr uint8_t STEP_MS1 = 0;
constexpr uint8_t STEP_MS2 = 0;
constexpr uint8_t STEP_MS3 = 0;

/* ================= DRIVE RAMP / SAFETY TIMING ================= */

constexpr uint32_t DRIVE_DIR_CHANGE_DELAY_MS = 150;
constexpr uint32_t DRIVE_RAMP_INTERVAL_MS    = 20;

// Softer ramp to reduce startup lunge
constexpr uint8_t DRIVE_ACCEL_STEP = 2;
constexpr uint8_t DRIVE_DECEL_STEP = 4;

constexpr uint32_t DRIVE_POWER_OFF_DELAY_MS = 150;
constexpr uint32_t WATCHDOG_TIMEOUT_MS      = 800;

/* ================= GLOBALS ================= */

portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;

enum DriveState
{
    DRIVE_IDLE = 0,
    DRIVE_FORWARD,
    DRIVE_BACKWARD,
    DRIVE_LEFT,
    DRIVE_RIGHT
};

enum StepperState
{
    STEPPER_IDLE = 0,
    STEPPER_CW,
    STEPPER_CCW
};

volatile DriveState   drive_state   = DRIVE_IDLE;
volatile StepperState stepper_state = STEPPER_IDLE;

volatile uint8_t drive_speed_percent = 20;
volatile uint8_t drive_duty = 20;

/* ================= DRIVE RAMP STATE ================= */

DriveState commanded_drive_state = DRIVE_IDLE;

uint8_t current_drive_duty = 0;
uint8_t target_drive_duty = 0;

uint32_t last_drive_ramp_ms = 0;

bool drive_interlock_active = false;
uint32_t drive_interlock_start_ms = 0;
DriveState pending_drive_state = DRIVE_IDLE;

bool drive_power_enabled = false;
bool drive_power_off_pending = false;
uint32_t drive_power_off_start_ms = 0;

/* ================= WATCHDOG STATE ================= */

volatile uint32_t lastPacketTime = 0;
bool watchdog_tripped = false;

/* ================= STEPPER NON-BLOCKING STATE ================= */

volatile bool stepper_enabled = false;
volatile bool stepper_dir_cw  = true;
volatile bool step_pin_state  = false;

uint32_t stepper_half_period_us = 2500;
uint32_t last_step_toggle_us = 0;

/* ================= LOW-LEVEL HELPERS ================= */

static inline void pin_write(gpio_num_t pin, bool value)
{
    gpio_set_level(pin, value ? 1 : 0);
}

static inline bool apply_dir_invert(bool dir_cmd, bool invert_flag)
{
    return invert_flag ? !dir_cmd : dir_cmd;
}

uint8_t speed_percent_to_duty(uint8_t percent)
{
    if (percent > 100) percent = 100;
    return (uint8_t)((255UL * percent) / 100UL);
}

void set_brake(gpio_num_t pin, bool brake_active)
{
    bool level = BRAKE_ACTIVE_HIGH ? brake_active : !brake_active;
    pin_write(pin, level);
}

void set_drive_power(bool enabled)
{
    bool level = DRIVE_POWER_ACTIVE_HIGH ? enabled : !enabled;
    pin_write(DRIVE_POWER_EN_PIN, level);

    drive_power_enabled = enabled;

    if (enabled)
    {
        drive_power_off_pending = false;
    }
}

void request_drive_power_off_delay()
{
    if (!drive_power_enabled)
        return;

    if (!drive_power_off_pending)
    {
        drive_power_off_pending = true;
        drive_power_off_start_ms = millis();
    }
}

void service_drive_power_off()
{
    if (!drive_power_off_pending)
        return;

    if (commanded_drive_state != DRIVE_IDLE || current_drive_duty != 0 || drive_interlock_active)
    {
        drive_power_off_pending = false;
        return;
    }

    uint32_t now_ms = millis();

    if ((uint32_t)(now_ms - drive_power_off_start_ms) >= DRIVE_POWER_OFF_DELAY_MS)
    {
        set_drive_power(false);
        drive_power_off_pending = false;
        Serial.println("DRIVE_POWER_OFF");
    }
}

/* ================= TELEMETRY HELPERS ================= */

const char* drive_state_to_str(DriveState s)
{
    switch (s)
    {
        case DRIVE_FORWARD:  return "FWD";
        case DRIVE_BACKWARD: return "BACK";
        case DRIVE_LEFT:     return "LEFT";
        case DRIVE_RIGHT:    return "RIGHT";
        default:             return "STOP";
    }
}

const char* stepper_state_to_str(StepperState s)
{
    switch (s)
    {
        case STEPPER_CW:   return "UP";
        case STEPPER_CCW:  return "DOWN";
        default:           return "STOP";
    }
}

String build_telemetry_string()
{
    String s;
    s.reserve(160);

    s += "DRIVE:";
    s += drive_state_to_str((DriveState)drive_state);

    s += "|CMD:";
    s += drive_state_to_str(commanded_drive_state);

    s += "|LIFT:";
    s += stepper_state_to_str((StepperState)stepper_state);

    s += "|SPD:";
    s += String(drive_speed_percent);

    s += "|DUTY:";
    s += String(current_drive_duty);

    s += "|PWR:";
    s += drive_power_enabled ? "ON" : "OFF";

    s += "|WDG_MS:";
    s += String((uint32_t)(millis() - lastPacketTime));

    return s;
}

/* ================= MOTOR CONTROL ================= */

void motor_set_left(uint8_t duty, bool dir, bool brake)
{
    dir = apply_dir_invert(dir, LEFT_DIR_INVERT);

    set_drive_power(true);
    pin_write(LDIR_PIN, dir);
    set_brake(LBRAKE_PIN, brake);
    ledcWrite(LPWM_CH, brake ? 0 : duty);
}

void motor_set_right(uint8_t duty, bool dir, bool brake)
{
    dir = apply_dir_invert(dir, RIGHT_DIR_INVERT);

    set_drive_power(true);
    pin_write(RDIR_PIN, dir);
    set_brake(RBRAKE_PIN, brake);
    ledcWrite(RPWM_CH, brake ? 0 : duty);
}

void drive_stop_raw()
{
    ledcWrite(LPWM_CH, 0);
    ledcWrite(RPWM_CH, 0);

    set_brake(LBRAKE_PIN, true);
    set_brake(RBRAKE_PIN, true);
}

void drive_forward_raw(uint8_t duty)
{
    motor_set_left(duty, true, false);
    motor_set_right(duty, true, false);
}

void drive_backward_raw(uint8_t duty)
{
    motor_set_left(duty, false, false);
    motor_set_right(duty, false, false);
}

void drive_left_raw(uint8_t duty)
{
    motor_set_left(duty, false, false);
    motor_set_right(duty, true, false);
}

void drive_right_raw(uint8_t duty)
{
    motor_set_left(duty, true, false);
    motor_set_right(duty, false, false);
}

void apply_drive_output(DriveState state, uint8_t duty)
{
    drive_state = state;

    if (state == DRIVE_IDLE || duty == 0)
    {
        drive_stop_raw();
        drive_state = DRIVE_IDLE;
        return;
    }

    set_drive_power(true);

    switch (state)
    {
        case DRIVE_FORWARD:
            drive_forward_raw(duty);
            break;

        case DRIVE_BACKWARD:
            drive_backward_raw(duty);
            break;

        case DRIVE_LEFT:
            drive_left_raw(duty);
            break;

        case DRIVE_RIGHT:
            drive_right_raw(duty);
            break;

        default:
            drive_stop_raw();
            drive_state = DRIVE_IDLE;
            break;
    }
}

bool is_opposite_drive_direction(DriveState old_state, DriveState new_state)
{
    return
        ((old_state == DRIVE_FORWARD)  && (new_state == DRIVE_BACKWARD)) ||
        ((old_state == DRIVE_BACKWARD) && (new_state == DRIVE_FORWARD));
}

void command_drive_state(DriveState new_state)
{
    if (new_state == DRIVE_IDLE)
    {
        commanded_drive_state = DRIVE_IDLE;
        target_drive_duty = 0;
        drive_interlock_active = false;

        Serial.println("DRIVE_COMMAND_STOP_DECEL");
        return;
    }

    set_drive_power(true);

    bool reversing =
        is_opposite_drive_direction(commanded_drive_state, new_state) ||
        is_opposite_drive_direction((DriveState)drive_state, new_state);

    if (reversing)
    {
        commanded_drive_state = DRIVE_IDLE;
        target_drive_duty = 0;

        pending_drive_state = new_state;
        drive_interlock_active = true;
        drive_interlock_start_ms = 0;

        Serial.println("DRIVE_REVERSAL_REQUEST_DECEL_FIRST");
        return;
    }

    commanded_drive_state = new_state;
    target_drive_duty = drive_duty;
    drive_power_off_pending = false;

    // Prevent initial lunge from stale duty.
    // Starting from stop must always ramp up from zero.
    if (drive_state == DRIVE_IDLE || current_drive_duty == 0)
    {
        current_drive_duty = 0;
        last_drive_ramp_ms = millis();
    }

    // Safety clamp: never start above the requested target.
    if (current_drive_duty > target_drive_duty)
    {
        current_drive_duty = target_drive_duty;
    }

    Serial.printf("DRIVE_COMMAND_%s target=%u\n", drive_state_to_str(new_state), target_drive_duty);
}

void service_drive_ramp()
{
    uint32_t now_ms = millis();

    if ((uint32_t)(now_ms - last_drive_ramp_ms) < DRIVE_RAMP_INTERVAL_MS)
        return;

    last_drive_ramp_ms = now_ms;

    if (current_drive_duty < target_drive_duty)
    {
        uint16_t next = current_drive_duty + DRIVE_ACCEL_STEP;
        current_drive_duty = (next > target_drive_duty) ? target_drive_duty : (uint8_t)next;
    }
    else if (current_drive_duty > target_drive_duty)
    {
        if (current_drive_duty > DRIVE_DECEL_STEP)
            current_drive_duty -= DRIVE_DECEL_STEP;
        else
            current_drive_duty = 0;

        if (current_drive_duty < target_drive_duty)
            current_drive_duty = target_drive_duty;
    }

    if (current_drive_duty == 0)
    {
        apply_drive_output(DRIVE_IDLE, 0);

        if (drive_interlock_active)
        {
            if (drive_interlock_start_ms == 0)
            {
                drive_interlock_start_ms = now_ms;
                Serial.printf("DRIVE_BRAKE_HOLD_%luMS\n", (unsigned long)DRIVE_DIR_CHANGE_DELAY_MS);
            }

            if ((uint32_t)(now_ms - drive_interlock_start_ms) >= DRIVE_DIR_CHANGE_DELAY_MS)
            {
                drive_interlock_active = false;
                commanded_drive_state = pending_drive_state;
                target_drive_duty = drive_duty;
                drive_power_off_pending = false;

                current_drive_duty = 0;
                last_drive_ramp_ms = millis();

                set_drive_power(true);

                Serial.printf("DRIVE_REVERSAL_APPLY_%s\n", drive_state_to_str(pending_drive_state));
            }
        }
        else if (commanded_drive_state == DRIVE_IDLE)
        {
            request_drive_power_off_delay();
        }

        return;
    }

    apply_drive_output(commanded_drive_state, current_drive_duty);
}

void update_drive_speed(uint8_t received_percent)
{
    if (received_percent > 100) received_percent = 100;

    // Invert Python speed.
    // Example: Python sends 87, ESP32 uses 13.
    uint8_t effective_percent = 100 - received_percent;

    drive_speed_percent = effective_percent;
    drive_duty = speed_percent_to_duty(effective_percent);

    if (commanded_drive_state != DRIVE_IDLE && !drive_interlock_active)
    {
        target_drive_duty = drive_duty;

        if (current_drive_duty > target_drive_duty)
        {
            current_drive_duty = target_drive_duty;
        }
    }

    Serial.printf(
        "SPEED_RX=%u -> EFFECTIVE=%u%% (target duty=%u)\n",
        received_percent,
        drive_speed_percent,
        drive_duty
    );
}

/* ================= STEPPER CONTROL ================= */

void stepper_set_dir(bool cw)
{
    stepper_dir_cw = cw;
    pin_write(DIR_PIN, cw ? 1 : 0);
}

void stepper_enable(bool enable)
{
    portENTER_CRITICAL(&stepperMux);
    stepper_enabled = enable;

    if (!enable)
    {
        step_pin_state = false;
        pin_write(STEP_PIN, 0);
    }
    else
    {
        last_step_toggle_us = micros();
    }

    portEXIT_CRITICAL(&stepperMux);
}

void stepper_set_microstep(uint8_t ms1, uint8_t ms2, uint8_t ms3)
{
    pin_write(MS1_PIN, ms1);
    pin_write(MS2_PIN, ms2);
    pin_write(MS3_PIN, ms3);
}

void stepper_set_rate_hz(uint32_t pulse_hz)
{
    if (pulse_hz < 1) pulse_hz = 1;

    stepper_half_period_us = 1000000UL / (pulse_hz * 2UL);

    if (stepper_half_period_us < 2)
        stepper_half_period_us = 2;
}

void stepper_stop_immediate()
{
    stepper_enable(false);
    stepper_state = STEPPER_IDLE;
    Serial.println("LIFT_STOP");
}

void apply_stepper_state(StepperState new_state)
{
    if (new_state == STEPPER_IDLE)
    {
        stepper_stop_immediate();
        return;
    }

    stepper_state = new_state;

    switch (new_state)
    {
        case STEPPER_CW:
            stepper_set_dir(true);
            stepper_enable(true);
            Serial.println("LIFT_UP");
            break;

        case STEPPER_CCW:
            stepper_set_dir(false);
            stepper_enable(true);
            Serial.println("LIFT_DOWN");
            break;

        default:
            stepper_stop_immediate();
            break;
    }
}

void service_stepper()
{
    if (!stepper_enabled)
        return;

    uint32_t now_us = micros();

    if ((uint32_t)(now_us - last_step_toggle_us) >= stepper_half_period_us)
    {
        portENTER_CRITICAL(&stepperMux);
        step_pin_state = !step_pin_state;
        pin_write(STEP_PIN, step_pin_state);
        last_step_toggle_us = now_us;
        portEXIT_CRITICAL(&stepperMux);
    }
}

/* ================= SAFETY ================= */

void refresh_watchdog()
{
    lastPacketTime = millis();
    watchdog_tripped = false;
}

void stopAllMotion()
{
    drive_interlock_active = false;
    commanded_drive_state = DRIVE_IDLE;
    pending_drive_state = DRIVE_IDLE;
    target_drive_duty = 0;
    current_drive_duty = 0;

    apply_drive_output(DRIVE_IDLE, 0);
    request_drive_power_off_delay();

    apply_stepper_state(STEPPER_IDLE);
}

/* ================= COMMAND HANDLING ================= */

bool is_valid_command(char c)
{
    return (
        c == 'W' || c == 'A' || c == 'S' || c == 'D' ||
        c == 'U' || c == 'J' || c == 'K' || c == 'X'
    );
}

void apply_command(char cmd)
{
    Serial.print("RX TOKEN: ");
    Serial.println(cmd);

    refresh_watchdog();

    switch (cmd)
    {
        case 'W':
            command_drive_state(DRIVE_FORWARD);
            break;

        case 'S':
            command_drive_state(DRIVE_BACKWARD);
            break;

        case 'A':
            command_drive_state(DRIVE_LEFT);
            break;

        case 'D':
            command_drive_state(DRIVE_RIGHT);
            break;

        case 'U':
            apply_stepper_state(STEPPER_CW);
            break;

        case 'J':
            apply_stepper_state(STEPPER_CCW);
            break;

        case 'K':
            apply_stepper_state(STEPPER_IDLE);
            break;

        case 'X':
            command_drive_state(DRIVE_IDLE);
            apply_stepper_state(STEPPER_IDLE);
            Serial.println("STOP_ALL_DECEL");
            break;

        default:
            Serial.println("UNKNOWN TOKEN");
            break;
    }
}

/* ================= HTTP HANDLERS ================= */

void handle_ping()
{
    server.send(200, "text/plain", build_telemetry_string());
}

void handle_cmd()
{
    if (!server.hasArg("plain"))
    {
        server.send(400, "text/plain", "ERROR:NO_BODY");
        return;
    }

    String body = server.arg("plain");
    body.trim();

    if (body.length() != 1)
    {
        server.send(400, "text/plain", "ERROR:INVALID_LEN");
        return;
    }

    char cmd = body.charAt(0);

    if (!is_valid_command(cmd))
    {
        server.send(400, "text/plain", "ERROR:INVALID_CMD");
        return;
    }

    apply_command(cmd);
    server.send(200, "text/plain", build_telemetry_string());
}

void handle_speed()
{
    if (!server.hasArg("plain"))
    {
        server.send(400, "text/plain", "ERROR:NO_BODY");
        return;
    }

    String body = server.arg("plain");
    body.trim();

    int percent = body.toInt();

    if (percent < 0 || percent > 100)
    {
        server.send(400, "text/plain", "ERROR:INVALID_SPEED");
        return;
    }

    update_drive_speed((uint8_t)percent);
    refresh_watchdog();

    server.send(200, "text/plain", build_telemetry_string());
}

void handle_heartbeat()
{
    if (!server.hasArg("plain"))
    {
        server.send(400, "text/plain", "ERROR:NO_BODY");
        return;
    }

    String body = server.arg("plain");
    body.trim();
    body.toUpperCase();

    if (body != "ALIVE")
    {
        server.send(400, "text/plain", "ERROR:INVALID_HEARTBEAT");
        return;
    }

    refresh_watchdog();
    server.send(200, "text/plain", build_telemetry_string());
}

void handle_status()
{
    server.send(200, "text/plain", build_telemetry_string());
}

void handle_not_found()
{
    server.send(404, "text/plain", "ERROR:NOT_FOUND");
}

/* ================= INIT ================= */

void gpio_init_all()
{
    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(MS1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MS2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MS3_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(LDIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RDIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LBRAKE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RBRAKE_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(DRIVE_POWER_EN_PIN, GPIO_MODE_OUTPUT);

    pin_write(STEP_PIN, 0);
    pin_write(DIR_PIN, 0);

    stepper_set_microstep(STEP_MS1, STEP_MS2, STEP_MS3);

    set_drive_power(false);
    drive_stop_raw();
    stepper_enable(false);
}

void pwm_init()
{
    ledcSetup(LPWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(LPWM_PIN, LPWM_CH);
    ledcWrite(LPWM_CH, 0);

    ledcSetup(RPWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(RPWM_PIN, RPWM_CH);
    ledcWrite(RPWM_CH, 0);
}

void wifi_init()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);

    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
}

void server_init()
{
    server.on("/api/ping",      HTTP_GET,  handle_ping);
    server.on("/api/cmd",       HTTP_POST, handle_cmd);
    server.on("/api/speed",     HTTP_POST, handle_speed);
    server.on("/api/heartbeat", HTTP_POST, handle_heartbeat);
    server.on("/api/status",    HTTP_GET,  handle_status);
    server.onNotFound(handle_not_found);

    server.begin();
    Serial.println("HTTP server started on port 80");
}

/* ================= SETUP ================= */

void setup()
{
    Serial.begin(115200);

    gpio_init_all();
    pwm_init();
    wifi_init();
    server_init();

    drive_duty = speed_percent_to_duty(drive_speed_percent);
    stepper_set_rate_hz(STEPPER_PULSE_HZ);

    commanded_drive_state = DRIVE_IDLE;
    drive_state = DRIVE_IDLE;
    current_drive_duty = 0;
    target_drive_duty = 0;

    apply_drive_output(DRIVE_IDLE, 0);
    set_drive_power(false);
    apply_stepper_state(STEPPER_IDLE);

    lastPacketTime = millis();
    watchdog_tripped = false;

    Serial.println("Robot ready");
}

/* ================= LOOP ================= */

void loop()
{
    server.handleClient();

    service_stepper();
    service_drive_ramp();
    service_drive_power_off();

    uint32_t now_ms = millis();

    if (!watchdog_tripped && ((uint32_t)(now_ms - lastPacketTime) > WATCHDOG_TIMEOUT_MS))
    {
        stopAllMotion();
        watchdog_tripped = true;
        Serial.println("[WATCHDOG] timeout -> STOP_ALL");
    }
}