#include <WiFi.h>
#include <WebServer.h>
#include "driver/gpio.h"

/* ================= CONFIG ================= */

// SoftAP config
const char* AP_SSID = "ESP32_ROBOT";
const char* AP_PASS = "12345678";

/* ================= HTTP SERVER ================= */

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

/* ================= PWM CONFIG ================= */

constexpr uint8_t  LPWM_CH        = 0;
constexpr uint8_t  RPWM_CH        = 1;
constexpr uint32_t MOTOR_PWM_FREQ = 20000;
constexpr uint8_t  MOTOR_PWM_RES  = 8;

/* ================= MOTOR POLARITY CONFIG ================= */

// If one side spins the wrong way during FORWARD, flip that side's flag.
constexpr bool LEFT_DIR_INVERT  = false;
constexpr bool RIGHT_DIR_INVERT = true;

/* ================= STEPPER CONFIG ================= */

constexpr uint32_t STEPPER_PULSE_HZ = 200;

// Microstep pins
constexpr uint8_t STEP_MS1 = 0;
constexpr uint8_t STEP_MS2 = 0;
constexpr uint8_t STEP_MS3 = 0;

/* ================= SAFETY / TIMING ================= */

constexpr uint32_t DRIVE_DIR_CHANGE_DELAY_MS = 50;
constexpr uint32_t WATCHDOG_TIMEOUT_MS       = 800;

/* ================= GLOBALS ================= */

portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;

/* ================= STATE ENUMS ================= */

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

/* ================= RUNTIME STATE ================= */

volatile DriveState   drive_state   = DRIVE_IDLE;
volatile StepperState stepper_state = STEPPER_IDLE;

volatile uint8_t drive_speed_percent = 50;
volatile uint8_t drive_duty = 127;

/* ================= WATCHDOG STATE ================= */

volatile uint32_t lastPacketTime = 0;
bool watchdog_tripped = false;

/* ================= DRIVE INTERLOCK STATE ================= */

bool drive_interlock_active = false;
uint32_t drive_interlock_start_ms = 0;
DriveState pending_drive_state = DRIVE_IDLE;

/* ================= STEPPER NON-BLOCKING STATE ================= */

volatile bool stepper_enabled = false;
volatile bool stepper_dir_cw  = true;
volatile bool step_pin_state  = false;

uint32_t stepper_half_period_us = 2500;   // 200 Hz -> 2500 us half-period
uint32_t last_step_toggle_us = 0;

/* ================= LOW-LEVEL GPIO HELPERS ================= */

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

/* ================= TELEMETRY HELPERS ================= */

const char* drive_state_to_str(DriveState s)
{
    switch (s)
    {
        case DRIVE_FORWARD:  return "FWD";
        case DRIVE_BACKWARD: return "BACK";
        case DRIVE_LEFT:     return "LEFT";
        case DRIVE_RIGHT:    return "RIGHT";
        case DRIVE_IDLE:
        default:             return "STOP";
    }
}

const char* stepper_state_to_str(StepperState s)
{
    switch (s)
    {
        case STEPPER_CW:   return "UP";
        case STEPPER_CCW:  return "DOWN";
        case STEPPER_IDLE:
        default:           return "STOP";
    }
}

String build_telemetry_string()
{
    String s;
    s.reserve(80);

    s += "DRIVE:";
    s += drive_state_to_str((DriveState)drive_state);

    s += "|LIFT:";
    s += stepper_state_to_str((StepperState)stepper_state);

    s += "|SPD:";
    s += String(drive_speed_percent);

    s += "|WDG_MS:";
    s += String((uint32_t)(millis() - lastPacketTime));

    return s;
}

/* ================= MOTOR CONTROL ================= */

void motor_set_left(uint8_t duty, bool dir, bool brake)
{
    dir = apply_dir_invert(dir, LEFT_DIR_INVERT);
    pin_write(LDIR_PIN, dir);
    pin_write(LBRAKE_PIN, brake);
    ledcWrite(LPWM_CH, brake ? 0 : duty);
}

void motor_set_right(uint8_t duty, bool dir, bool brake)
{
    dir = apply_dir_invert(dir, RIGHT_DIR_INVERT);
    pin_write(RDIR_PIN, dir);
    pin_write(RBRAKE_PIN, brake);
    ledcWrite(RPWM_CH, brake ? 0 : duty);
}

void drive_stop_raw()
{
    // Do NOT rewrite direction pins during stop.
    // Only brake and remove PWM so the analyzer does not see a fake direction snap.
    pin_write(LBRAKE_PIN, true);
    pin_write(RBRAKE_PIN, true);

    ledcWrite(LPWM_CH, 0);
    ledcWrite(RPWM_CH, 0);
}

void drive_forward_raw()
{
    motor_set_left(drive_duty, true, false);
    motor_set_right(drive_duty, true, false);
}

void drive_backward_raw()
{
    motor_set_left(drive_duty, false, false);
    motor_set_right(drive_duty, false, false);
}

void drive_left_raw()
{
    motor_set_left(drive_duty, false, false);
    motor_set_right(drive_duty, true, false);
}

void drive_right_raw()
{
    motor_set_left(drive_duty, true, false);
    motor_set_right(drive_duty, false, false);
}

void apply_drive_state_immediate(DriveState new_state)
{
    drive_state = new_state;

    switch (new_state)
    {
        case DRIVE_IDLE:
            drive_stop_raw();
            Serial.println("DRIVE_IDLE");
            break;

        case DRIVE_FORWARD:
            drive_forward_raw();
            Serial.printf("DRIVE_FORWARD @ %u%%\n", drive_speed_percent);
            break;

        case DRIVE_BACKWARD:
            drive_backward_raw();
            Serial.printf("DRIVE_BACKWARD @ %u%%\n", drive_speed_percent);
            break;

        case DRIVE_LEFT:
            drive_left_raw();
            Serial.printf("DRIVE_LEFT @ %u%%\n", drive_speed_percent);
            break;

        case DRIVE_RIGHT:
            drive_right_raw();
            Serial.printf("DRIVE_RIGHT @ %u%%\n", drive_speed_percent);
            break;

        default:
            drive_stop_raw();
            drive_state = DRIVE_IDLE;
            Serial.println("DRIVE_DEFAULT_TO_IDLE");
            break;
    }
}

void apply_drive_state(DriveState new_state)
{
    bool reversing =
        ((drive_state == DRIVE_FORWARD)  && (new_state == DRIVE_BACKWARD)) ||
        ((drive_state == DRIVE_BACKWARD) && (new_state == DRIVE_FORWARD));

    if (reversing)
    {
        drive_stop_raw();
        drive_state = DRIVE_IDLE;

        drive_interlock_active = true;
        drive_interlock_start_ms = millis();
        pending_drive_state = new_state;

        Serial.println("DRIVE_INTERLOCK_50MS");
        return;
    }

    apply_drive_state_immediate(new_state);
}

void update_drive_speed(uint8_t percent)
{
    if (percent > 100) percent = 100;

    drive_speed_percent = percent;
    drive_duty = speed_percent_to_duty(percent);

    Serial.printf("SPEED_UPDATED -> %u%% (duty=%u)\n", drive_speed_percent, drive_duty);

    if (!drive_interlock_active)
    {
        apply_drive_state_immediate((DriveState)drive_state);
    }
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
    apply_drive_state_immediate(DRIVE_IDLE);
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
            apply_drive_state(DRIVE_FORWARD);
            break;

        case 'S':
            apply_drive_state(DRIVE_BACKWARD);
            break;

        case 'A':
            apply_drive_state(DRIVE_LEFT);
            break;

        case 'D':
            apply_drive_state(DRIVE_RIGHT);
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
            stopAllMotion();
            Serial.println("STOP_ALL");
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

    pin_write(STEP_PIN, 0);
    pin_write(DIR_PIN, 0);

    stepper_set_microstep(STEP_MS1, STEP_MS2, STEP_MS3);

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
    server.on("/api/ping",   HTTP_GET,  handle_ping);
    server.on("/api/cmd",    HTTP_POST, handle_cmd);
    server.on("/api/speed",  HTTP_POST, handle_speed);
    server.on("/api/status", HTTP_GET,  handle_status);
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

    apply_drive_state_immediate(DRIVE_IDLE);
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

    if (drive_interlock_active)
    {
        uint32_t now_ms = millis();

        if ((uint32_t)(now_ms - drive_interlock_start_ms) >= DRIVE_DIR_CHANGE_DELAY_MS)
        {
            drive_interlock_active = false;
            apply_drive_state_immediate(pending_drive_state);
        }
    }

    uint32_t now_ms = millis();

    if (!watchdog_tripped && ((uint32_t)(now_ms - lastPacketTime) > WATCHDOG_TIMEOUT_MS))
    {
        stopAllMotion();
        watchdog_tripped = true;
        Serial.println("[WATCHDOG] timeout -> STOP_ALL");
    }
}