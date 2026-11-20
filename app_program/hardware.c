#include "hardware.h"
#include "car_config.h"

#define REG(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define IO_BASE     0x40014000u
#define TIM_BASE    0x40054000u
#define PWM_ADDR    0x40050000u
#define IO_IRQ      13u
#define PULSE_IRQ   0u
#define CAR_GPIO_CTRL(pin) (IO_BASE + 8u * (pin) + 4u)
#define OUT_FORCE_LOW (2u << 8)
#define EDGE_FALL   4u
#define EDGE_RISE   8u
#define PWM_TOP     999
#define ECHO_MAX_US 29000u

#define EDGE_LIMIT    100u
#define EDGE_OVERFLOW (1u << 14)
#define WDOG_BASE     0x40058000u
#define WDOG_LOAD     400000u /* RP2040 decrements twice per microsecond. */

static volatile uint16_t encoder_word;
static ID echo_events;
static bool brake_latched = true;
static volatile struct {
    uint32_t armed_at;
    uint32_t rise_at;
    uint16_t serial;
    uint16_t width;
    bool armed : 1;
    bool rising : 1;
} echo;

uint32_t app_us(void)
{
    return REG(TIM_BASE + 0x28u);
}

static void pwm_pair(unsigned pin, unsigned a, unsigned b)
{
    unsigned slice = (pin >> 1) & 7u;
    REG(PWM_ADDR + 0x14u * slice + 0x0cu) = a | (b << 16);
}

void hardware_brake(bool engaged)
{
    static const unsigned motor_pins[] = {
        PIN_MOTOR_LEFT_A, PIN_MOTOR_LEFT_B, PIN_MOTOR_RIGHT_A, PIN_MOTOR_RIGHT_B
    };
    UW key;

    DI(key);
    brake_latched = engaged || (car_faults & FATAL_FAULTS) != 0u
                 || hardware_estop_held();
    if (brake_latched) {
        pwm_pair(PIN_MOTOR_LEFT_A, 0u, 0u);
        pwm_pair(PIN_MOTOR_RIGHT_A, 0u, 0u);
    }
    for (unsigned i = 0; i < 4u; ++i) {
        /* RP2040 atomic register alias, leaving the PWM mux untouched. */
        REG(CAR_GPIO_CTRL(motor_pins[i]) + (brake_latched ? 0x2000u : 0x3000u))
            = OUT_FORCE_LOW;
    }
    EI(key);
}

static void motor(unsigned pin, int duty)
{
    unsigned forward = duty > 0 ? (unsigned)duty : 0u;
    unsigned reverse = duty < 0 ? (unsigned)-duty : 0u;

    pwm_pair(pin, forward, reverse);
}

void hardware_drive(int left, int right)
{
    UW key;

    DI(key);
    if (!brake_latched) {
        motor(PIN_MOTOR_LEFT_A, LEFT_MOTOR_SIGN * left);
        motor(PIN_MOTOR_RIGHT_A, RIGHT_MOTOR_SIGN * right);
    }
    EI(key);
}

static void count_edge(unsigned shift)
{
    unsigned count = (encoder_word >> shift) & 127u;

    if (count < EDGE_LIMIT) {
        encoder_word += (uint16_t)(1u << shift);
    } else {
        encoder_word |= EDGE_OVERFLOW;
    }
}

bool hardware_take_edges(uint8_t count[2])
{
    UW key;
    uint16_t packed;

    DI(key);
    packed = encoder_word;
    encoder_word = 0u;
    EI(key);
    count[0] = packed & 127u;
    count[1] = (packed >> 7) & 127u;
    return (packed & EDGE_OVERFLOW) == 0u;
}

void hardware_watchdog_feed(void)
{
    REG(WDOG_BASE + 4u) = WDOG_LOAD;
}

void hardware_watchdog_start(void)
{
    /* Reset RP2040 peripherals and processor on a control-task hang. */
    REG(0x40010008u) = 0x0001fffcu;
    hardware_watchdog_feed();
    REG(WDOG_BASE) = 1u << 30;
}

bool hardware_button_held(void)
{
    return gpio_get_val(PIN_START) == 0;
}

bool hardware_estop_held(void)
{
    return gpio_get_val(PIN_ESTOP) == 0;
}

void hardware_servo(unsigned angle)
{
    int degrees = (int)angle - 90;
    int pulse = SERVO_CENTER_US + degrees * SERVO_US_PER_60 / 60
                               * (SERVO_REVERSED ? -1 : 1);

    pwm_pair(PIN_SERVO, (unsigned)pulse, 0u);
}

static void pulse_end(UINT irq)
{
    (void)irq;
    REG(TIM_BASE + 0x34u) = 1u;
    gpio_set_val(PIN_TRIGGER, 0);
}

static void echo_edge(unsigned events)
{
    uint32_t now = app_us();
    uint32_t age = now - echo.armed_at;

    if (!echo.armed || age > ECHO_MAX_US) {
        return;
    }
    if (events == EDGE_RISE && !echo.rising && age >= 60u) {
        echo.rise_at = now;
        echo.rising = true;
    } else if (events == EDGE_FALL && echo.rising) {
        uint32_t width = now - echo.rise_at;
        if (width >= 90u && width <= ECHO_MAX_US) {
            echo.width = (uint16_t)width;
            echo.armed = false;
            (void)tk_set_flg(echo_events, 1u);
        } else {
            echo.rising = false;
        }
    }
}

static unsigned gpio_pending(unsigned pin)
{
    unsigned shift = (pin & 7u) * 4u;
    unsigned group = pin / 8u;
    uint32_t bits = REG(IO_BASE + 0x120u + group * 4u)
                    & (12u << shift);

    if (bits != 0u) {
        REG(IO_BASE + 0xf0u + group * 4u) = bits;
    }
    return bits >> shift;
}

static void gpio_isr(UINT irq)
{
    unsigned echo_bits;

    (void)irq;
    if (gpio_pending(PIN_ESTOP) != 0u) {
        car_fault(FAULT_ESTOP);
    }
    if (gpio_pending(PIN_ENCODER_LEFT) != 0u) {
        count_edge(0u);
    }
    if (gpio_pending(PIN_ENCODER_RIGHT) != 0u) {
        count_edge(7u);
    }
    echo_bits = gpio_pending(PIN_ECHO);
    if (echo_bits != 0u) {
        echo_edge(echo_bits);
    }
    if (gpio_pending(PIN_START) != 0u) {
        (void)tk_set_flg(controller_events, EVENT_BUTTON);
    }
}

static void input(unsigned pin, unsigned mode, unsigned events)
{
    unsigned shift = (pin & 7u) * 4u;
    unsigned offset = pin / 8u * 4u;

    gpio_set_pin(pin, mode);
    REG(IO_BASE + 0xf0u + offset) = 12u << shift;
    REG(IO_BASE + 0x2100u + offset) = events << shift;
}

static void pwm(unsigned pin, unsigned divider, unsigned top)
{
    unsigned slice = (pin >> 1) & 7u;
    unsigned base = PWM_ADDR + 0x14u * slice;

    REG(base) = 0;
    REG(base + 4u) = divider;
    REG(base + 8u) = 0;
    REG(base + 16u) = top;
    pwm_pair(pin, 0, 0);
    (void)pwm_set_pin(pin);
    REG(base) = 1u;
}

bool hardware_init(void)
{
    static const unsigned motor_pins[] = {
        PIN_MOTOR_LEFT_A, PIN_MOTOR_LEFT_B, PIN_MOTOR_RIGHT_A, PIN_MOTOR_RIGHT_B
    };
    T_DINT interrupt = { .intatr = TA_HLNG, .inthdr = (FP)gpio_isr };

    for (unsigned i = 0; i < 4u; ++i) {
        gpio_set_val(motor_pins[i], 0);
        gpio_set_pin(motor_pins[i], GPIO_MODE_OUT);
    }
    REG(0x4000f000u) = (1u << 14) | (1u << 21); /* RESETS CLR, PWM and timer */
    for (unsigned attempt = 0; attempt < 10000u; ++attempt) {
        if ((REG(0x4000c008u) & ((1u << 14) | (1u << 21)))
                                == ((1u << 14) | (1u << 21))) {
            break;
        }
        if (attempt == 9999u) {
            return false;
        }
    }
    REG(0x4005802cu) = 0x20cu; /* 12 MHz crystal / 12, one microsecond */
    REG(TIM_BASE + 0x30u) = 0u;
    pwm(PIN_MOTOR_LEFT_A, 100u, PWM_TOP);
    (void)pwm_set_pin(PIN_MOTOR_LEFT_B);
    pwm(PIN_MOTOR_RIGHT_A, 100u, PWM_TOP);
    (void)pwm_set_pin(PIN_MOTOR_RIGHT_B);
    pwm(PIN_SERVO, 2000u, 19999u);
    hardware_brake(true);
    hardware_servo(90u);
    gpio_set_val(PIN_TRIGGER, 0);
    gpio_set_pin(PIN_TRIGGER, GPIO_MODE_OUT);
    echo_events = app_events();
    if (echo_events <= 0 || tk_def_int(IO_IRQ, &interrupt) < E_OK) {
        return false;
    }
    interrupt.inthdr = (FP)pulse_end;
    if (tk_def_int(PULSE_IRQ, &interrupt) < E_OK) {
        return false;
    }
    input(PIN_ENCODER_LEFT, GPIO_MODE_IN, EDGE_RISE);
    input(PIN_ENCODER_RIGHT, GPIO_MODE_IN, EDGE_RISE);
    input(PIN_ECHO, GPIO_MODE_IN, EDGE_FALL | EDGE_RISE);
    input(PIN_ESTOP, GPIO_MODE_IN, EDGE_FALL);
    input(PIN_START, GPIO_MODE_IN, EDGE_FALL);
    /* Robo Pico buttons short to ground. Encoder pull-ups cover open drain. */
    const unsigned pull_pins[] = { PIN_ENCODER_LEFT, PIN_ENCODER_RIGHT, PIN_ESTOP, PIN_START };
    for (unsigned i = 0; i < 4u; ++i) {
        unsigned address = 0x4001c004u + 4u * pull_pins[i];
        REG(address) = (REG(address) & ~(1u << 2)) | (1u << 3);
    }
    REG(TIM_BASE + 0x34u) = 1u;
    REG(TIM_BASE + 0x2038u) = 1u;
    /* Alarm and GPIO share the highest configurable external IRQ priority.
     * Single-core profile means no cross-core ISR/task data access. */
    EnableInt(PULSE_IRQ, 0);
    EnableInt(IO_IRQ, 0);
    return true;
}

bool hardware_ping(uint16_t *generation)
{
    UW key;

    (void)tk_clr_flg(echo_events, 0u);
    DI(key);
    if (gpio_get_val(PIN_ECHO) != 0) {
        EI(key);
        return false;
    }
    echo.width = 0u;
    echo.rising = false;
    echo.armed = true;
    *generation = ++echo.serial;
    gpio_set_val(PIN_TRIGGER, 1);
    echo.armed_at = app_us();
    REG(TIM_BASE + 0x10u) = echo.armed_at + 10u;
    EI(key);
    return true;
}

bool hardware_echo(uint16_t generation, uint16_t *pulse_us)
{
    UW key;
    UINT event;

    (void)tk_wai_flg(echo_events, 1u, TWF_ORW | TWF_BITCLR, &event, 30);
    DI(key);
    *pulse_us = echo.serial == generation ? echo.width : 0u;
    echo.armed = false;
    EI(key);
    return *pulse_us != 0u;
}
