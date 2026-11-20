#include "obstacle.h"
#include "../hardware.h"
#include "../car_config.h"

#define SCAN_REQUEST 1u
#define SONAR_INTERVAL_US 62000u
#define SERVO_MS_PER_DEG 3u

static ID scan_events;
static range_t range;
static obstacle_t latest;
static bool profile_ready;
static unsigned angle_now = 90u;
static uint32_t last_ping;

static scan_point_t measure(unsigned angle, bool extra_sample)
{
    unsigned difference = angle >= angle_now ? angle - angle_now : angle_now - angle;
    unsigned attempts = extra_sample ? 2u : 1u;
    unsigned nearest = 0u;

    if (difference != 0u) {
        hardware_servo(angle);
        angle_now = angle;
        (void)tk_dly_tsk(38u + difference * SERVO_MS_PER_DEG);
    }
    for (unsigned i = 0; i < attempts; ++i) {
        uint16_t ticket;
        uint16_t duration;
        unsigned elapsed = app_us() - last_ping;
        if (elapsed < SONAR_INTERVAL_US) {
            (void)tk_dly_tsk((SONAR_INTERVAL_US - elapsed + 999u) / 1000u);
        }
        last_ping = app_us();
        if (hardware_ping(&ticket) && hardware_echo(ticket, &duration)) {
            unsigned mm = ((unsigned)duration * 343u + 1000u) / 2000u;
            if (mm <= 4095u && (nearest == 0u || mm < nearest)) { nearest = mm; }
        }
    }
    return (scan_point_t){.mm = nearest, .angle = angle};
}

static obstacle_t sweep(void)
{
    scan_point_t coarse[5];
    scan_point_t fine[9];
    unsigned closest = 540u;
    unsigned mid = 90u;
    unsigned count = 0u;

    for (unsigned i = 0; i < 5u; ++i) {
        coarse[i] = measure(30u + 30u * i, true);
        if (i > 0u && i < 4u && coarse[i].mm != 0u && coarse[i].mm < closest) {
            closest = coarse[i].mm;
            mid = coarse[i].angle;
        }
    }
    if (closest < 540u) {
        for (unsigned angle = mid - 20u; angle <= mid + 20u; angle += 5u) {
            fine[count++] = measure(angle, false);
        }
    }
    return obstacle_profile(coarse, fine, count);
}

void sonar_scan(void)
{
    UW key;

    DI(key);
    profile_ready = false;
    EI(key);
    (void)tk_set_flg(scan_events, SCAN_REQUEST);
}

range_t sonar_range(void)
{
    UW key;
    range_t copy;

    DI(key);
    copy = range;
    EI(key);
    return copy;
}

bool sonar_take_profile(obstacle_t *profile)
{
    bool ready;
    UW key;

    DI(key);
    ready = profile_ready;
    if (ready) {
        *profile = latest;
        profile_ready = false;
    }
    EI(key);
    return ready;
}

static void sonar_task(INT argument, void *context)
{
    UINT flags;
    UW key;

    (void)argument;
    (void)context;
    for (;;) {
        ER event = tk_wai_flg(scan_events, SCAN_REQUEST, TWF_ORW | TWF_BITCLR, &flags, 1);
        if (event >= E_OK) {
            obstacle_t profile = sweep();
            /* Restore the forward beam before returning drive authority. */
            scan_point_t center = measure(90u, false);
            DI(key);
            range.range_mm = center.mm;
            ++range.generation;
            latest = profile;
            profile_ready = true;
            EI(key);
        } else {
            scan_point_t center = measure(90u, false);
            DI(key);
            range.range_mm = center.mm;
            ++range.generation;
            EI(key);
        }
        (void)tk_set_flg(controller_events, EVENT_RANGE);
    }
}

bool sonar_start(void)
{
    scan_events = app_events();
    return scan_events > 0 && app_start((FP)sonar_task, 5, 1200);
}
