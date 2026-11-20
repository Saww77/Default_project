#include <assert.h>
#include <stdio.h>
#include "fake_kernel.h"

#include "vehicle_controller/controller.c"
#include "subsystem1/route_config.c"

ID controller_events = 1;
volatile uint8_t car_faults;
static int last_move;
static int last_turn;
static bool braking;
static bool impact_armed;
static int requested_speed[2];
static unsigned scan_requests;
static line_sample_t line;
static obstacle_t profile;
static bool profile_valid;

void car_fault(uint8_t faults) { car_faults |= faults; braking = true; }
void motion_stop(void) { braking = true; }
void motion_velocity(int left, int right)
{
    assert(left >= 0 && right >= 0);
    requested_speed[0] = left;
    requested_speed[1] = right;
    braking = false;
}
bool motion_move(int mm) { last_move = mm; braking = false; return true; }
bool motion_turn(int degrees) { last_turn = degrees; braking = false; return true; }
bool hardware_button_held(void) { return true; }
bool hardware_estop_held(void) { return false; }
void line_read_barcode(bool enabled) { (void)enabled; }
void line_calibrate(unsigned surface) { (void)surface; }
void terrain_set_armed(bool armed) { impact_armed = armed; }
void terrain_begin_run(void) { }
int tm_printf(UB *format, ...) { (void)format; return 0; }
void sonar_scan(void) { ++scan_requests; }
bool sonar_take_profile(obstacle_t *output)
{
    *output = profile;
    bool had = profile_valid;
    profile_valid = false;
    return had;
}

static void set_letter(char letter)
{
    line.letter = (uint8_t)(letter - 'A' + 1);
    ++line.barcode_id;
}

static void check_junction(void)
{
    uint32_t now = 1000000u;
    line = (line_sample_t){.calibrated = true, .visible = true, .time_us = now};
    motion_sample_t motion = {.travelled_mm = 350};
    terrain_sample_t terrain = {.ready = true};
    routes_t routes = routes_default();
    assert(routes_set(&routes, 'A', ROUTE_LEFT));
    assert(routes_set(&routes, 'X', ROUTE_RIGHT));
    route_config_init();
    assert(route_config_apply(&routes));

    mission.phase = CAR_READY;
    button(now, line, terrain, motion);
    assert(mission.phase == CAR_FOLLOW && impact_armed);
    set_letter('A');
    navigate(line, motion, now);
    assert(mission.route == ROUTE_LEFT && mission.phase == CAR_FOLLOW && last_turn == 0);

    /* Make A go straight over MQTT while the left turn is already queued. */
    assert(routes_set(&routes, 'A', ROUTE_STRAIGHT));
    assert(route_config_apply(&routes));
    line.junction = true;
    navigate(line, motion, now);
    assert(mission.phase == CAR_JUNCTION && mission.nav == NAV_APPROACH);
    assert(last_move == IR_TO_AXLE_MM && last_turn == 0);
    junction_step(MOVE_COMPLETE, now);
    assert(last_turn == 90);
    junction_step(MOVE_COMPLETE, now);
    assert(mission.phase == CAR_REACQUIRE && mission.route == ROUTE_NONE && braking);

    begin_follow(now, line);
    set_letter('A');
    navigate(line, motion, now);
    junction_step(MOVE_COMPLETE, now);
    assert(mission.nav == NAV_EXIT && last_move == 70);
    junction_step(MOVE_COMPLETE, now);
    begin_follow(now, line);
    set_letter('X');
    line.junction = false;
    navigate(line, motion, now);
    motion.travelled_mm += JUNCTION_WINDOW_MM + 1;
    navigate(line, motion, now);
    assert(mission.phase == CAR_STOPPED && braking && !impact_armed && (car_faults & FAULT_PATH));
}

static void check_avoidance(void)
{
    mission = (mission_t){.phase = CAR_FOLLOW, .run_started = true};
    car_faults = 0;
    scan_requests = 0;
    braking = false;

    update_range((range_t){.range_mm = 205}, 2000000);
    assert(mission.phase == CAR_SCAN && braking && scan_requests == 1);
    profile = (obstacle_t){.plan = PLAN_LEFT, .offset_mm = 190, .forward_mm = 365};
    profile_valid = true;
    scan_step(3000000, (motion_sample_t){.travelled_mm = 170}, line);
    assert(mission.phase == CAR_DETOUR && last_turn == 90);
    range_t range = {.generation = 1};
    detour_step(MOVE_COMPLETE, 3000000, range, line, 170);
    assert(mission.range_needed && mission.detour == DETOUR_OUT);
    ++range.generation;
    range.range_mm = 650;
    detour_step(MOVE_IDLE, 3100000, range, line, 170);
    assert(!mission.range_needed && last_move == 190);
    detour_step(MOVE_COMPLETE, 6000000, range, line, 360);
    assert(last_turn == -90 && mission.detour == DETOUR_FACE);
    detour_step(MOVE_COMPLETE, 7500000, range, line, 440);
    assert(mission.range_needed && mission.detour == DETOUR_PAST);
    ++range.generation;
    detour_step(MOVE_IDLE, 7600000, range, line, 440);
    assert(last_move == 365);
    detour_step(MOVE_COMPLETE, 11500000, range, line, 805);
    assert(last_turn == -90 && mission.detour == DETOUR_INWARD);
    detour_step(MOVE_COMPLETE, 12900000, range, line, 890);
    ++range.generation;
    detour_step(MOVE_IDLE, 13010000, range, line, 890);
    assert(last_move == 255 && mission.detour == DETOUR_CROSS);
    line.time_us = 15000000;
    for (unsigned i = 0; i < 3; ++i) {
        detour_step(MOVE_RUNNING, line.time_us, range, line, 1026);
    }
    assert(last_move == IR_TO_AXLE_MM - 3 && mission.center_on_line);
    detour_step(MOVE_COMPLETE, 15800000, range, line, 1090);
    assert(last_turn == 90 && mission.detour == DETOUR_RESTORE);
    detour_step(MOVE_COMPLETE, 17500000, range, line, 1175);
    assert(mission.phase == CAR_REACQUIRE && braking);
    line.time_us = 17700000;
    line.error = 30;
    for (unsigned i = 0; i < 4; ++i) {
        recover(MOVE_IDLE, line, line.time_us);
    }
    assert(mission.phase == CAR_FOLLOW && impact_armed);

    mission.phase = CAR_DETOUR;
    mission.detour = DETOUR_PAST;
    update_range((range_t){.range_mm = 80}, 18000000);
    assert(mission.phase == CAR_STOPPED && braking && !impact_armed && (car_faults & FAULT_PATH));
}

static void check_tracking(void)
{
    mission = (mission_t){0};
    line = (line_sample_t){.visible = true};
    for (int i = 0; i < 30; ++i) {
        line.error = i % 2 == 0 ? -40 : 40;
        steering(line, 10000u * (unsigned)i, LINE_SPEED_MM_S);
        assert(requested_speed[0] == requested_speed[1]);
    }
    line.error = 510;
    steering(line, 400000, LINE_SPEED_MM_S);
    assert(mission.steering == 7 && requested_speed[0] < requested_speed[1]);
    line.barcode_busy = true;
    for (unsigned i = 0; i < 12; ++i) { steering(line, 450000u + 10000u * i, BARCODE_SPEED_MM_S); }
    assert(mission.steering <= 23);
}

int main(void)
{
    check_tracking();
    check_junction();
    check_avoidance();
    puts("PASS line steering, MQTT remapping at junction, detour, line recovery and obstacle stop");
    return 0;
}
