#include "controller.h"
#include "../app.h"
#include "../car_config.h"
#include "../hardware.h"
#include "../subsystem2/motion.h"
#include "../subsystem1/routes.h"
#include "../subsystem3/line.h"
#include "../subsystem4/terrain.h"
#include "../subsystem5/obstacle.h"

enum { NAV_APPROACH, NAV_TURN, NAV_EXIT };
enum { DETOUR_TURN, DETOUR_OUT, DETOUR_FACE, DETOUR_PAST, DETOUR_INWARD,
       DETOUR_CROSS, DETOUR_RESTORE, DETOUR_REVERSE };

typedef struct {
    uint32_t last_button;
    uint32_t line_at;
    uint32_t state_at;
    uint32_t run_origin;
    uint32_t nav_origin;
    uint32_t range_after;
    uint32_t crossing_origin;
    int16_t last_error;
    int8_t steering;
    uint8_t phase : 3;
    uint8_t setup : 2;
    uint8_t nav : 2;
    uint8_t detour : 3;
    uint8_t route : 3;
    uint8_t retries : 1;
    uint8_t hunt : 2;
    uint8_t lock_on : 3;
    uint8_t last_scan_id : 4;
    uint8_t no_echo : 3;
    uint8_t barcode_id;
    uint8_t observed_barcode : 5;
    bool range_needed : 1;
    bool center_on_line : 1;
    bool run_started : 1;
    obstacle_t obstacle;
} mission_t;

static car_status_t shared;
static mission_t mission;

static int clamp(int value, int lo, int hi)
{
    return value < lo ? lo : value > hi ? hi : value;
}

static int absolute(int value)
{
    return value < 0 ? -value : value;
}

static void transition(unsigned phase, uint32_t now)
{
    mission.phase = (uint8_t)phase;
    mission.state_at = now;
    line_read_barcode(phase == CAR_FOLLOW);
}

static void stop_fault(uint8_t fault, uint32_t now)
{
    car_fault(fault);
    motion_stop();
    transition(CAR_STOPPED, now);
}

static void begin_scan(uint32_t now, bool new_obstacle)
{
    if (new_obstacle) {
        mission.retries = 0u;
        mission.route = ROUTE_NONE;
        mission.hunt = 0u;
    }
    motion_stop();
    transition(CAR_SCAN, now);
    sonar_scan();
}

static bool line_fresh(line_sample_t line, uint32_t now)
{
    return line.calibrated && now - line.time_us < 40000u;
}

static void begin_follow(uint32_t now, line_sample_t line)
{
    mission.steering = 0;
    mission.last_error = line.error;
    mission.line_at = now;
    mission.lock_on = 0;
    mission.barcode_id = line.barcode_id;
    transition(CAR_FOLLOW, now);
}

static void button(uint32_t now, line_sample_t line, terrain_sample_t terrain,
                   motion_sample_t motion)
{
    if (now - mission.last_button < 230000u || !hardware_button_held()) { return; }
    mission.last_button = now;
    if (mission.phase == CAR_SETUP) {
        if (mission.setup == SETUP_WHITE || mission.setup == SETUP_BAD) {
            line_calibrate(CAL_WHITE);
            mission.setup = SETUP_WAIT;
            tm_printf((UB *)"Sampling white\n");
        } else if (mission.setup == SETUP_BLACK) {
            line_calibrate(CAL_BLACK);
            mission.setup = SETUP_WAIT;
            tm_printf((UB *)"Sampling black\n");
        }
        return;
    }
    if (mission.phase != CAR_READY && mission.phase != CAR_STOPPED) { return; }

    /* Local physical start can acknowledge course faults once the operator
     * has repositioned the car. Hardware, timing, and impact faults require reset. */
    UW key;
    DI(key);
    if (!hardware_estop_held()) {
        car_faults &= (uint8_t)~(FAULT_ESTOP | FAULT_LINE | FAULT_PATH);
    }
    bool safe = !hardware_estop_held() && (car_faults & FATAL_FAULTS) == 0u;
    EI(key);
    if (!safe || !terrain.ready || !line_fresh(line, now) || !line.visible) {
        tm_printf((UB *)"Start rejected: align line, release stop, and check status\n");
        return;
    }
    mission.run_origin = motion.travelled_mm;
    mission.run_started = true;
    mission.route = ROUTE_NONE;
    mission.obstacle = (obstacle_t){0};
    mission.observed_barcode = 0u;
    terrain_begin_run();
    begin_follow(now, line);
}

static void setup_step(uint32_t now)
{
    if (mission.setup != SETUP_WAIT) { return; }
    unsigned result = line_calibration();
    if (result == CAL_WHITE) {
        mission.setup = SETUP_BLACK;
        tm_printf((UB *)"Place all three sensors over matte black. Press GP21.\n");
    } else if (result == CAL_COMPLETE) {
        transition(CAR_READY, now);
        tm_printf((UB *)"Align on the track with GP27 over the barcode lane. Press GP21.\n");
    } else if (result == CAL_INVALID) {
        mission.setup = SETUP_BAD;
        tm_printf((UB *)"IR contrast too low. Place all sensors over white and retry.\n");
    }
}

static void steering(line_sample_t line, uint32_t now, unsigned base)
{
    if (line.visible) {
        mission.line_at = now;
        int error = absolute(line.error) <= 62 ? 0
                  : line.error - (line.error < 0 ? -62 : 62);
        int derivative = clamp((error - mission.last_error) / 12, -12, 12);
        int limit = line.barcode_busy ? 23 : 44;
        int demand = clamp(error / (line.barcode_busy ? 25 : 16) + derivative,
                           -limit, limit);
        int change = clamp(demand - mission.steering, -7, 7);
        mission.steering = (int8_t)(mission.steering + change);
        mission.last_error = (int16_t)error;
        base -= (unsigned)absolute(mission.steering) / 3u;
        if (!line.barcode_busy && absolute(error) > 460 && base > 78u) { base = 78u; }
    } else {
        /* Coast through short print gaps with a small remembered correction. */
        mission.steering = (int8_t)clamp(mission.steering, -19, 19);
        base = 57u;
    }
    motion_velocity((int)base - mission.steering, (int)base + mission.steering);
}

static bool start_turn(int degrees, uint32_t now)
{
    if (motion_turn(degrees)) { return true; }
    stop_fault(FAULT_MOTION, now);
    return false;
}

static bool start_move(int mm, uint32_t now)
{
    if (motion_move(mm)) { return true; }
    stop_fault(FAULT_MOTION, now);
    return false;
}

static void reacquire(uint32_t now)
{
    motion_stop();
    mission.hunt = 0u;
    mission.lock_on = 0u;
    transition(CAR_REACQUIRE, now);
}

static void depart_junction(uint32_t now)
{
    if (mission.route == ROUTE_STRAIGHT) {
        mission.nav = NAV_EXIT;
        (void)start_move(70, now);
    } else {
        int degrees = mission.route == ROUTE_LEFT ? 90 : mission.route == ROUTE_RIGHT ? -90 : 180;
        mission.nav = NAV_TURN;
        (void)start_turn(degrees, now);
    }
}

static void navigate(line_sample_t line, motion_sample_t motion, uint32_t now)
{
    if (line.barcode_id != mission.barcode_id) {
        mission.barcode_id = line.barcode_id;
        mission.observed_barcode = line.letter;
        char letter = line.letter != 0u ? (char)('A' + line.letter - 1) : 0;
        unsigned action = route_config_action(letter);
        if (action != ROUTE_NONE && mission.route == ROUTE_NONE) {
            mission.route = (uint8_t)action;
            mission.nav_origin = motion.travelled_mm;
        }
    }
    if (mission.route != ROUTE_NONE && line_fresh(line, now)) {
        unsigned since = motion.travelled_mm - mission.nav_origin;
        bool measured_offset = false;
#if BARCODE_TO_JUNCTION_MM > 0
        measured_offset = since >= BARCODE_TO_JUNCTION_MM;
#endif
        if (line.junction || measured_offset) {
            transition(CAR_JUNCTION, now);
            motion_stop();
            if (line.junction) {
                mission.nav = NAV_APPROACH;
                (void)start_move(IR_TO_AXLE_MM, now);
            } else {
                depart_junction(now);
            }
            return;
        }
        if (since >= JUNCTION_WINDOW_MM ||
            (!line.visible && now - mission.line_at > 210000u)) {
            stop_fault(FAULT_PATH, now);
            return;
        }
    }
    if (!line_fresh(line, now)) {
        stop_fault(FAULT_LINE, now);
    } else if (!line.visible && now - mission.line_at > 210000u) {
        reacquire(now);
    } else {
        steering(line, now, line.barcode_busy || mission.route != ROUTE_NONE
                            ? BARCODE_SPEED_MM_S : LINE_SPEED_MM_S);
    }
}

static void junction_step(move_result_t result, uint32_t now)
{
    if (result != MOVE_COMPLETE) { return; }

    if (mission.nav == NAV_APPROACH) {
        depart_junction(now);
    } else {
        mission.route = ROUTE_NONE;
        reacquire(now);
    }
}

static void scan_step(uint32_t now, motion_sample_t motion, line_sample_t line)
{
    obstacle_t output;

    if (!sonar_take_profile(&output)) {
        if (now - mission.state_at > 5400000u) { stop_fault(FAULT_PATH, now); }
        return;
    }
    mission.obstacle = output;
    if (output.plan == PLAN_CONTINUE) {
        if (line.visible && line_fresh(line, now)) {
            begin_follow(now, line);
        } else {
            reacquire(now);
        }
    } else if (output.plan == PLAN_LEFT || output.plan == PLAN_RIGHT) {
        mission.detour = DETOUR_TURN;
        mission.center_on_line = false;
        mission.range_needed = false;
        transition(CAR_DETOUR, now);
        (void)start_turn(output.plan == PLAN_LEFT ? 90 : -90, now);
    } else if (output.plan == PLAN_REVERSE && !mission.retries &&
               motion.travelled_mm - mission.run_origin >= 100u) {
        mission.retries = 1u;
        mission.range_needed = false;
        mission.detour = DETOUR_REVERSE;
        transition(CAR_DETOUR, now);
        (void)start_move(-80, now);
    } else {
        stop_fault(FAULT_PATH, now);
    }
}

static void wait_forward_beam(uint32_t now, range_t range)
{
    mission.range_needed = true;
    mission.range_after = now + 90000u;
    mission.last_scan_id = range.generation;
}

static void detour_step(move_result_t result, uint32_t now, range_t range,
                        line_sample_t line, uint32_t distance_mm)
{
    int side = mission.obstacle.plan == PLAN_LEFT ? 1 : -1;

    if (mission.range_needed) {
        if ((int32_t)(now - (mission.range_after + 340000u)) > 0) {
            stop_fault(FAULT_PATH, now);
        } else if (range.generation != mission.last_scan_id &&
                   (int32_t)(now - mission.range_after) >= 0) {
            mission.range_needed = false;
            unsigned distance = mission.detour == DETOUR_OUT ? mission.obstacle.offset_mm
                              : mission.detour == DETOUR_PAST ? mission.obstacle.forward_mm
                              : mission.obstacle.offset_mm + 65u;
            if (range.range_mm != 0u && range.range_mm < distance + HARD_RANGE_MM) {
                stop_fault(FAULT_PATH, now);
            } else {
                if (mission.detour == DETOUR_CROSS) { mission.crossing_origin = distance_mm; }
                (void)start_move((int)distance, now);
            }
        }
        return;
    }
    int crossing_min = (int)mission.obstacle.offset_mm - IR_TO_AXLE_MM - 18;
    if (mission.detour == DETOUR_CROSS && !mission.center_on_line &&
        line_fresh(line, now) && line.visible &&
        (int)(distance_mm - mission.crossing_origin) >= crossing_min) {
        if (++mission.lock_on >= 3u) {
            motion_stop();
            mission.center_on_line = true;
            (void)start_move(IR_TO_AXLE_MM - 3, now);
            return;
        }
    } else {
        mission.lock_on = 0u;
    }
    if (result != MOVE_COMPLETE) { return; }

    switch (mission.detour) {
    case DETOUR_TURN:
    case DETOUR_FACE:
    case DETOUR_INWARD:
        ++mission.detour;
        wait_forward_beam(now, range);
        break;
    case DETOUR_OUT:
    case DETOUR_PAST:
        ++mission.detour;
        (void)start_turn(-side * 90, now);
        break;
    case DETOUR_CROSS:
        if (mission.center_on_line) {
            mission.detour = DETOUR_RESTORE;
            (void)start_turn(side * 90, now);
        } else {
            stop_fault(FAULT_LINE, now);
        }
        break;
    case DETOUR_RESTORE:
        reacquire(now);
        break;
    case DETOUR_REVERSE:
        begin_scan(now, false);
        break;
    default:
        stop_fault(FAULT_PATH, now);
        break;
    }
}

static void recover(move_result_t result, line_sample_t line, uint32_t now)
{
    const int turns[] = { 18, -36, 36 };

    if (!line_fresh(line, now) || now - mission.state_at > 2800000u) {
        stop_fault(FAULT_LINE, now);
        return;
    }
    if (line.visible && absolute(line.error) <= 620) {
        if (++mission.lock_on >= 4u) {
            motion_stop();
            begin_follow(now, line);
            return;
        }
    } else {
        mission.lock_on = 0u;
    }
    if (result == MOVE_RUNNING || mission.lock_on != 0u ||
        now - mission.state_at < 70000u) {
        return;
    }
    if (mission.hunt >= 3u) {
        stop_fault(FAULT_LINE, now);
        return;
    }
    int side = mission.last_error < 0 ? -1 : 1;
    int turn = turns[mission.hunt++];
    (void)start_turn(side * turn, now);
}

static bool forward_segment(void)
{
    if (mission.phase == CAR_JUNCTION) {
        return mission.nav == NAV_APPROACH || mission.nav == NAV_EXIT;
    }
    return mission.phase == CAR_DETOUR && !mission.range_needed &&
           (mission.detour == DETOUR_OUT || mission.detour == DETOUR_PAST ||
            mission.detour == DETOUR_CROSS);
}

static void update_range(range_t range, uint32_t now)
{
    if (range.range_mm == 0u) {
        if (mission.no_echo < 7u) { ++mission.no_echo; }
    } else {
        mission.no_echo = 0u;
    }
    UW key;
    DI(key);
    if (mission.no_echo >= 4u) { car_faults |= FAULT_NO_ECHO; }
    else { car_faults &= (uint8_t)~FAULT_NO_ECHO; }
    EI(key);
    if (mission.phase == CAR_FOLLOW && range.range_mm != 0u &&
        range.range_mm <= STOP_RANGE_MM) {
        begin_scan(now, true);
    } else if (forward_segment() && range.range_mm != 0u && range.range_mm <= HARD_RANGE_MM) {
        stop_fault(FAULT_PATH, now);
    }
}

static void publish_status(motion_sample_t motion, line_sample_t line, terrain_sample_t terrain,
                           range_t range)
{
    car_status_t next = {
        .distance_mm = mission.run_started ? motion.travelled_mm - mission.run_origin : 0u,
        .front_mm = range.range_mm,
        .max_hump_mm = terrain.max_hump_mm,
        .obstacle_width_mm = mission.obstacle.width_mm,
        .speed_mm_s = clamp(motion.speed_mm_s, -512, 511),
        .yaw_deg_s = terrain.yaw_deg_s,
        .phase = mission.phase,
        .terrain = terrain.event,
        .barcode = mission.observed_barcode,
        .plan = mission.obstacle.plan,
        .line = line.visible,
        .magnetic_rate = terrain.magnetic_rate,
    };
    UW key;
    DI(key);
    next.faults = car_faults;
    shared = next;
    EI(key);
}

car_status_t controller_status(void)
{
    UW key;
    car_status_t copy;

    DI(key);
    copy = shared;
    EI(key);
    return copy;
}

static void controller_task(INT argument, void *context)
{
    UINT events;
    uint32_t last_control = app_us();

    (void)argument;
    (void)context;
    tm_printf((UB *)"Place all three IR sensors over white. Press GP21. GP20 stops.\n");
    for (;;) {
        (void)tk_wai_flg(controller_events, 31u, TWF_ORW | TWF_BITCLR, &events, TMO_FEVR);
        uint32_t now = app_us();
        range_t range = sonar_range();
        line_sample_t line = line_sample();
        terrain_sample_t terrain = terrain_sample();
        motion_sample_t motion = motion_sample();
        bool range_update = (events & EVENT_RANGE) != 0u;

        if ((events & EVENT_STOP) != 0u || (car_faults & FATAL_FAULTS) != 0u) {
            motion_stop();
            if (mission.phase != CAR_SETUP) { transition(CAR_STOPPED, now); }
        }
        if ((events & EVENT_BUTTON) != 0u && (events & EVENT_STOP) == 0u) {
            button(now, line, terrain, motion);
        }
        if (range_update) { update_range(range, now); }
        if ((events & EVENT_TICK) == 0u) {
            publish_status(motion, line, terrain, range);
            continue;
        }

        /* Cyclic callbacks can coalesce. Enforce a physical minimum interval
         * and let the motion owner fault if a control deadline was missed. */
        if (now - last_control < 7000u) { continue; }
        last_control = now;
        hardware_watchdog_feed();
        move_result_t result = motion_update(now);
        motion = motion_sample();
        if (result == MOVE_FAILED && mission.phase != CAR_SETUP) {
            transition(CAR_STOPPED, now);
        }
        switch (mission.phase) {
        case CAR_SETUP: setup_step(now); break;
        case CAR_FOLLOW: navigate(line, motion, now); break;
        case CAR_JUNCTION: junction_step(result, now); break;
        case CAR_SCAN: scan_step(now, motion, line); break;
        case CAR_DETOUR: detour_step(result, now, range, line, motion.travelled_mm); break;
        case CAR_REACQUIRE: recover(result, line, now); break;
        default: break;
        }
        publish_status(motion, line, terrain, range);
    }
}

bool controller_start(void)
{
    return app_periodic(controller_events, 10u) && app_start((FP)controller_task, 3, 1600);
}
