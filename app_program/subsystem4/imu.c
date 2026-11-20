#include "terrain.h"
#include "../app.h"
#include "../car_config.h"
#include "../subsystem2/motion.h"
#include "../../device/include/dev_i2c.h"

#define ACCEL 0x19u
#define MAG   0x1eu
#define CAL_COUNT 64u

static terrain_sample_t shared;
static ID timer;
static volatile uint8_t run_epoch;

static int absolute(int x)
{
    return x < 0 ? -x : x;
}

static bool burst(ID dev, UW address, UB reg, UB *data, SZ bytes)
{
    SZ actual;
    T_I2C_EXEC transfer = {
        .sadr = address, .snd_size = 1, .snd_data = &reg,
        .rcv_size = bytes, .rcv_data = data
    };

    return tk_swri_dev(dev, TDN_I2C_EXEC, &transfer, sizeof(transfer), &actual) >= E_OK
           && actual == sizeof(transfer);
}

static bool setup(ID dev)
{
    UB acc;
    UB mag[3];

    if (!burst(dev, ACCEL, 0x0fu, &acc, 1) || acc != 0x33u ||
        !burst(dev, MAG, 0x0au, mag, 3) ||
        mag[0] != 'H' || mag[1] != '4' || mag[2] != '3') {
        return false;
    }
    /* BDU + high resolution ±4g, then 100 Hz XYZ. Mag 75 Hz continuous. */
    return i2c_write_reg(dev, ACCEL, 0x23, 0x98) >= E_OK
        && i2c_write_reg(dev, ACCEL, 0x20, 0x57) >= E_OK
        && i2c_write_reg(dev, MAG, 0x00, 0x18) >= E_OK
        && i2c_write_reg(dev, MAG, 0x01, 0x20) >= E_OK
        && i2c_write_reg(dev, MAG, 0x02, 0x00) >= E_OK;
}

static int16_t word(UB high, UB low)
{
    return (int16_t)((uint16_t)high << 8 | low);
}

static int wrapped_delta(int next, int previous)
{
    int delta = next - previous;
    if (delta > 180000) { return delta - 360000; }
    if (delta < -180000) { return delta + 360000; }
    return delta;
}

void terrain_begin_run(void)
{
    ++run_epoch;
}

terrain_sample_t terrain_sample(void)
{
    terrain_sample_t copy;
    UW key;

    DI(key);
    copy = shared;
    EI(key);
    return copy;
}

static void terrain_task(INT argument, void *context)
{
    ID imu = tk_opn_dev((const UB *)"iica", TD_READ | TD_WRITE);
    uint32_t previous_position = 0u;
    uint32_t last_accel = app_us();
    uint32_t next_mag = 0u;
    uint32_t mag_time = 0u;
    int last_heading = 0;
    int magnetic_turn = 0;
    int bias[3] = {0};
    int acc[3] = {0};
    int baseline = 0;
    int last_speed = 0;
    int longitudinal_q8 = 0;
    unsigned samples = 0u;
    uint8_t impacts = 0u;
    unsigned stable_ms = 0u;
    uint8_t epoch = 0u;
    hump_t hump = {0};
    terrain_sample_t next = {0};
    UINT flags;
    UW key;

    (void)argument;
    (void)context;
    if (imu < E_OK || !setup(imu)) {
        car_fault(FAULT_IMU);
        return;
    }
    last_accel = app_us();
    for (;;) {
        UB raw[7];
        (void)tk_wai_flg(timer, EVENT_TICK, TWF_ORW | TWF_BITCLR, &flags, TMO_FEVR);
        uint32_t now = app_us();
        if (!burst(imu, ACCEL, 0xa7u, raw, 7) || !(raw[0] & 8u)) {
            if (now - last_accel > 70000u) { car_fault(FAULT_IMU); }
            continue;
        }
        unsigned dt_us = now - last_accel;
        last_accel = now;
        for (unsigned i = 0; i < 3u; ++i) {
            acc[i] = (word(raw[2u + 2u * i], raw[1u + 2u * i]) / 16) * 2;
        }
        int fwd = acc[IMU_FORWARD_AXIS] * IMU_FORWARD_SIGN;
        int left = acc[IMU_LEFT_AXIS] * IMU_LEFT_SIGN;
        int up = acc[IMU_UP_AXIS] * IMU_UP_SIGN;
        int norm_sq = fwd * fwd + left * left + up * up;
        if (samples < CAL_COUNT) {
            if (up < 750 || up > 1250 || absolute(fwd) > 350 || absolute(left) > 350) {
                samples = 0u;
                bias[0] = bias[1] = bias[2] = 0;
                if (++stable_ms >= 200u) { car_fault(FAULT_IMU); }
                continue;
            }
            if (samples != 0u && (absolute(fwd - bias[0] / (int)samples) > 95 ||
                absolute(left - bias[1] / (int)samples) > 95 ||
                absolute(up - bias[2] / (int)samples) > 95)) {
                samples = 0u;
                bias[0] = bias[1] = bias[2] = 0;
                if (++stable_ms >= 200u) { car_fault(FAULT_IMU); }
                continue;
            }
            bias[0] += fwd;
            bias[1] += left;
            bias[2] += up;
            if (++samples == CAL_COUNT) {
                baseline = terrain_atan(bias[0] / (int)samples, bias[2] / (int)samples);
                longitudinal_q8 = baseline * 256;
                next.ready = true;
            }
        }
        if (!next.ready) { continue; }
        motion_sample_t motion = motion_sample();
        if (epoch != run_epoch) {
            epoch = run_epoch;
            hump = (hump_t){0};
            previous_position = motion.travelled_mm;
        }
        unsigned distance = motion.travelled_mm - previous_position;
        previous_position = motion.travelled_mm;
        int speed = motion.speed_mm_s;
        int yaw = motion.yaw_deg_s;
        /* Subtract the measured change in wheel speed from longitudinal specific force. */
        int dynamic = dt_us >= 6000u && dt_us <= 25000u
                      ? (last_speed - speed) * 102000 / (int)dt_us : 0;
        last_speed = speed;
        if (dynamic < -180) { dynamic = -180; }
        if (dynamic > 180) { dynamic = 180; }
        int current_angle = terrain_atan(fwd - dynamic, up);
        longitudinal_q8 += (current_angle * 256 - longitudinal_q8) / 5;
        int slope = longitudinal_q8 / 256 - baseline;
        bool gravity_like = norm_sq > 630000 && norm_sq < 1660000;
        bool driving = speed > 25 && absolute(yaw) < 28;
        terrain_hump_update(&hump, slope, distance, motion.travelled_mm,
                             gravity_like && distance <= 8u, driving);
        if ((int32_t)(now - next_mag) >= 0 || next_mag == 0u) {
            UB magnetic[6];
            next_mag = now + 40000u;
            if (burst(imu, MAG, 0x03u, magnetic, 6)) {
                int field[3] = {
                    word(magnetic[0], magnetic[1]) - IMU_MAG_X_OFFSET,
                    word(magnetic[4], magnetic[5]) - IMU_MAG_Y_OFFSET,
                    word(magnetic[2], magnetic[3]) * 11 / 10 - IMU_MAG_Z_OFFSET
                };
                int mx = field[IMU_FORWARD_AXIS] * IMU_FORWARD_SIGN;
                int my = field[IMU_LEFT_AXIS] * IMU_LEFT_SIGN;
                int mz = field[IMU_UP_AXIS] * IMU_UP_SIGN;
                if (absolute(mx) < 2000 && absolute(my) < 2000 && absolute(mz) < 2000 &&
                    up >= 650 && mx * mx + my * my + mz * mz > 10000) {
                    int heading = terrain_atan(my * up - mz * left, mx * up - mz * fwd);
                    unsigned elapsed = now - mag_time;
                    if (mag_time != 0u && elapsed >= 30000u && elapsed <= 75000u) {
                        magnetic_turn = -wrapped_delta(heading, last_heading) * 1000 / (int)elapsed;
                    }
                    mag_time = now;
                    last_heading = heading;
                }
            }
        }

        next.magnetic_rate = mag_time != 0u && now - mag_time < 85000u &&
                             absolute(magnetic_turn - yaw) <= 42;
        int selected_yaw = next.magnetic_rate ? (magnetic_turn + yaw) / 2 : yaw;
        if (selected_yaw > 511) { selected_yaw = 511; }
        if (selected_yaw < -512) { selected_yaw = -512; }
        next.yaw_deg_s = selected_yaw;
        next.max_hump_mm = hump.max_mm;
        if (norm_sq > 4200000 || norm_sq < 110000) {
            if (impacts < 2u) { ++impacts; }
        } else {
            impacts = 0u;
        }
        if (impacts >= 2u || up < 280 || absolute(left) > 850) {
            next.event = TERRAIN_IMPACT;
            car_fault(FAULT_COLLISION);
        } else if (absolute(slope) > 26000 || absolute(left - bias[1] / (int)samples) > 420) {
            next.event = TERRAIN_TILT;
        } else if (hump.phase == 2u) {
            next.event = TERRAIN_DESCENT;
        } else if (hump.phase == 1u) {
            next.event = TERRAIN_CLIMB;
        } else if (absolute(yaw) > 28) {
            next.event = TERRAIN_TURNING;
        } else if (absolute(dynamic) > 65) {
            next.event = TERRAIN_ACCELERATING;
        } else {
            next.event = absolute(speed) > 18 ? TERRAIN_CRUISING : TERRAIN_STILL;
        }
        DI(key);
        shared = next;
        EI(key);
    }
}

bool terrain_start(void)
{
    timer = app_events();
    return timer > 0 && app_periodic(timer, 10u) && app_start((FP)terrain_task, 4, 1280);
}
