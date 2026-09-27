#!/usr/bin/env python3
"""Pre-flight check for the real drone (props OFF).

Connects to the Pixhawk and checks what must be right before the hop test
or any autonomous flight: link, firmware, disarmed, GPS, sensor health,
EKF, battery, RC receiver, and the safety parameters (geofence, RC-loss
and battery failsafes, RTL altitude, a manual mode on the mode switch).
Also prints the Pixhawk's own PreArm messages.

    python3 resq_mavlink/preflight.py                       # via ./start.sh real (UDP 14552)
    python3 resq_mavlink/preflight.py --url /dev/ttyUSB0 --baud 57600   # radio directly
    python3 resq_mavlink/preflight.py --url tcp:127.0.0.1:5762          # simulator

Exit code 0 when nothing FAILs.
"""
import argparse
import sys
import time

from pymavlink import mavutil

# Flight modes (ArduCopter numbers) a pilot can take over with.
MANUAL_MODES = {0: 'STABILIZE', 2: 'ALT_HOLD', 5: 'LOITER', 16: 'POSHOLD'}
MODE_NAMES = {0: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 9: 'LAND',
              16: 'POSHOLD', 17: 'BRAKE', 21: 'SMART_RTL'}
PARAMS = ['FENCE_ENABLE', 'FENCE_TYPE', 'FENCE_ALT_MAX', 'FENCE_RADIUS', 'FENCE_ACTION', 'RTL_ALT',
          'FS_THR_ENABLE', 'BATT_MONITOR', 'BATT_LOW_VOLT', 'BATT_FS_LOW_ACT', 'BATT_CRT_VOLT', 'BATT_FS_CRT_ACT',
          'ARMING_CHECK', 'FLTMODE_CH'] + [f'FLTMODE{i}' for i in range(1, 7)]

results = []


def report(status, what, detail, fix=''):
    results.append(status)
    colour = {'PASS': '\033[32m', 'WARN': '\033[33m', 'FAIL': '\033[31m'}[status]
    print(f"  {colour}{status}\033[0m  {what:<22} {detail}" + (f"\n        -> {fix}" if fix else ''))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--url', default='udp:127.0.0.1:14552')
    ap.add_argument('--baud', type=int, default=57600)
    ap.add_argument('--hop-alt', type=float, default=20.0, help='planned hop-test altitude (m)')
    args = ap.parse_args()

    print(f"Connecting to {args.url} ...")
    m = mavutil.mavlink_connection(args.url, baud=args.baud)
    hb, t_wait = None, time.time()
    while time.time() - t_wait < 15:
        msg = m.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        # Other components (MAVProxy, companion computers) send heartbeats
        # too; the autopilot's has a real autopilot type.
        if msg and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            hb = msg
            m.target_system, m.target_component = msg.get_srcSystem(), msg.get_srcComponent()
            break
    if hb is None:
        report('FAIL', 'Link', 'no heartbeat in 15 s',
               'check the radio pair (both LEDs solid), baud rate, and that ./start.sh real is running')
        return 1
    report('PASS', 'Link', f'heartbeat from system {m.target_system}')
    for rate_msg in (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                     mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS):
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                0, rate_msg, 500000, 0, 0, 0, 0, 0)
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                            0, mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
    for name in PARAMS:
        m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)

    seen, params, statustext = {}, {}, []
    t0 = time.time()
    while time.time() - t0 < 8.0:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'PARAM_VALUE':
            params[msg.param_id.strip('\x00') if isinstance(msg.param_id, str) else msg.param_id] = msg.param_value
        elif t == 'STATUSTEXT':
            statustext.append(msg.text)
        elif t == 'HEARTBEAT' and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue
        else:
            seen[t] = msg

    hb = seen.get('HEARTBEAT', hb)
    if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        v = seen.get('AUTOPILOT_VERSION')
        ver = f"{(v.flight_sw_version >> 24) & 255}.{(v.flight_sw_version >> 16) & 255}.{(v.flight_sw_version >> 8) & 255}" if v else '?'
        report('PASS', 'Firmware', f'ArduPilot {ver}, vehicle type {hb.type}')
    else:
        report('FAIL', 'Firmware', f'autopilot {hb.autopilot} is not ArduPilot', 'flash ArduCopter (Pixhawk1-1M build for 2.4.8 clones)')
    armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    report('FAIL' if armed else 'PASS', 'Disarmed', 'ARMED - disarm before checks' if armed else 'yes')

    gps = seen.get('GPS_RAW_INT')
    if gps is None:
        report('FAIL', 'GPS', 'no GPS data', 'check the GPS cable and GPS_TYPE')
    else:
        hdop = gps.eph / 100.0 if gps.eph != 65535 else 99
        ok = gps.fix_type >= 3 and gps.satellites_visible >= 8 and hdop < 1.5
        report('PASS' if ok else 'FAIL', 'GPS', f'fix type {gps.fix_type} (3 = 3D), {gps.satellites_visible} satellites, HDOP {hdop:.1f}',
               '' if ok else 'wait outdoors with a clear sky until 3D fix, 8+ satellites, HDOP < 1.5')

    st = seen.get('SYS_STATUS')
    if st:
        bad = st.onboard_control_sensors_enabled & ~st.onboard_control_sensors_health & st.onboard_control_sensors_present
        unhealthy = [e.name.replace('MAV_SYS_STATUS_SENSOR_', '') for k, e in mavutil.mavlink.enums['MAV_SYS_STATUS_SENSOR'].items()
                     if isinstance(k, int) and k and bad & k]
        report('PASS' if not unhealthy else 'FAIL', 'Sensors', 'all enabled sensors healthy' if not unhealthy else 'UNHEALTHY: ' + ', '.join(unhealthy),
               '' if not unhealthy else 'calibrate in Mission Planner (accel, compass) and check wiring')
        volts = st.voltage_battery / 1000.0
        pct = st.battery_remaining
        cells = round(volts / 3.9) if volts > 3 else 0
        per_cell = volts / cells if cells else 0
        ok = per_cell >= 3.8
        report('PASS' if ok else ('WARN' if per_cell >= 3.6 else 'FAIL'), 'Battery',
               f'{volts:.2f} V (~{cells}S, {per_cell:.2f} V/cell), {pct}% reported',
               '' if ok else 'charge the battery (start flights at 4.0+ V/cell)')
    else:
        report('FAIL', 'Sensors / battery', 'no SYS_STATUS received')

    ekf = seen.get('EKF_STATUS_REPORT')
    if ekf:
        need = (mavutil.mavlink.EKF_ATTITUDE | mavutil.mavlink.EKF_VELOCITY_HORIZ | mavutil.mavlink.EKF_POS_HORIZ_ABS
                | mavutil.mavlink.EKF_POS_VERT_ABS)
        ok = (ekf.flags & need) == need
        report('PASS' if ok else 'FAIL', 'EKF', 'position and attitude good' if ok else f'flags 0x{ekf.flags:x}: not ready',
               '' if ok else 'wait for GPS lock and EKF to settle (a minute after boot)')
    else:
        report('WARN', 'EKF', 'no EKF status received')

    rc = seen.get('RC_CHANNELS')
    if rc and rc.chancount >= 5:
        report('PASS', 'RC receiver', f'{rc.chancount} channels, throttle {rc.chan3_raw} us')
    else:
        report('FAIL', 'RC receiver', 'no RC input', 'turn on the transmitter and check the receiver is bound: it is your override')

    def p(name, default=None):
        return params.get(name, default)
    fence = p('FENCE_ENABLE')
    alt_max = p('FENCE_ALT_MAX')
    if fence == 1 and alt_max is not None and args.hop_alt + 5 <= alt_max <= 60:
        report('PASS', 'Geofence', f'on, max altitude {alt_max:.0f} m, radius {p("FENCE_RADIUS", 0):.0f} m')
    else:
        report('FAIL', 'Geofence', f'FENCE_ENABLE={fence}, FENCE_ALT_MAX={alt_max}',
               f'set FENCE_ENABLE=1, FENCE_TYPE=3 (altitude+circle), FENCE_ALT_MAX={args.hop_alt + 10:.0f}, FENCE_RADIUS=50, FENCE_ACTION=1 (RTL)')
    rtl = p('RTL_ALT')
    if rtl is not None:
        report('PASS' if 1000 <= rtl <= 3000 else 'WARN', 'RTL altitude', f'{rtl / 100:.0f} m',
               '' if 1000 <= rtl <= 3000 else 'set RTL_ALT between 1000 and 3000 (cm)')
    else:
        report('WARN', 'RTL altitude', 'RTL_ALT not found (renamed in newer firmware?)',
               'check the RTL altitude in Mission Planner: 10-30 m')
    fs = p('FS_THR_ENABLE')
    report('PASS' if fs and fs >= 1 else 'FAIL', 'RC-loss failsafe', f'FS_THR_ENABLE={fs}',
           '' if fs and fs >= 1 else 'set FS_THR_ENABLE=1 (RTL on RC loss) and calibrate the throttle failsafe PWM')
    bfs = p('BATT_FS_LOW_ACT')
    ok = p('BATT_MONITOR', 0) and bfs and bfs >= 1 and p('BATT_LOW_VOLT', 0) > 0
    report('PASS' if ok else 'FAIL', 'Battery failsafe',
           f'monitor {p("BATT_MONITOR")}, low {p("BATT_LOW_VOLT")} V -> action {bfs}',
           '' if ok else 'set BATT_MONITOR (4 for the power module), BATT_LOW_VOLT (e.g. 3.6 V x cells), BATT_FS_LOW_ACT=2 (RTL)')
    ac = p('ARMING_CHECK')
    if ac is None:
        report('WARN', 'Arming checks', 'ARMING_CHECK not found (newer firmware: ARMING_SKIPCHK)',
               'make sure no arming checks are skipped')
    else:
        report('PASS' if ac == 1 else 'WARN', 'Arming checks', f'ARMING_CHECK={ac:.0f}',
               '' if ac == 1 else 'set ARMING_CHECK=1 (all checks)')
    modes = [int(p(f'FLTMODE{i}', -1)) for i in range(1, 7)]
    manual = sorted({MANUAL_MODES[x] for x in modes if x in MANUAL_MODES})
    report('PASS' if manual else 'FAIL', 'Takeover mode',
           'mode switch has ' + (', '.join(manual) if manual else 'no manual mode') +
           f" (modes: {', '.join(MODE_NAMES.get(x, str(x)) for x in modes)})",
           '' if manual else 'put LOITER (and STABILIZE) on the mode switch: flipping to it takes over from the bridge')

    prearm = sorted({t for t in statustext if 'PreArm' in t or 'Arm' in t})
    if prearm:
        report('FAIL', 'Pixhawk PreArm', f'{len(prearm)} message(s):')
        for t in prearm:
            print(f"          {t}")
    else:
        report('PASS', 'Pixhawk PreArm', 'no PreArm complaints')

    fails = results.count('FAIL')
    print(f"\n{'READY for the hop test (props on, open field, you on the RC).' if not fails else f'NOT READY: {fails} FAIL(s) above.'}")
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
