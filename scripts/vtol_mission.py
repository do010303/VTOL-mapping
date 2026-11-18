#!/usr/bin/env python3
"""Fly the survey: VTOL takeoff, transition, climb in a loiter to the survey altitude, N parallel lines
over the forest (north-south, alternating direction), descend in a loiter, VTOL landing at home.
Run against ./run_sitl.sh.

Positions are local ENU metres from the world origin (forest centre); home is the spawn point. Each line runs
over --line-y plus --run-in at both ends, so the vehicle is settled on the line before the measured part.

  scripts/vtol_mission.py                                  # 500 m, one line x = 0, y -650..650, land
  scripts/vtol_mission.py --alt 600 --lines 10 --spacing 95  # spec 7: X = -427.5 + 95 k
  scripts/vtol_mission.py --stop-after-lines               # exit after the last line (no landing wait)
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

R_EARTH = 6378137.0


def offset(lat0, lon0, east, north):
    return (lat0 + math.degrees(north / R_EARTH),
            lon0 + math.degrees(east / (R_EARTH * math.cos(math.radians(lat0)))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--alt", type=float, default=500, help="survey altitude above home, m")
    ap.add_argument("--takeoff-alt", type=float, default=60)
    ap.add_argument("--lines", type=int, default=1)
    ap.add_argument("--spacing", type=float, default=95.0, help="between lines, m")
    ap.add_argument("--x0", type=float, help="x of the first line (default: the lines centred on x = 0)")
    ap.add_argument("--line-y", type=float, nargs=2, default=(-650, 650), help="measured part of each line")
    ap.add_argument("--run-in", type=float, default=350.0,
                    help="extra length at both ends of each line, m (R4: after a 95 m U-turn at 25 m/s the track "
                         "settles within ~400 m)")
    ap.add_argument("--land-descent-alt", type=float, default=100, help="loiter down to this before landing, m")
    ap.add_argument("--climb", type=float, nargs=2, default=(0, -1500), help="loiter-climb centre x y")
    ap.add_argument("--spawn", type=float, nargs=2, default=(0, -1900), help="home x y (world ENU)")
    ap.add_argument("--stop-after-lines", "--stop-after-line", action="store_true",
                    help="exit once the last line is flown (PX4 still descends and lands)")
    ap.add_argument("--timeout", type=float, default=3600, help="wall s")
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.url, source_system=245)
    m.wait_heartbeat(timeout=60)
    print(f"connected to system {m.target_system}", flush=True)

    # Wait for a home position (EKF2 with GPS).
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                            0, mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION, 1e6, 0, 0, 0, 0, 0)
    home = None
    t0 = time.time()
    while home is None and time.time() - t0 < 180:
        msg = m.recv_match(type="HOME_POSITION", blocking=True, timeout=2)
        if msg:
            home = (msg.latitude * 1e-7, msg.longitude * 1e-7)
    if home is None:
        sys.exit("no home position")
    # Origin of the world from home and the spawn offset.
    lat0, lon0 = offset(home[0], home[1], -args.spawn[0], -args.spawn[1])

    def item(seq, cmd, x, y, alt, p1=0, p2=0, p3=0, p4=0.0,
             frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT):
        lat, lon = offset(lat0, lon0, x, y)
        return m.mav.mission_item_int_encode(m.target_system, m.target_component, seq, frame, cmd, 0, 1,
                                             p1, p2, p3, p4, int(lat * 1e7), int(lon * 1e7), alt,
                                             mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    sx, sy = args.spawn
    y0, y1 = args.line_y
    x0 = args.x0 if args.x0 is not None else -(args.lines - 1) / 2 * args.spacing
    cmd = mavutil.mavlink
    items = [item(0, cmd.MAV_CMD_NAV_VTOL_TAKEOFF, sx, sy + 200, args.takeoff_alt, p4=float("nan")),
             item(1, cmd.MAV_CMD_NAV_LOITER_TO_ALT, args.climb[0], args.climb[1], args.alt, 0, 150)]
    lines = []
    for k in range(args.lines):
        x = x0 + k * args.spacing
        a, b = (y0 - args.run_in, y1 + args.run_in) if k % 2 == 0 else (y1 + args.run_in, y0 - args.run_in)
        lines.append((x, a, b))
        items.append(item(len(items), cmd.MAV_CMD_NAV_WAYPOINT, x, a, args.alt))
        items.append(item(len(items), cmd.MAV_CMD_NAV_WAYPOINT, x, b, args.alt))
    last_line_end = len(items) - 1
    items.append(item(len(items), cmd.MAV_CMD_NAV_LOITER_TO_ALT, args.climb[0], args.climb[1], args.land_descent_alt, 0, 150))
    items.append(item(len(items), cmd.MAV_CMD_NAV_VTOL_LAND, sx, sy, 0, p4=float("nan")))
    land_seq = len(items) - 1
    print("lines: " + ", ".join(f"x {x:+.1f} y {a:+.0f}->{b:+.0f}" for x, a, b in lines), flush=True)
    m.mav.mission_count_send(m.target_system, m.target_component, len(items), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    while True:
        req = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=10)
        if req is None:
            sys.exit("mission upload timed out")
        if req.get_type() == "MISSION_ACK":
            if req.type != 0:
                sys.exit(f"mission rejected: {req.type}")
            break
        m.mav.send(items[req.seq])
    print(f"mission uploaded: {len(items)} items, {args.lines} line(s) at {args.alt} m", flush=True)

    # Arm and start (retry while the pre-flight checks settle).
    for _ in range(60):
        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=2)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0:
            break
        time.sleep(2)
    else:
        sys.exit("could not arm")
    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_MISSION_START,
                            0, 0, len(items) - 1, 0, 0, 0, 0, 0)
    print("armed, mission started", flush=True)

    t0 = time.time()
    last = 0
    seq = -1
    started = False         # MISSION_CURRENT may first repeat the previous mission's index (kept in dataman)
    while time.time() - t0 < args.timeout:
        msg = m.recv_match(type=["MISSION_CURRENT", "VFR_HUD", "EXTENDED_SYS_STATE", "STATUSTEXT", "HEARTBEAT"],
                           blocking=True, timeout=2)
        if msg is None:
            continue
        typ = msg.get_type()
        if typ == "MISSION_CURRENT" and not started:
            started = msg.seq == 0
            if not started:
                continue
        if typ == "MISSION_CURRENT" and msg.seq != seq:
            seq = msg.seq
            print(f"{time.time() - t0:6.0f} s  mission item {seq}", flush=True)
            if 2 <= seq <= last_line_end + 1 and seq % 2 == 1:
                print(f"{time.time() - t0:6.0f} s  on line {(seq - 2) // 2 + 1}/{args.lines}", flush=True)
            if seq > last_line_end:
                print(f"{time.time() - t0:6.0f} s  all lines flown", flush=True)
                if args.stop_after_lines:
                    return
        elif typ == "STATUSTEXT":
            print(f"{time.time() - t0:6.0f} s  PX4: {msg.text}", flush=True)
        elif typ == "VFR_HUD" and time.time() - last > 10:
            last = time.time()
            print(f"{time.time() - t0:6.0f} s  alt {msg.alt:6.1f} m  airspeed {msg.airspeed:5.1f} m/s  "
                  f"groundspeed {msg.groundspeed:5.1f} m/s  heading {msg.heading:3d}", flush=True)
        elif typ == "HEARTBEAT" and seq >= land_seq and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("landed and disarmed", flush=True)
            return
    sys.exit("timeout")


if __name__ == "__main__":
    main()
