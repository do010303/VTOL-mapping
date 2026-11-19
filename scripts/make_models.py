#!/usr/bin/env python3
"""Sensor payload, VTOL models and worlds for the VTOL LiDAR + camera simulation.

One place for the sensor parameters (spec sections 2, 3 and 5), so the static
rig, the flyable PX4 VTOL and the processing scripts all use the same values.
Writes:

  models/vtol_rig/                static VTOL-sized body (CW-30E: 2.4 m long, 4.4 m span) + payload, no physics
  models/standard_vtol_mapping/   PX4's standard_vtol (airframe 4004) + the same payload under its belly
  models/cw30e_mapping/           CW-30E-sized quadplane (4.4 m, 38 kg) derived from standard_vtol + the payload;
                                  PX4 airframe 4004 with config/cw30e_px4_params.json
  worlds/forest_static.sdf        forest + vtol_rig at --height above the ground
  worlds/forest_slope*.sdf        the sloping forest (make_forest.py --terrain slope --name forest_slope)
  worlds/forest_1400.sdf          the 1.4 km phase 1/2 forest (R1-R6 data)
  worlds/flat_static.sdf          ground only + vtol_rig (range accuracy check, spec 9.1)
  worlds/forest.sdf               forest only; PX4 spawns standard_vtol_mapping into it
  config/sensors.json             every sensor parameter and mount transform, for the nodes and the report

Payload, centred under the belly (body frame FLU: x forward, y left, z up):
  camera  Sony ILX-LR1 + 35 mm, nadir, long image side across track       at (0, 0, cam_z)
  LiDAR   RIEGL VUX-240-like line scanner, nadir, line across track       at (0, 0, lidar_z), below the camera

  python3 scripts/make_models.py                              # spec values: 4752 x 3168, +-20 deg, 1333 rays, 200 lines/s
  python3 scripts/make_models.py --max-range 1200             # the 300 kHz mode's range instead of 880 m (600 kHz)
  python3 scripts/make_models.py --image full                 # 9504 x 6336
"""

import argparse
import json
import math
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PX4_DIR = os.path.abspath(os.path.join(ROOT, "..", ".."))

# Sensor 35.7 x 23.8 mm behind a 35 mm lens (spec 3).
SENSOR_W_MM, FOCAL_MM = 35.7, 35.0
IMAGES = {"full": (9504, 6336), "half": (4752, 3168), "quarter": (2376, 1584), "eighth": (1188, 792)}

WORLD_PLUGINS = """    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster">
      <!-- The vehicle's pose stream (dynamic_pose/info): 500 Hz instead of 60, spec 4 asks for >= 200 Hz. -->
      <dynamic_pose_hertz>500</dynamic_pose_hertz>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>"""
# Deliberately not PX4's server.config: its GstCameraSystem would stream the 15 Mpx camera over UDP (and with
# the NVIDIA 595 driver its nvh264enc pipeline never plays, so the frames pile up until the server is OOM-killed).


def sensor_config(args):
    w, h = IMAGES[args.image]
    fx = FOCAL_MM / SENSOR_W_MM * w
    hfov = 2 * math.atan(w / 2 / fx)
    half = math.radians(args.scan_half_deg)
    return {
        "frames": {
            "world": "ENU, x east, y north, z up, metres",
            "body": "FLU, x forward, y left, z up (gz link frame)",
            "lidar": "beam d_L = (0, sin(beta), -cos(beta)), beta > 0 to the left; gz scan angle = beta",
            "camera_optical": "x image right, y image down, z optical axis",
            "quaternion_order": "qx, qy, qz, qw",
        },
        "camera": {
            "model": "Sony ILX-LR1 + SEL35F28Z (virtual pinhole)",
            "topic": "/vtol/camera/image", "info_topic": "/vtol/camera/camera_info",
            "trigger_topic": "/vtol/camera/trigger", "triggered": args.camera_triggered,
            "width": w, "height": h, "fx": fx, "fy": fx, "cx": (w - 1) / 2, "cy": (h - 1) / 2,
            "hfov_deg": math.degrees(hfov), "vfov_deg": math.degrees(2 * math.atan(h / 2 / fx)),
            "distortion": [0, 0, 0, 0, 0], "update_rate_hz": args.camera_rate,
            "trigger_spacing_m": 60.0, "clip_far_m": 4000.0,
            "t_BC": [0.0, 0.0, args.cam_z],
            "R_BC": [[0, -1, 0], [-1, 0, 0], [0, 0, -1]],
            "gz_pose_rpy": [0, math.pi / 2, 0],
        },
        "lidar": {
            "model": "RIEGL VUX-240^24-like line scanner (ideal geometric rays)",
            "topic": "/vtol/lidar/scan",
            "scan_min_deg": -args.scan_half_deg, "scan_max_deg": args.scan_half_deg,
            "rays_per_line": args.rays, "lines_per_s": args.line_rate,
            "angle_step_deg": 2 * args.scan_half_deg / (args.rays - 1),
            "range_min_m": 5.0, "range_max_m": args.max_range,
            "prr_hz_reference": 600000 if args.max_range <= 880 else 300000,
            "beam_divergence_mrad": 0.35, "noise": 0,
            "pulse_spacing_s": 1 / 600000,
            "note": "gz renders a whole scan line at one instant (t_s of the line). The ideal schedule, ray k emitted at "
                    "t_s + k * pulse_spacing_s (a 2.22 ms window in the 5 ms line period), each ray with the pose at its own "
                    "time, is applied by scripts/lidar_exact.py (ray casting on the scene mesh).",
            "t_BL": [0.0, 0.0, args.lidar_z], "R_BL": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "gz_pose_rpy": [0, math.pi / 2, 0],
            "half_angle_rad": half,
        },
        "rig_height_m": args.height,
        "image_preset": args.image,
    }


def payload_sdf(cfg, link_pose_z=0.0):
    """Camera + LiDAR sensors and their housings, for a link whose frame is the body frame."""
    c, l = cfg["camera"], cfg["lidar"]
    trig = ""
    if c["triggered"]:
        trig = f"""
          <triggered>true</triggered>
          <trigger_topic>{c['trigger_topic']}</trigger_topic>"""
    lo, hi = math.radians(l["scan_min_deg"]), math.radians(l["scan_max_deg"])
    cz, lz = c["t_BC"][2] - link_pose_z, l["t_BL"][2] - link_pose_z
    return f"""
      <!-- Camera body 100 x 74 x 42.5 mm and lens 61.5 x 36.5 mm (spec 3), lens down. -->
      <visual name="camera_body">
        <pose>0 0 {cz + 0.058:.4f} 0 0 1.5708</pose>
        <geometry><box><size>0.100 0.074 0.0425</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <visual name="camera_lens">
        <pose>0 0 {cz + 0.018:.4f} 0 0 0</pose>
        <geometry><cylinder><radius>0.03075</radius><length>0.0365</length></cylinder></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material>
      </visual>
      <!-- Scanner 292 x 162 x 185 mm (spec 2), long side along the track. -->
      <visual name="lidar_body">
        <pose>0 0 {lz + 0.0925:.4f} 0 0 0</pose>
        <geometry><box><size>0.292 0.162 0.185</size></box></geometry>
        <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material>
      </visual>
      <sensor name="nadir_camera" type="camera">
        <pose>0 0 {cz:.4f} 0 1.5707963 0</pose>
        <always_on>1</always_on>
        <update_rate>{c['update_rate_hz']}</update_rate>
        <topic>{c['topic']}</topic>
        <camera name="ilx_lr1">
          <camera_info_topic>{c['info_topic']}</camera_info_topic>{trig}
          <horizontal_fov>{math.radians(c['hfov_deg']):.7f}</horizontal_fov>
          <image><width>{c['width']}</width><height>{c['height']}</height><format>R8G8B8</format></image>
          <clip><near>1</near><far>{c['clip_far_m']}</far></clip>
          <lens>
            <intrinsics><fx>{c['fx']:.3f}</fx><fy>{c['fy']:.3f}</fy><cx>{c['cx']}</cx><cy>{c['cy']}</cy><s>0</s></intrinsics>
          </lens>
        </camera>
      </sensor>
      <!-- Preview: same mount and field of view as the mapping camera, 1/6 of its resolution, free-running at
           10 Hz, for watching the camera view live (Gazebo GUI, scripts/camera_view.py). Not for mapping. -->
      <sensor name="nadir_camera_preview" type="camera">
        <pose>0 0 {cz:.4f} 0 1.5707963 0</pose>
        <always_on>1</always_on>
        <update_rate>10</update_rate>
        <topic>/vtol/camera/preview</topic>
        <camera name="preview">
          <!-- Its own info topic: the default (<topic dir>/camera_info) is the mapping camera's. -->
          <camera_info_topic>/vtol/camera/preview/camera_info</camera_info_topic>
          <horizontal_fov>{math.radians(c['hfov_deg']):.7f}</horizontal_fov>
          <image><width>{c['width'] // 6}</width><height>{c['height'] // 6}</height><format>R8G8B8</format></image>
          <clip><near>1</near><far>{c['clip_far_m']}</far></clip>
        </camera>
      </sensor>
      <sensor name="nadir_lidar" type="gpu_lidar">
        <!-- Pitched 90 deg: the scan plane is the body y-z plane, the zero angle points down,
             positive angles to the left. -->
        <pose>0 0 {lz:.4f} 0 1.5707963 0</pose>
        <always_on>1</always_on>
        <update_rate>{l['lines_per_s']}</update_rate>
        <topic>{l['topic']}</topic>
        <lidar>
          <scan>
            <horizontal><samples>{l['rays_per_line']}</samples><resolution>1</resolution>
              <min_angle>{lo:.7f}</min_angle><max_angle>{hi:.7f}</max_angle></horizontal>
            <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical>
          </scan>
          <range><min>{l['range_min_m']}</min><max>{l['range_max_m']}</max><resolution>0.001</resolution></range>
        </lidar>
        <visualize>false</visualize>
      </sensor>"""


def rig_sdf(cfg):
    """Static body the size of a CW-30E (2.4 m x 4.4 m), sensors under the fuselage centre."""
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/make_models.py; do not edit. -->
<sdf version="1.10">
  <model name="vtol_rig">
    <static>true</static>
    <link name="base_link">
{cw30e_visuals()}{payload_sdf(cfg)}
    </link>
  </model>
</sdf>
"""


def moving_rig_sdf(cfg, speed):
    """The rig as a body moving at a constant velocity along its x axis (no gravity): its true position is
    known analytically (start + speed x t), for the timing check of scripts/sync_check.py."""
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/make_models.py; do not edit. -->
<sdf version="1.10">
  <model name="vtol_rig_moving">
    <link name="base_link">
      <gravity>false</gravity>
      <inertial><mass>10</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <visual name="fuselage">
        <geometry><box><size>2.4 0.30 0.30</size></box></geometry>
      </visual>{payload_sdf(cfg)}
    </link>
    <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
      <initial_linear>{speed} 0 0</initial_linear>
    </plugin>
  </model>
</sdf>
"""


def sync_world_sdf(height, start_y):
    """Flat ground with a 20 m step whose edge is the line y = 0, and the moving rig flying north over it."""
    w = world_sdf("sync_test", False, None)
    extra = f"""
    <model name="step">
      <static>true</static>
      <pose>0 1500 10 0 0 0</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>3000 3000 20</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
      </link>
    </model>
    <!-- A 10 m step whose south edge is slanted (y = -100 + 0.1 x): each LiDAR ray reaches it at its own time,
         1.1 ms apart, so the lines' timing is resolved to about 1 ms (scripts/sync_check.py). -->
    <model name="slant">
      <static>true</static>
      <pose>{-40 * math.sin(math.atan(0.1)):.4f} {-100 + 40 * math.cos(math.atan(0.1)):.4f} 5 0 0 {math.atan(0.1):.7f}</pose>
      <link name="link">
        <visual name="v">
          <geometry><box><size>1000 80 10</size></box></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>
    <include>
      <uri>model://vtol_rig_moving</uri>
      <name>vtol_rig_moving</name>
      <pose>0 {start_y} {height} 0 0 1.5707963</pose>
    </include>
  </world>"""
    return w.replace("\n  </world>", extra, 1)


def vtol_sdf(cfg):
    """PX4's standard_vtol with the payload as a link fixed under base_link."""
    src = open(os.path.join(PX4_DIR, "Tools/simulation/gz/models/standard_vtol/model.sdf")).read()
    src = src.replace("<model name='standard_vtol'>", "<model name='standard_vtol_mapping'>", 1)
    src = re.sub(r"<sdf version=['\"][0-9.]+['\"]>", '<sdf version="1.10">', src, count=1)
    # The payload goes into base_link itself: a separate near-massless link on a fixed joint made ODE
    # blow up (the parked vehicle was flung off and fell through the ground). Visuals and sensors only, so
    # the standard_vtol dynamics PX4 is tuned for are unchanged.
    start = src.index("<link name='base_link'>")
    end = src.index("</link>", start)
    src = src[:end] + "  <!-- Mapping payload (scripts/make_models.py): camera above, LiDAR below, under the belly. -->" + \
        payload_sdf(cfg) + "\n    " + src[end:]
    j = src.index("?>") + 2 if src.lstrip().startswith("<?xml") else 0
    return src[:j] + "\n<!-- Generated by scripts/make_models.py from PX4's standard_vtol; do not edit. -->" + src[j:]


# ------------------------------------------------------------------------------------------------- CW-30E-like VTOL
#
# Spec 4: JOUAV CW-30E, a quadplane (4 lift rotors + pusher), 2.4 m long, 4.4 m span, 38 kg MTOW (8 kg payload),
# cruise 25 m/s. JOUAV publishes no aerodynamic or inertia data, so this is PX4's standard_vtol (the same
# 4 + 1 layout, same plugins, same actuator mapping as airframe 4004) re-dimensioned: geometry scaled to the
# CW-30E's span, MTOW mass including the payload, inertia from a simple mass breakdown, wing area and control
# surfaces sized so that the angular acceleration per radian of deflection at 25 m/s matches standard_vtol's at
# its 17 m/s trim (the gains PX4 ships for 4004 then still fit), lift rotors with thrust-to-weight 2.5.
CW30E = {
    "span_m": 4.4, "length_m": 2.4, "mass_kg": 38.0,
    # Mass breakdown for the inertia: wings 6 kg (uniform over the span), 4 lift motors + props 1.5 kg each at the
    # rotor positions, fuselage + payload + energy 24 kg (2.4 m, concentrated around the CG), tail 2 kg at -1.3 m.
    "inertia": {"ixx": 13.0, "iyy": 11.0, "izz": 22.0},
    "rotor_xy_m": 0.72,                  # standard_vtol 0.35 x 4.4 / 2.144
    "rotor_max_thrust_n": 233.0,         # per lift rotor: 4 x 233 N = 2.5 x 38 kg x g
    "rotor_max_rad_s": 1500.0,           # as standard_vtol (PX4: SIM_GZ_EC_MAX1..4 1500)
    "pusher_max_thrust_n": 250.0,        # T/W 0.67: 25 m/s cruise (~60 N drag) and 5 m/s climb with margin
    "pusher_max_rad_s": 3500.0,          # as standard_vtol (SIM_GZ_EC_MAX5 3500)
    "wing_area_m2": 2.2,                 # 4.4 m x 0.5 m chord, aspect ratio 8.8 (both halves)
    "wing_cp_x_m": -0.12,
    # Elevons: -2.8 (matching standard_vtol's roll acceleration per radian at 25 m/s) over-reacted in the first
    # flight (roll rate 58-81 deg/s for 35 deg/s asked, the long wing's damping is relatively weaker): halved.
    "elevon_rad_to_cl": -1.4,
    # Horizontal tail: volume coefficient ~0.37 (S_t l_t / (S c) = 0.4 x 1.03 / (2.2 x 0.5)), as real aircraft
    # (0.3-0.6); standard_vtol's 0.01 m^2 scaled up (0.072 m^2) left pitch badly damped: phugoid +-8 m/s, the
    # airspeed check tripped. Incidence a0 trims the wing's nose-down moment at 25 m/s; elevator authority
    # (area x rad_to_cl) as before.
    "tail_area_m2": 0.4, "tail_cp_x_m": -1.03, "tail_a0": -0.1, "elevator_rad_to_cl": -2.2,
    "fin_area_m2": 0.15, "fin_cp_x_m": -1.25,
    "gear_z_m": -0.56,                   # bottom of the skids; the LiDAR housing ends at -0.42 m
}


def cw30e_visuals(cfg_vehicle=CW30E):
    """Simple CW-30E-sized body (spec 4: a simple 3D model to length and span), body frame FLU."""
    r = cfg_vehicle["rotor_xy_m"]
    grey = "<material><ambient>0.82 0.84 0.86 1</ambient><diffuse>0.82 0.84 0.86 1</diffuse></material>"
    dark = "<material><ambient>0.15 0.15 0.15 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>"
    v = f"""
      <visual name="fuselage"><pose>0.1 0 0 0 0 0</pose>
        <geometry><box><size>1.6 0.32 0.30</size></box></geometry>{grey}</visual>
      <visual name="tail_boom"><pose>-0.95 0 0.05 0 0 0</pose>
        <geometry><box><size>0.9 0.08 0.08</size></box></geometry>{grey}</visual>
      <visual name="wing"><pose>0.05 0 0.15 0 0 0</pose>
        <geometry><box><size>0.5 {cfg_vehicle['span_m']} 0.05</size></box></geometry>{grey}</visual>
      <visual name="tail"><pose>-1.25 0 0.10 0 0 0</pose>
        <geometry><box><size>0.3 1.2 0.04</size></box></geometry>{grey}</visual>
      <visual name="fin"><pose>-1.25 0 0.30 0 0 0</pose>
        <geometry><box><size>0.3 0.04 0.4</size></box></geometry>{grey}</visual>"""
    for side, y in (("left", r), ("right", -r)):
        v += f"""
      <visual name="boom_{side}"><pose>0 {y} 0.12 0 0 0</pose>
        <geometry><box><size>{2 * r + 0.4:.2f} 0.06 0.06</size></box></geometry>{dark}</visual>
      <visual name="skid_{side}"><pose>0 {0.45 if y > 0 else -0.45} {cfg_vehicle['gear_z_m'] + 0.025} 0 0 0</pose>
        <geometry><box><size>1.2 0.05 0.05</size></box></geometry>{dark}</visual>"""
    return v


def cw30e_sdf(cfg, name="cw30e_mapping"):
    """Flyable CW-30E-sized quadplane for PX4 airframe 4004 (+ parameters in run_sitl.sh), payload in base_link."""
    v = CW30E
    r = v["rotor_xy_m"]
    k_lift = v["rotor_max_thrust_n"] / v["rotor_max_rad_s"] ** 2
    k_push = v["pusher_max_thrust_n"] / v["pusher_max_rad_s"] ** 2
    half = v["wing_area_m2"] / 2
    gz = v["gear_z_m"]
    contact = """<surface><contact><ode><kp>1000000</kp><kd>100</kd><max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode></contact>
          <friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>"""

    def tiny_link(name, pose):
        return f"""
    <link name="{name}">
      <pose>{pose}</pose>
      <inertial><mass>0.005</mass><inertia><ixx>9.75e-07</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.000166704</iyy><iyz>0</iyz><izz>0.000167604</izz></inertia></inertial>"""

    rotors = ""
    # Same order, positions (scaled) and spin directions as standard_vtol: 0 front right ccw, 1 rear left ccw,
    # 2 front left cw, 3 rear right cw.
    for i, (x, y) in enumerate(((r, -r), (-r, r), (r, r), (-r, -r))):
        rotors += tiny_link(f"rotor_{i}", f"{x} {y} 0.16 0 0 0") + f"""
      <visual name="rotor_{i}_visual">
        <geometry><cylinder><length>0.01</length><radius>0.38</radius></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 0.5</ambient><diffuse>0.1 0.1 0.1 0.5</diffuse></material>
        <transparency>0.6</transparency>
      </visual>
    </link>
    <joint name="rotor_{i}_joint" type="revolute">
      <child>rotor_{i}</child><parent>base_link</parent>
      <axis><xyz>0 0 1</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis>
    </joint>"""
    rotors += tiny_link("rotor_puller", "-0.75 0 0.05 0 1.57 0") + """
      <visual name="rotor_puller_visual">
        <geometry><cylinder><length>0.01</length><radius>0.28</radius></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 0.5</ambient><diffuse>0.1 0.1 0.1 0.5</diffuse></material>
        <transparency>0.6</transparency>
      </visual>
    </link>
    <joint name="rotor_puller_joint" type="revolute">
      <pose>0 0 0 0 -1.57 0</pose>
      <child>rotor_puller</child><parent>base_link</parent>
      <axis><xyz>1 0 0</xyz><limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis>
    </joint>"""

    surfaces = ""
    for n, joint_pose in (("left_elevon", f"-0.12 {1.2} 0.15 0 0 0"), ("right_elevon", f"-0.12 {-1.2} 0.15 0 0 0"),
                          ("elevator", f"{v['tail_cp_x_m']} 0 0.10 0 0 0")):
        surfaces += f"""
    <link name="{n}">
      <pose>{joint_pose}</pose>
      <inertial><mass>0.00000001</mass><inertia><ixx>0.000001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.000001</iyy><iyz>0</iyz><izz>0.000001</izz></inertia></inertial>
    </link>"""
    for j, child in enumerate(("left_elevon", "right_elevon", "elevator")):
        surfaces += f"""
    <joint name="servo_{j}" type="revolute">
      <parent>base_link</parent><child>{child}</child>
      <axis><xyz>0 1 0</xyz><limit><lower>-0.78</lower><upper>0.78</upper></limit><dynamics><damping>1.000</damping></dynamics></axis>
      <physics><ode><implicit_spring_damper>1</implicit_spring_damper></ode></physics>
    </joint>"""

    def lift_drag(a0, cp, area, joint, rad_to_cl):
        return f"""
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <a0>{a0}</a0><cla>4.752798721</cla><cda>0.6417112299</cda><cma>0.0</cma>
      <alpha_stall>0.3391428111</alpha_stall><cla_stall>-3.85</cla_stall><cda_stall>-0.9233984055</cda_stall><cma_stall>0</cma_stall>
      <cp>{cp}</cp><area>{area}</area><air_density>1.2041</air_density>
      <forward>1 0 0</forward><upward>0 0 1</upward><link_name>base_link</link_name>
      <control_joint_name>{joint}</control_joint_name><control_joint_rad_to_cl>{rad_to_cl}</control_joint_rad_to_cl>
    </plugin>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name><sub_topic>{joint}</sub_topic>
    </plugin>"""

    aero = (lift_drag(0.05984281113, f"{v['wing_cp_x_m']} {0.3 * 4.4 / 2.144:.3f} 0.15", half, "servo_0", v["elevon_rad_to_cl"]) +
            lift_drag(0.05984281113, f"{v['wing_cp_x_m']} {-0.3 * 4.4 / 2.144:.3f} 0.15", half, "servo_1", v["elevon_rad_to_cl"]) +
            lift_drag(v["tail_a0"], f"{v['tail_cp_x_m']} 0 0.10", v["tail_area_m2"], "servo_2", v["elevator_rad_to_cl"]) +
            f"""
    <!-- Vertical fin: directional (weathercock) stability; no rudder (yaw in FW flight by coordination). -->
    <plugin filename="gz-sim-lift-drag-system" name="gz::sim::systems::LiftDrag">
      <a0>0</a0><cla>4.752798721</cla><cda>0.6417112299</cda><cma>0.0</cma>
      <alpha_stall>0.3391428111</alpha_stall><cla_stall>-3.85</cla_stall><cda_stall>-0.9233984055</cda_stall><cma_stall>0</cma_stall>
      <cp>{v['fin_cp_x_m']} 0 0.30</cp><area>{v['fin_area_m2']}</area><air_density>1.2041</air_density>
      <forward>1 0 0</forward><upward>0 1 0</upward><link_name>base_link</link_name>
    </plugin>""")

    motors = ""
    for i, (joint, link, turn, k, wmax, km) in enumerate((
            ("rotor_0_joint", "rotor_0", "ccw", k_lift, v["rotor_max_rad_s"], 0.06),
            ("rotor_1_joint", "rotor_1", "ccw", k_lift, v["rotor_max_rad_s"], 0.06),
            ("rotor_2_joint", "rotor_2", "cw", k_lift, v["rotor_max_rad_s"], 0.06),
            ("rotor_3_joint", "rotor_3", "cw", k_lift, v["rotor_max_rad_s"], 0.06),
            ("rotor_puller_joint", "rotor_puller", "cw", k_push, v["pusher_max_rad_s"], 0.01))):
        motors += f"""
    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <jointName>{joint}</jointName><linkName>{link}</linkName><turningDirection>{turn}</turningDirection>
      <timeConstantUp>0.03</timeConstantUp><timeConstantDown>0.05</timeConstantDown>
      <maxRotVelocity>{wmax}</maxRotVelocity><motorConstant>{k:.6e}</motorConstant><momentConstant>{km}</momentConstant>
      <commandSubTopic>command/motor_speed</commandSubTopic><motorNumber>{i}</motorNumber>
      <rotorDragCoefficient>0.000106428</rotorDragCoefficient><rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <rotorVelocitySlowdownSim>20</rotorVelocitySlowdownSim><motorType>velocity</motorType>
    </plugin>"""

    # PX4's sensors (IMU, baro, mag, GPS) exactly as in standard_vtol's base_link.
    src = open(os.path.join(PX4_DIR, "Tools/simulation/gz/models/standard_vtol/model.sdf")).read()
    a = src.index('<sensor name="imu_sensor"')
    b = src.index("</link>", a)
    px4_sensors = src[a:b].rstrip()
    iner = v["inertia"]
    skids = "".join(f"""
      <collision name="skid_{s}">
        <pose>0 {y} {gz + 0.025} 0 0 0</pose>
        <geometry><box><size>1.2 0.05 0.05</size></box></geometry>
        {contact}
      </collision>""" for s, y in (("left", 0.45), ("right", -0.45)))
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/make_models.py (cw30e_sdf): a CW-30E-sized quadplane derived from PX4's standard_vtol.
     Mass {v['mass_kg']} kg incl. payload, span {v['span_m']} m, length {v['length_m']} m. Do not edit. -->
<sdf version="1.10">
  <model name="{name}">
    <pose>0 0 {-gz:.3f} 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <pose>0 0 0 0 0 0</pose>
        <mass>{v['mass_kg']}</mass>
        <inertia><ixx>{iner['ixx']}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iner['iyy']}</iyy><iyz>0</iyz><izz>{iner['izz']}</izz></inertia>
      </inertial>{skids}
      <collision name="fuselage_collision">
        <pose>0.1 0 0 0 0 0</pose>
        <geometry><box><size>1.6 0.32 0.30</size></box></geometry>
      </collision>{cw30e_visuals()}
      <!-- Mapping payload (scripts/make_models.py): camera above, LiDAR below, under the belly. -->{payload_sdf(cfg)}
      {px4_sensors}
    </link>{rotors}{surfaces}{aero}{motors}
    <static>0</static>
  </model>
</sdf>
"""


def cw30e_px4_params():
    """PX4 parameters for airframe 4004 flying the CW-30E-sized model (exported as PX4_PARAM_* by run_sitl.sh)."""
    r = CW30E["rotor_xy_m"]
    # Control allocation geometry (FRD: x forward, y right): rotor 0 front right, 1 rear left, 2 front left,
    # 3 rear right; standard_vtol's 4004 values do not match its own SDF either, here they do.
    p = {"CA_ROTOR0_PX": r, "CA_ROTOR0_PY": r, "CA_ROTOR1_PX": -r, "CA_ROTOR1_PY": -r,
         "CA_ROTOR2_PX": r, "CA_ROTOR2_PY": -r, "CA_ROTOR3_PX": -r, "CA_ROTOR3_PY": r,
         "CA_ROTOR4_PX": -0.75}
    # Hover: 38 kg on 4 x 233 N max, thrust ~ omega^2 -> sqrt(0.4) = 0.64 of full command.
    p |= {"MPC_THR_HOVER": 0.64, "THR_MDL_FAC": 0.0}
    # Rate loops: 4-5x less angular acceleration per unit command than standard_vtol (inertia x 30, arms x 2).
    p |= {"MC_ROLLRATE_P": 0.9, "MC_ROLLRATE_I": 0.6, "MC_ROLLRATE_D": 0.012, "MC_ROLLRATE_K": 1.0,
          "MC_PITCHRATE_P": 0.6, "MC_PITCHRATE_I": 0.6, "MC_PITCHRATE_D": 0.01,
          "MC_YAWRATE_P": 0.6, "MC_YAWRATE_I": 0.3, "MC_YAW_P": 1.2, "MC_YAWRATE_MAX": 30}
    # Fixed wing at 25 m/s (spec 4); stall ~12-15 m/s with 17 kg/m^2 wing loading.
    p |= {"FW_AIRSPD_STALL": 13, "FW_AIRSPD_MIN": 18, "FW_AIRSPD_TRIM": 25, "FW_AIRSPD_MAX": 30,
          "VT_ARSP_TRANS": 19, "VT_ARSP_BLEND": 15, "FW_THR_TRIM": 0.5, "FW_THR_MAX": 1.0, "FW_THR_MIN": 0.1,
          "FW_T_CLMB_MAX": 5, "FW_T_SINK_MAX": 4, "FW_T_SINK_MIN": 2, "VT_TRANS_TIMEOUT": 25}
    # A 38 kg airframe turns gently: bank <= 35 deg, roll/pitch rates <= 30 deg/s, slower attitude loops,
    # loiters of 150 m (25 m/s: 23 deg bank).
    p |= {"FW_R_LIM": 35, "FW_R_RMAX": 30, "FW_P_RMAX_POS": 30, "FW_P_RMAX_NEG": 30, "FW_R_TC": 0.6, "FW_P_TC": 0.5,
          "NAV_LOITER_RAD": 150}
    # TECS for a heavier, slower-responding aircraft.
    p |= {"FW_T_ALT_TC": 8.0, "FW_T_TAS_TC": 5.0}
    return p


def world_sdf(name, include_forest, rig_height=None, step=0.001, forest_model="forest"):
    # Ground collision: a 6 x 6 km box, top at z = 0. Not a <plane>: with gz-physics' DART a plane only
    # holds within some 1-2 km of the origin (spawned at y = -1900 m the vehicle sank through it and glided off).
    forest = """
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 0 -5 0 0 0</pose>
          <geometry><box><size>6000 6000 10</size></box></geometry>
        </collision>
      </link>
    </model>"""
    forest += """
    <include>
      <uri>model://{forest_model}</uri>
    </include>""".format(forest_model=forest_model) if include_forest else """
    <model name="ground">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <geometry><mesh><uri>model://forest/meshes/ground.obj</uri></mesh></geometry>
        </visual>
      </link>
    </model>"""
    rig = "" if rig_height is None else f"""
    <include>
      <uri>model://vtol_rig</uri>
      <name>vtol_rig</name>
      <pose>0 0 {rig_height} 0 0 1.5707963</pose>
    </include>"""
    return f"""<?xml version="1.0"?>
<!-- Generated by scripts/make_models.py; do not edit. -->
<sdf version="1.10">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>{round(1 / step)}</real_time_update_rate>
    </physics>
{WORLD_PLUGINS}
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.62 0.72 0.85 1</background>
      <shadows>false</shadows>
    </scene>
    <light name="sun" type="directional">
      <pose>0 0 1000 0 0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>1</intensity>
      <direction>0.35 0.45 -0.82</direction>
      <diffuse>0.95 0.95 0.92 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>21.0285</latitude_deg>
      <longitude_deg>105.8542</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>{forest}{rig}
  </world>
</sdf>
"""


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


def model_config(name, desc):
    return (f'<?xml version="1.0"?>\n<model><name>{name}</name><version>1.0</version>'
            f'<sdf version="1.10">model.sdf</sdf><description>{desc}</description></model>\n')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, default=500, help="rig height above ground in the static worlds, m")
    ap.add_argument("--image", choices=list(IMAGES), default="half", help="camera resolution (same FOV)")
    ap.add_argument("--camera-rate", type=float, default=1.0, help="camera frames/s (static rig); flight: triggered")
    ap.add_argument("--camera-triggered", action="store_true",
                    help="the flight model's camera fires on /vtol/camera/trigger (every 60 m) instead of a fixed rate")
    ap.add_argument("--scan-half-deg", type=float, default=20.0)
    ap.add_argument("--rays", type=int, default=1333)
    ap.add_argument("--line-rate", type=float, default=200.0)
    ap.add_argument("--max-range", type=float, default=880.0, help="880 m: 600 kHz mode (spec 2, default); 1200 m: 300 kHz")
    ap.add_argument("--cam-z", type=float, default=-0.18, help="camera origin below the body origin, m")
    ap.add_argument("--lidar-z", type=float, default=-0.42, help="LiDAR origin (below the camera), m")
    args = ap.parse_args()

    cfg = sensor_config(args)
    write(os.path.join(ROOT, "config", "sensors.json"), json.dumps(cfg, indent=2) + "\n")

    write(os.path.join(ROOT, "models", "vtol_rig", "model.sdf"), rig_sdf(cfg))
    write(os.path.join(ROOT, "models", "vtol_rig", "model.config"),
          model_config("vtol_rig", "Static CW-30E-sized body with the mapping payload"))

    # The flight model fires its camera on a trigger (spec: every 60 m along the line).
    flight = dict(cfg, camera=dict(cfg["camera"], triggered=True, update_rate_hz=5))
    write(os.path.join(ROOT, "models", "standard_vtol_mapping", "model.sdf"), vtol_sdf(flight))
    write(os.path.join(ROOT, "models", "standard_vtol_mapping", "model.config"),
          model_config("standard_vtol_mapping", "PX4 standard_vtol with the mapping payload"))
    write(os.path.join(ROOT, "config", "sensors_flight.json"), json.dumps(flight, indent=2) + "\n")

    write(os.path.join(ROOT, "worlds", "forest_static.sdf"), world_sdf("forest_static", True, args.height))
    write(os.path.join(ROOT, "worlds", "flat_static.sdf"), world_sdf("flat_static", False, args.height))
    write(os.path.join(ROOT, "worlds", "forest.sdf"), world_sdf("forest", True, None))
    write(os.path.join(ROOT, "worlds", "forest_slope.sdf"), world_sdf("forest_slope", True, None, forest_model="forest_slope"))
    write(os.path.join(ROOT, "worlds", "forest_slope_static.sdf"),
          world_sdf("forest_slope_static", True, args.height, forest_model="forest_slope"))
    write(os.path.join(ROOT, "worlds", "forest_1400.sdf"), world_sdf("forest_1400", True, None, forest_model="forest_1400"))
    write(os.path.join(ROOT, "worlds", "forest_1400_static.sdf"),
          world_sdf("forest_1400_static", True, args.height, forest_model="forest_1400"))
    write(os.path.join(ROOT, "models", "cw30e_mapping", "model.sdf"), cw30e_sdf(flight))
    write(os.path.join(ROOT, "models", "cw30e_mapping", "model.config"),
          model_config("cw30e_mapping", "CW-30E-sized quadplane (from PX4 standard_vtol) with the mapping payload"))
    write(os.path.join(ROOT, "config", "cw30e_px4_params.json"), json.dumps(cw30e_px4_params(), indent=2) + "\n")
    # Timing check: the moving rig with a faster camera.
    sync = dict(cfg, camera=dict(cfg["camera"], update_rate_hz=2.0), sync={"speed_mps": 25.0, "start_y_m": -400.0,
                                                                           "height_m": 500.0, "edge_y_m": 0.0, "step_m": 20.0,
                                                                           "slant_y0_m": -100.0, "slant_tan": 0.1,
                                                                           "slant_step_m": 10.0})
    write(os.path.join(ROOT, "models", "vtol_rig_moving", "model.sdf"), moving_rig_sdf(sync, 25.0))
    write(os.path.join(ROOT, "models", "vtol_rig_moving", "model.config"),
          model_config("vtol_rig_moving", "Rig at constant velocity for the timing check"))
    write(os.path.join(ROOT, "worlds", "sync_test.sdf"), sync_world_sdf(500.0, -400.0))
    write(os.path.join(ROOT, "config", "sensors_sync.json"), json.dumps(sync, indent=2) + "\n")
    c, l = cfg["camera"], cfg["lidar"]
    print(f"camera {c['width']}x{c['height']} fx {c['fx']:.3f} hfov {c['hfov_deg']:.3f} deg; "
          f"lidar {l['rays_per_line']} rays x {l['lines_per_s']:.0f} lines/s, +-{args.scan_half_deg} deg, "
          f"{l['range_min_m']}-{l['range_max_m']} m; rig at {args.height} m")


if __name__ == "__main__":
    main()
