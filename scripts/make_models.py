#!/usr/bin/env python3
"""Sensor payload, VTOL models and worlds for the VTOL LiDAR + camera simulation.

One place for the sensor parameters (spec sections 2, 3 and 5), so the static
rig, the flyable PX4 VTOL and the processing scripts all use the same values.
Writes:

  models/vtol_rig/                static VTOL-sized body (CW-30E: 2.4 m long, 4.4 m span) + payload, no physics
  models/standard_vtol_mapping/   PX4's standard_vtol (airframe 4004) + the same payload under its belly
  worlds/forest_static.sdf        forest + vtol_rig at --height above the ground
  worlds/flat_static.sdf          ground only + vtol_rig (range accuracy check, spec 9.1)
  worlds/forest.sdf               forest only; PX4 spawns standard_vtol_mapping into it
  config/sensors.json             every sensor parameter and mount transform, for the nodes and the report

Payload, centred under the belly (body frame FLU: x forward, y left, z up):
  camera  Sony ILX-LR1 + 35 mm, nadir, long image side across track       at (0, 0, cam_z)
  LiDAR   RIEGL VUX-240-like line scanner, nadir, line across track       at (0, 0, lidar_z), below the camera

  python3 scripts/make_models.py                              # spec values: 4752 x 3168, +-20 deg, 1333 rays, 200 lines/s
  python3 scripts/make_models.py --max-range 880              # the 600 kHz mode's range instead of 1200 m (300 kHz)
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
            "note": "gz renders a whole line at one instant; per-ray times t0 + k/600000 are assigned by the node",
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
      <visual name="fuselage">
        <geometry><box><size>2.4 0.30 0.30</size></box></geometry>
        <material><ambient>0.9 0.9 0.92 1</ambient><diffuse>0.9 0.9 0.92 1</diffuse></material>
      </visual>
      <visual name="wing">
        <pose>0.15 0 0.12 0 0 0</pose>
        <geometry><box><size>0.45 4.4 0.05</size></box></geometry>
        <material><ambient>0.9 0.9 0.92 1</ambient><diffuse>0.9 0.9 0.92 1</diffuse></material>
      </visual>
      <visual name="boom_left"><pose>0 1.1 0.05 0 0 0</pose>
        <geometry><box><size>1.8 0.06 0.06</size></box></geometry></visual>
      <visual name="boom_right"><pose>0 -1.1 0.05 0 0 0</pose>
        <geometry><box><size>1.8 0.06 0.06</size></box></geometry></visual>
      <visual name="tail">
        <pose>-1.1 0 0.15 0 0 0</pose>
        <geometry><box><size>0.3 1.4 0.04</size></box></geometry>
      </visual>{payload_sdf(cfg)}
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


def world_sdf(name, include_forest, rig_height=None, step=0.001):
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
      <uri>model://forest</uri>
    </include>""" if include_forest else """
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
    ap.add_argument("--max-range", type=float, default=1200.0, help="1200 m: 300 kHz mode; 880 m: 600 kHz mode")
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
