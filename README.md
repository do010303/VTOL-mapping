# VTOL-mapping

Mô phỏng VTOL bay trên rừng ở 500–600 m AGL, mang camera nadir (Sony ILX‑LR1 + 35 mm) và LiDAR quét dòng
(thông số RIEGL VUX‑240²⁴), để sinh dữ liệu cho thuật toán mapping rừng. Gazebo Harmonic + PX4 SITL; PX4 chạy
nguyên bản, mọi thứ nằm trong thư mục này.

- Yêu cầu, thông số thiết bị: [Thong_so_mo_phong_VTOL_mapping_rung.md](Thong_so_mo_phong_VTOL_mapping_rung.md)
- Kết quả kiểm tra khả thi: [docs/ket_qua_kha_thi.md](docs/ket_qua_kha_thi.md)
- Giai đoạn 2 (GUI, camera view, bay đủ chu trình, 600 m / 10 tuyến, xuất dữ liệu): [docs/ke_hoach_chay.md](docs/ke_hoach_chay.md)

Repo này là thư mục `examples/VTOL-mapping` của PX4-Autopilot: clone nó vào đúng chỗ đó, vì các script dùng
PX4 ở `../..` (bản build SITL, model `standard_vtol` và airframe 4004):

```bash
cd PX4-Autopilot/examples && git clone git@github.com:do010303/VTOL-mapping.git
```

Mọi lệnh chạy trong `examples/VTOL-mapping/`. Không có trong repo (sinh lại hoặc tự tạo khi chạy): `models/forest/`
(`scripts/make_forest.py`), `out/`, `run/`.

## Chuẩn bị (một lần)

Cần: PX4 SITL đã build (`build/px4_sitl_mapping` hoặc `build/px4_sitl_default`), Gazebo Harmonic, Python với numpy,
opencv, pillow, matplotlib, scipy, pymavlink.

```bash
python3 scripts/make_forest.py      # rừng 1,4 × 1,4 km (seed 2026), ~10 s, 217 MB mesh trong models/forest/ (không commit)
python3 scripts/make_models.py      # payload, VTOL, các world, config/sensors*.json
```

Đổi thông số cảm biến bằng tham số của `make_models.py`, rồi chạy lại các bài thử, ví dụ:

- `--max-range 880`: chế độ 600 kHz.
- `--image full`: ảnh 9504 × 6336.
- `--line-rate`, `--rays`, `--scan-half-deg`.

## Chạy

**Mô hình tĩnh trên rừng.** Rig cố định, camera và LiDAR ghi cùng lúc, cuối lượt có báo cáo:

```bash
scripts/static_run.sh                                         # 500 m, 10 s mô phỏng
HEIGHTS="500 600 880 1000 1100 1150 1200 1250" DURATION=4 scripts/static_run.sh   # quét ngưỡng tầm LiDAR
WORLD=flat_static HEIGHTS="500 1000" scripts/static_run.sh     # đất phẳng: sai số cự ly
GUI=1 scripts/static_run.sh                                   # kèm giao diện Gazebo
```

Kết quả nằm trong `out/<world>_<thời gian>/`:

- `report.md`: bảng kết quả.
- `lidar_returns.png`: tỉ lệ phản hồi theo góc quét và theo độ cao.
- `overlay_h<H>.jpg`: đỉnh cây và dòng LiDAR chiếu lên ảnh.
- `h<H>/`: dữ liệu gốc.

**Bay PX4.** Hai terminal:

```bash
./run_sitl.sh                                   # terminal 1: Gazebo + PX4 (standard_vtol_mapping) + các node; GUI=1 để xem
scripts/vtol_mission.py                         # terminal 2: cất cánh, leo vòng lên 500 m, 1 tuyến x = 0, hạ cánh tại home
python3 scripts/flight_report.py out/flight_<thời gian> --planned-x 0      # sau khi hạ cánh (Ctrl+C terminal 1)
```

Cấu hình chính của tài liệu (600 m, 25 m/s, 10 tuyến cách 95 m, ảnh chỉ trong vùng rừng):

```bash
AIRSPEED=25 MIN_ALT=550 TRIGGER_BOX="-700 700 -700 700" ./run_sitl.sh
scripts/vtol_mission.py --alt 600 --lines 10 --spacing 95
python3 scripts/flight_report.py out/flight_<thời gian> --planned-x -427.5 -332.5 -237.5 -142.5 -47.5 47.5 142.5 237.5 332.5 427.5
python3 scripts/export_dataset.py out/flight_<thời gian> --lines 10 --spacing 95
```

- `vtol_mission.py`:
  - `--lines N --spacing D`: các tuyến bắc–nam, bay xen kẽ chiều, căn giữa quanh x = 0.
  - `--run-in 350`: đoạn vào tuyến ở mỗi đầu. Sau vòng quay đầu 95 m ở 25 m/s, quỹ đạo cần khoảng 400 m để ổn định.
  - `--stop-after-lines`: thoát sau tuyến cuối; PX4 vẫn tự hạ cánh.
- `run_sitl.sh`:
  - `AIRSPEED`: tốc độ bay. Trên 18 m/s, giới hạn FW_AIRSPD_MAX tự được nâng.
  - `THR_MAX`: giới hạn ga.
  - `MIN_ALT`: các node chỉ ghi và kích chụp ở trên độ cao này.
  - `TRIGGER_BOX`: camera chỉ chụp trong vùng này.
  - `VIEW=0`: không ghi video camera view.
- Thời gian bay: 1 tuyến ở 500 m khoảng 14 phút; 10 tuyến ở 600 m khoảng 30 phút.
- Khi có GUI, mô phỏng chạy ở khoảng 0,5 lần thời gian thực (không GUI: 0,9).

**Camera view.** Có một camera xem trước cùng vị trí và cùng FOV với camera chính (792 × 528, 10 Hz):

- Khi chạy `GUI=1`, nó hiện thành panel "Camera view" trong cửa sổ Gazebo.
- Mỗi lượt chạy đều ghi video `camera_view.mp4`, với vạch đỏ là dải quét LiDAR, thước 50 m, thời gian, độ cao và
  tốc độ.
- `scripts/camera_window.sh` mở cửa sổ riêng hiện camera view trực tiếp bên cạnh một sim đang chạy (có GUI hay không
  đều được). Nhấn `q`/`Esc` hoặc đóng cửa sổ để thoát; sim vẫn chạy tiếp. Với mô hình tĩnh:
  `scripts/camera_window.sh forest_static vtol_rig`.
- Camera này chỉ để xem, không dùng cho mapping.

**Xuất dữ liệu** (`scripts/export_dataset.py`, mục 8 của tài liệu) vào `out/<run>/export/`:

| Tệp | Nội dung |
|---|---|
| `trajectory.csv` | Pose 500 Hz + `strip_id` và cờ survey/transit |
| `lidar/strip_*.npz` | Phép đo gốc theo dải |
| `las/strip_*.las` | LAS 1.2 PDRF 1, gps_time = thời gian mô phỏng, point_source_id = dải; mặc định cắt theo vùng đánh giá + 100 m |
| `truth/` | Nhãn đất/tán/mã cây, để riêng |
| `colmap/sparse/0/` | Pose ảnh cho COLMAP |
| `checks.json`, `coverage.png` | Mật độ, độ phủ, số ảnh nhìn thấy mỗi ô |

**Đồng bộ thời gian và hiệu chuẩn LiDAR.** Cần làm lại khi đổi số tia, góc quét hoặc phiên bản Gazebo:

```bash
WORLD=flat_static HEIGHTS=500 DURATION=2 OUT=out/flat_cal scripts/static_run.sh
python3 scripts/calibrate_lidar.py out/flat_cal/h500          # góc ngang tuyến thực của từng tia
OUT=out/sync scripts/sync_run.sh                              # rig 25 m/s qua bậc vuông và bậc xiên
python3 scripts/calibrate_lidar_along.py out/sync             # độ nghiêng dọc tuyến của từng tia
python3 scripts/sync_check.py out/sync                        # độ trễ pose / LiDAR / camera
```

Hai hiệu chuẩn này ghi vào `config/lidar_angles.json`. Các script báo cáo dùng nó tự động; thêm `--no-calibration`
để so với góc danh nghĩa.

**Dừng:** `scripts/stop_sim.sh`. Lệnh này chỉ dừng các PID mà ví dụ này đã khởi động.

## Dữ liệu xuất (mục 8 của tài liệu)

| Tệp | Nội dung |
|---|---|
| `sensors.json` | Mọi thông số camera/LiDAR, phép biến đổi gá, quy ước hệ trục |
| `trajectory.csv` | t_s, x, y, z, qx, qy, qz, qw của thân (ENU, thế giới), 500 Hz |
| `lidar/scans_*.npz` | `t_s` mỗi dòng, `scan_id`, `ranges_m` (dòng × 1333, float32, `inf` = không phản hồi) |
| `camera/img_*.png`, `images.csv` | Ảnh RGB; image_id, t_s, pose thân lúc chụp, thời điểm trigger |
| `camera/camera_info.json` | K, kích thước, méo do Gazebo phát |
| `cloud_line.npz` | (flight_report) xyz + t các điểm trên tuyến |
| `models/forest/truth.npz`, `trees.csv` | DTM, DSM, CHM chuẩn 1 m; danh sách cây (vị trí, cao, tán, hướng) |

Hướng tia k: d_L = chuẩn hóa(γ_k, sin β_k, −cos β_k), với β_k và γ_k lấy từ `config/lidar_angles.json`.
Điểm: p_W = T_WB(t) · (t_BL + range · d_L).

## Lưu ý

- **Nhường sim khác.** Chỉ khởi động khi máy không có `gz sim`, sim px4-mapping, ArduPilot hay PX4 SITL nào khác.
  Nếu một sim khác khởi động giữa chừng, sim này tự dừng.
- Gazebo chạy ở partition riêng `GZ_PARTITION=vtol_mapping` (`source env.sh` trước khi dùng `gz topic`), dưới tên
  tiến trình `vtol_mapping_gazebo`.
- **Bộ nhớ.** Gazebo dùng khoảng 2,1 GB. Nếu MemAvailable xuống dưới 1 GB (`MIN_FREE_MB`), sim tự dừng.
- World không nạp `server.config` của PX4: GstCameraSystem của nó sẽ stream camera 15 MP và rò bộ nhớ với driver
  NVIDIA hiện tại.
- Mặt đất va chạm là hộp 6 × 6 km, không dùng `<plane>`: với DART, plane chỉ giữ được trong khoảng 1–2 km quanh gốc.
