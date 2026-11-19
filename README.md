# VTOL-mapping

Mô phỏng VTOL bay trên rừng ở 500–600 m AGL, mang camera nadir (Sony ILX‑LR1 + 35 mm) và LiDAR quét dòng
(thông số RIEGL VUX‑240²⁴), để sinh dữ liệu cho thuật toán mapping rừng. Gazebo Harmonic + PX4 SITL; PX4 chạy
nguyên bản, mọi thứ nằm trong thư mục này.

- Yêu cầu, thông số thiết bị: [Thong_so_mo_phong_VTOL_mapping_rung.md](Thong_so_mo_phong_VTOL_mapping_rung.md)
- Kết quả kiểm tra khả thi: [docs/ket_qua_kha_thi.md](docs/ket_qua_kha_thi.md)
- Giai đoạn 2 (GUI, camera view, bay đủ chu trình, 600 m / 10 tuyến, xuất dữ liệu): [docs/ke_hoach_chay.md](docs/ke_hoach_chay.md)
- Giai đoạn 3 (khớp tài liệu: rừng 2 km phẳng + dốc, LiDAR 600 kHz với thời điểm từng tia, máy bay cỡ CW‑30E,
  map 3D + RViz, §9.6 tách đất, xem point cloud, lưu trữ gọn, C++): [docs/ket_qua_giai_doan_3.md](docs/ket_qua_giai_doan_3.md)

Repo này là thư mục `examples/VTOL-mapping` của PX4-Autopilot: clone nó vào đúng chỗ đó, vì các script dùng
PX4 ở `../..` (bản build SITL, model `standard_vtol` và airframe 4004):

```bash
cd PX4-Autopilot/examples && git clone git@github.com:do010303/VTOL-mapping.git
```

Mọi lệnh chạy trong `examples/VTOL-mapping/`. Không có trong repo (sinh lại hoặc tự tạo khi chạy): `models/forest*/`
(`scripts/make_forest.py`), `out/`, `run/`.

## Bắt đầu nhanh

Mỗi bước một lệnh, chạy trong `examples/VTOL-mapping/`. Các phần sau của README giải thích chi tiết.

**1. Chuẩn bị (một lần, khoảng 5 phút)**

```bash
python3 scripts/make_forest.py                                       # rừng phẳng 2 × 2 km
python3 scripts/make_forest.py --terrain slope --name forest_slope   # rừng dốc
python3 scripts/make_models.py                                       # máy bay, world, config/
cpp/build.sh                                                         # phần xử lý LiDAR bằng C++ (nhanh hơn)
pip install --user laspy lazrs cloth-simulation-filter               # LAZ và CSF
```

**2. Bay thử 3 tuyến** (khoảng 25 phút, khoảng 0,4 GB). Đây là bước kiểm tra hệ thống của §7, ảnh lưu ở 1/4 cỡ:

```bash
PROFILE=test LINES=3 GUI=1 scripts/main_run.sh       # GUI=1: mở Gazebo; bỏ đi để chạy nhanh hơn
scripts/camera_window.sh                             # (tuỳ chọn, terminal khác) camera + panel LiDAR khi đang bay
```

**3. Bay cấu hình chính** (10 tuyến, 600 m, 25 m/s; khoảng 35 phút, khoảng 1,3 GB):

```bash
scripts/main_run.sh                                  # rừng phẳng
WORLD=forest_slope scripts/main_run.sh               # rừng dốc
```

`main_run.sh` tự làm hết một lượt: bay, dừng sim, dò lại từng tia (`lidar_exact`), viết báo cáo, kiểm tra SfM,
dựng map 3D. Kết quả nằm trong `out/main_<world>_<thời gian>/` (hoặc `out/<OUT>`).

**4. Xem kết quả** (sau khi mô phỏng đã dừng)

```bash
python3 scripts/view_cloud.py                        # point cloud 3D của lượt mới nhất (cách nên dùng)
less out/<run>/flight_report_exact.md                # báo cáo: tuyến bay, mật độ, độ phủ, DSM, ảnh
scripts/lidar_rviz.sh                                # (tuỳ chọn) phát lại trên RViz, chỉ khi sim đã dừng
```

**5. Đánh giá và xuất dữ liệu (khi cần)**

```bash
python3 scripts/ground_eval.py out/<run phẳng> out/<run dốc>   # §9.6: CSF/PTD, DTM/DSM/CHM, một bảng chung
python3 scripts/export_dataset.py out/<run> --lines 10         # §8: LAZ + COLMAP vào out/<run>/export/
```

**6. Dọn dẹp**

```bash
scripts/stop_sim.sh                                  # dừng sim (chỉ các tiến trình ví dụ này đã khởi động)
python3 scripts/compact_runs.py --apply              # nén lại các lượt cũ trong out/, không mất dữ liệu
```

Không chạy RViz hay một sim thứ hai cùng lúc với sim đang bay. Muốn theo dõi khi bay thì dùng Gazebo GUI và
`camera_window.sh`.

## Chuẩn bị (một lần)

Cần: PX4 SITL đã build (`build/px4_sitl_mapping` hoặc `build/px4_sitl_default`), Gazebo Harmonic, Python với numpy,
opencv, pillow, matplotlib, scipy, pymavlink.

```bash
python3 scripts/make_forest.py                                   # rừng phẳng 2 × 2 km (seed 2026), ~1 phút, 600 MB
python3 scripts/make_forest.py --terrain slope --name forest_slope   # rừng dốc 2,5° + đồi ±8 m, 750 MB
python3 scripts/make_models.py      # payload, máy bay (cw30e_mapping, standard_vtol_mapping), world, config/
cpp/build.sh                        # phần xử lý LiDAR bằng C++ (pybind11 + OpenMP; apt: python3-pybind11)
pip install --user laspy lazrs cloth-simulation-filter   # LAZ, CSF (§9.6)
```

`cpp/build.sh` build `cpp/vtol_core.cpp` thành `scripts/vtol_core*.so`. Thư viện này làm các vòng lặp nặng:
- đổi từng tia sang toạ độ thế giới, với pose nội suy đúng thời điểm của tia;
- gom điểm vào các ô raster (DSM/DTM);
- cộng dồn voxel cho map 3D.

Không build thì các script chạy cùng phép tính bằng NumPy (`scripts/fastcore.py`), kết quả như nhau (lệch dưới
0,02 mm), chỉ chậm hơn. Đo trên lượt 3 tuyến:

| Bước | NumPy | C++ |
|---|---|---|
| Báo cáo bay | 32 s, 3,4 GB RAM | 13 s, 2,9 GB |
| Map 3D | 119 s, 8,3 GB RAM | 35 s, 2,7 GB |

`VTOL_NO_NATIVE=1` ép chạy bằng NumPy.

Cảnh (mục 6 của tài liệu):

| World | Cảnh | Dùng cho |
|---|---|---|
| `forest`, `forest_static` | 2 × 2 km, đất phẳng, 129 554 cây 10–30 m, tán có texture | Cấu hình chính |
| `forest_slope`, `forest_slope_static` | Như trên trên địa hình dốc 2,5° về phía đông + đồi ±8 m; vùng cất cánh phẳng | Cảnh dốc của tài liệu |
| `forest_1400`, `forest_1400_static` | Rừng 1,4 km của giai đoạn 1–2 (màu trơn) | Tái lập R1–R6 (`python3 scripts/make_forest.py --size 1400 --name forest_1400` không tái tạo đúng cảnh cũ: cảnh cũ giữ nguyên trong `models/forest_1400/`) |

Dữ liệu chuẩn của từng cảnh nằm trong `models/<cảnh>/`: `truth.npz` (DTM, DSM, CHM 1 m và lưới địa hình chính
xác), `trees.csv` (cây, kèm `ground_z_m`), `forest.json` (seed, thông số). Mỗi lượt chạy ghi tên cảnh vào
`out/<run>/sensors.json` (`scene`), nên các script báo cáo tự lấy đúng dữ liệu chuẩn.

Đổi thông số cảm biến bằng tham số của `make_models.py`, rồi chạy lại các bài thử, ví dụ:

- `--max-range 1200`: chế độ 300 kHz (mặc định 880 m: chế độ 600 kHz của tài liệu).
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

**Bay PX4.** Máy bay mặc định là `cw30e_mapping`: quadplane cỡ JOUAV CW‑30E (sải cánh 4,4 m, dài 2,4 m, 38 kg
gồm cả tải cảm biến, bay bằng 25 m/s), dựng từ `standard_vtol` của PX4 (cùng bố trí 4 rotor nâng + 1 đẩy, cùng
airframe 4004) với kích thước, khối lượng, quán tính, cánh, đuôi và động cơ tính lại; tham số PX4 riêng trong
`config/cw30e_px4_params.json` (sinh bởi `make_models.py`). JOUAV không công bố số liệu khí động, nên đây là máy
bay đúng cỡ và đúng chế độ bay, không phải động lực học thật của CW‑30E. `VEHICLE=standard_vtol` dùng lại máy bay
5 kg của giai đoạn 1–2.

Một lệnh cho cấu hình chính của tài liệu (600 m, 25 m/s, 10 tuyến cách 95 m, ảnh chỉ trong vùng rừng), kể cả xử lý:

```bash
scripts/main_run.sh                                    # rừng phẳng -> out/main_forest_<thời gian>/
WORLD=forest_slope scripts/main_run.sh                 # rừng dốc
PROFILE=test LINES=3 GUI=1 scripts/main_run.sh         # kiểm tra hệ thống 3 tuyến (§7), ảnh 1/4, có GUI
EXPORT=1 scripts/main_run.sh                           # kèm xuất LAZ + COLMAP (hoặc export_dataset.py sau)
```

Ước tính dung lượng: lượt chính 10 tuyến khoảng 1,3 GB, lượt thử 3 tuyến khoảng 0,3 GB. Phần lớn là ảnh: JPEG 85,
chỉ chụp trên tuyến.

Hoặc từng bước, hai terminal:

```bash
./run_sitl.sh                                   # terminal 1: Gazebo + PX4 (cw30e_mapping) + các node; GUI=1 để xem
scripts/vtol_mission.py                         # terminal 2: cất cánh, leo vòng lên 500 m, 1 tuyến x = 0, hạ cánh tại home
python3 scripts/flight_report.py out/flight_<thời gian> --planned-x 0      # sau khi hạ cánh (Ctrl+C terminal 1)
```

```bash
MIN_ALT=550 TRIGGER_BOX="-700 700 -700 700" IMG=jpg ./run_sitl.sh
scripts/vtol_mission.py --alt 600 --lines 10 --spacing 95
python3 scripts/lidar_exact.py out/flight_<thời gian>
python3 scripts/flight_report.py out/flight_<thời gian> --planned-x -427.5 -332.5 -237.5 -142.5 -47.5 47.5 142.5 237.5 332.5 427.5
python3 scripts/export_dataset.py out/flight_<thời gian> --lines 10 --spacing 95
```

- `vtol_mission.py`:
  - `--lines N --spacing D`: các tuyến bắc–nam, bay xen kẽ chiều, căn giữa quanh x = 0.
  - `--run-in 350`: đoạn vào tuyến ở mỗi đầu. Sau vòng quay đầu 95 m ở 25 m/s, quỹ đạo cần khoảng 400 m để ổn định.
  - `--stop-after-lines`: thoát sau tuyến cuối; PX4 vẫn tự hạ cánh.
- `run_sitl.sh`:
  - `WORLD`: `forest` (mặc định), `forest_slope`, `forest_1400`. `VEHICLE`: `cw30e` (mặc định), `standard_vtol`.
  - `AIRSPEED`: tốc độ bay (cw30e: 25 m/s; standard_vtol: 17, trên 18 m/s thì FW_AIRSPD_MAX tự được nâng).
  - `THR_MAX`: giới hạn ga.
  - `MIN_ALT`: các node chỉ ghi và kích chụp ở trên độ cao này.
  - `TRIGGER_BOX`: camera chỉ chụp trong vùng này.
  - `VIEW=0`: không ghi video camera view. `IMG=jpg`: ảnh JPEG thay vì PNG (~16 MB). `IMG_QUALITY`: chất lượng JPEG
    (mặc định 85, khoảng 3,4 MB). Thử trên lượt 3 tuyến: số điểm SfM khớp không đổi so với chất lượng 95 (6,2 MB).
  - `IMG_SCALE=2`: ghi ảnh nhỏ hơn 2 lần, 2376 × 1584, khoảng 1 MB. `sensors.json` của lượt chạy ghi K đã chia
    tương ứng.
  - `ON_LINE=0`: chụp cả khi quay đầu và vào tuyến. Mặc định chỉ chụp trên tuyến: hướng bay bắc/nam lệch ≤ 10°,
    nghiêng cánh < 8°, mốc 60 m bắt đầu lại ở đầu mỗi tuyến. Điều kiện kích chụp được xét ở mỗi pose (500 Hz thời
    gian mô phỏng), nên khoảng cách chụp không phụ thuộc tốc độ chạy của mô phỏng.
- Thời gian bay: 1 tuyến ở 500 m khoảng 14 phút; 10 tuyến ở 600 m khoảng 30 phút.
- Khi có GUI, mô phỏng chạy ở khoảng 0,5 lần thời gian thực (không GUI: 0,9).

**Camera view.** Có một camera xem trước cùng vị trí và cùng FOV với camera chính (792 × 528, 10 Hz):

- Khi chạy `GUI=1`, nó hiện thành panel "Camera view" trong cửa sổ Gazebo.
- Mỗi lượt chạy đều ghi video `camera_view.mp4` (`VIEW=0` để tắt).
- Cửa sổ gồm ba phần:
  - **Ảnh camera.** Điểm LiDAR của 1 s gần nhất chiếu lên ảnh, màu theo độ cao trên địa hình (0–30 m). Bên dưới là
    mặt cắt ngang của dòng quét mới nhất.
  - **Panel LiDAR.** Thông số máy quét, rồi số liệu trực tiếp: dòng/s, % tia có phản hồi, điểm/s, cự ly
    min/trung vị/max, độ cao LiDAR trên mặt đất, bề rộng dải quét, khoảng cách dòng quét, mật độ điểm, % chạm đất,
    chiều cao tán p95. Tiếp theo là tổng số dòng/điểm, diện tích đã phủ, thang màu, và bản đồ nhìn từ trên mọi ô
    5 m đã quét (màu theo chiều cao tán), kèm vị trí, hướng máy bay và khung ảnh camera.
  - **Panel hiệu năng.** RTF, độ trễ, FPS, CPU/RAM từng tiến trình, GPU.
- `scripts/camera_window.sh` mở cửa sổ riêng hiện camera view trực tiếp bên cạnh một sim đang chạy (có GUI hay không
  đều được). Nhấn `q`/`Esc` hoặc đóng cửa sổ để thoát; sim vẫn chạy tiếp. Với mô hình tĩnh:
  `scripts/camera_window.sh forest_static vtol_rig`.
- Camera này chỉ để xem, không dùng cho mapping.

**LiDAR đúng lịch phát tia** (`scripts/lidar_exact.py`, mục 2 của tài liệu). Gazebo vẽ cả dòng quét tại một
thời điểm. Script này dò lại từng tia trên chính mesh của cảnh (Open3D): tia k phát ở t_s + k/600 000 s với pose
nội suy tại đúng thời điểm đó, góc danh nghĩa, không cần hiệu chuẩn. Khoảng 2 triệu tia/s.

```bash
python3 scripts/lidar_exact.py out/<run>         # -> out/<run>/lidar_exact/ (+ so sánh từng tia với Gazebo)
```

`flight_report.py` và `export_dataset.py` tự dùng `lidar_exact/` nếu có (`--lidar gz` để dùng bản Gazebo); báo
cáo khi đó có hậu tố `_exact`.

**Map 3D và RViz** (`scripts/lidar_map.py`, giống `ground_map.py` của px4-mapping). Node ghép point cloud thành
map voxel (mặc định 1 m), ước lượng mặt đất (điểm thấp nhất trong 11 m) và chiều cao tán, chỉ từ dữ liệu LiDAR:

RViz chỉ chạy **sau khi mô phỏng xong**, riêng một mình. Script tự từ chối nếu đang có `gz sim` chạy. Để theo dõi
trong lúc bay, dùng `scripts/camera_window.sh`: panel LiDAR của cửa sổ này có bản đồ nhìn từ trên.

```bash
scripts/lidar_rviz.sh                        # phát lại lượt bay mới nhất trong out/, nhanh gấp 20
scripts/lidar_rviz.sh out/main_flat 20       # một lượt cụ thể
python3 scripts/lidar_map.py out/main_flat   # chỉ dựng và lưu map, không mở RViz
```

> **Lưu ý về RViz.** RViz2 không chịu được map lớn. Ngày 19/11/2026, khi chạy song song với mô phỏng và map tăng
> lên vài triệu voxel gửi mỗi 2 s, nó đã tắt đột ngột mà không ghi lỗi. Node `lidar_map.py` vẫn lưu map khi
> dừng, chỉ cửa sổ RViz bị mất. Vì vậy:
>
> - Chỉ phát lại sau chuyến bay, không chạy song song với mô phỏng.
> - Mặc định gửi tối đa 1 triệu voxel (`--max-publish`). Map rất lớn thì giảm thêm:
>   `MAP_ARGS="--every 4 --publish-period 5 --max-publish 500000" scripts/lidar_rviz.sh`.
> - Chỉ cần xem map đã dựng (không cần phát lại theo thời gian) thì dùng `scripts/view_cloud.py` (bên dưới):
>   nhẹ hơn và không cần ROS.

Topic RViz (frame `map` = ENU thế giới): `/vtol/lidar/scan` (điểm của 1 s gần nhất), `/vtol/lidar/map` (voxel,
màu theo độ cao trên mặt đất), `/vtol/lidar/ground`, `/vtol/path`, `/vtol/vehicle`, `/vtol/camera_footprint`.
Khi dừng, map được lưu thành `lidar_map.npz` và `lidar_map.png` (`--ply`: thêm `lidar_map.ply` cho
CloudCompare/MeshLab).

**Xem map 3D không cần RViz** (`scripts/view_cloud.py`, kiểu `view_map.py` của px4-mapping). Đây là cách xem
point cloud nên dùng (xem lưu ý về RViz ở trên). Cửa sổ Open3D nền
sáng: các lớp đất / cây bụi 0,5–5 m / tán (màu theo chiều cao), đường bay và các tuyến khảo sát có nhãn, vị trí và
khung ảnh, lưới 100 m. Bảng bên phải có số liệu của map và của `flight_report` (nếu đã chạy), ô bật/tắt từng lớp, cỡ
điểm, các góc nhìn và nút lưu ảnh.

```bash
python3 scripts/view_cloud.py                              # lượt bay mới nhất trong out/
python3 scripts/view_cloud.py out/<run>                    # lượt này; lần đầu tự dựng lidar_map.npz (~1–2 phút)
python3 scripts/view_cloud.py out/<run> --crop -100 100 -100 100 --max-points 0   # cận cảnh, mọi voxel
python3 scripts/view_cloud.py out/<run> --whole --colour hits --theme dark --font-size 20
python3 scripts/view_cloud.py out/<run> --png map.png      # chỉ lưu ảnh
```

**Kiểm tra ảnh cho SfM** (mục 6, 9.5): `python3 scripts/sfm_check.py out/<run>` ghép SIFT giữa các cặp ảnh dọc
tuyến và ngang tuyến, rồi dùng pose thật để đếm cặp điểm khớp đúng hình học. `scripts/texture_pairs.sh` làm
việc đó với rig tĩnh (6 ảnh ở 600 m), để thử texture của một cảnh mà không cần bay.

**Xuất dữ liệu** (`scripts/export_dataset.py`, mục 8 của tài liệu) vào `out/<run>/export/`:

| Tệp | Nội dung |
|---|---|
| `trajectory.csv` | Pose 500 Hz + `strip_id` và cờ survey/transit |
| `lidar/strip_*.npz` | Phép đo gốc theo dải |
| `las/strip_*.laz` | LAZ (LAS 1.2 PDRF 1, nén không mất dữ liệu, cần `pip install --user laspy lazrs`; `--las` để giữ LAS thường), gps_time = thời gian mô phỏng, point_source_id = dải; mặc định cắt theo vùng đánh giá + 100 m |
| `truth/` | Nhãn đất/tán/mã cây, để riêng |
| `colmap/sparse/0/` | Pose ảnh cho COLMAP |
| `checks.json`, `coverage.png` | Mật độ, độ phủ, số ảnh nhìn thấy mỗi ô |

**Tách đất, DTM / DSM / CHM** (`scripts/ground_eval.py`, mục 9.6). Lọc đất bằng CSF (`pip install --user
cloth-simulation-filter`) và PTD (cài sẵn trong script) trên một mẫu con 10 điểm/m², dựng DTM 1 m từ điểm đất của
mỗi bộ lọc, DSM từ điểm cao nhất mỗi ô, CHM = DSM − DTM, rồi so với dữ liệu chuẩn của cảnh: precision / recall / F1
lớp đất, bias / RMSE / p95 của DTM, DSM, CHM, tỉ lệ ô có dữ liệu. Nhiều lượt bay (ví dụ cảnh phẳng và cảnh dốc) cho
một bảng chung:

```bash
python3 scripts/ground_eval.py out/main_flat out/main_slope   # -> out/ground_eval.md, out/<run>/ground_eval.{json,png}
```

**So sánh nhiều cấu hình bay cùng một lượt:** `flight_report.py ... --tag <tên>` ghi
`flight_report_exact_<tên>.md/json` để các báo cáo (ví dụ 1, 2, 10 tuyến) không ghi đè nhau.

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
| `lidar/scans_*.npz` | `t_s` mỗi dòng, `scan_id`, cự ly dòng × 1333 tia lưu theo mm, mã hoá chênh lệch giữa hai tia liền nhau (`scripts/scan_codec.py`; sai ≤ 0,5 mm, nhỏ hơn float32 khoảng 4,5 lần). Đọc bằng `scan_codec.load()`, trả về `ranges_m` float32, `inf` = không phản hồi. File cũ dạng `ranges_m` vẫn đọc được |
| `camera/img_*.jpg` (hoặc `.png`), `images.csv` | Ảnh RGB; image_id, t_s, pose thân lúc chụp, thời điểm trigger |
| `camera/camera_info.json` | K, kích thước, méo do Gazebo phát |
| `cloud_line.npz` | (flight_report) xyz + t các điểm trên tuyến |
| `lidar_exact/scans_*.npz` | (lidar_exact.py) như trên, tia k phát ở `t_s + k · pulse_spacing_s` |
| `models/<cảnh>/truth.npz`, `trees.csv` | DTM, DSM, CHM chuẩn 1 m, lưới địa hình; danh sách cây (vị trí, cao, tán, hướng, độ cao gốc) |

Bản Gazebo: hướng tia k là d_L = chuẩn hóa(γ_k, sin β_k, −cos β_k), với β_k và γ_k lấy từ
`config/lidar_angles.json`; điểm p_W = T_WB(t_s) · (t_BL + range · d_L). Bản `lidar_exact`: góc danh nghĩa
β_k = −20° + k · 0,030°, γ = 0, và p_W = T_WB(t_s + k/600 000) · (t_BL + range · d_L).

## Lưu ý

- **Dung lượng.** Một lượt 10 tuyến khoảng 3 GB (ảnh JPEG 1,4 GB, LiDAR nén ~0,5 GB, LAZ ~0,5 GB). Các script ghi
  LiDAR dạng nén (`.npz` zip, ~5 lần nhỏ hơn) và LAZ; `px4.ulg` được nén zstd (`zstd -d px4.ulg.zst` để đọc).
  `python3 scripts/compact_runs.py` (chạy thử) / `--apply` nén lại các lượt cũ trong `out/` mà không mất dữ liệu;
  `--png-to-jpg 95` thêm chuyển ảnh PNG sang JPEG (có mất mát).
- **Nhường sim khác.** Chỉ khởi động khi máy không có `gz sim`, sim px4-mapping, ArduPilot hay PX4 SITL nào khác.
  Nếu một sim khác khởi động giữa chừng, sim này tự dừng.
- Gazebo chạy ở partition riêng `GZ_PARTITION=vtol_mapping` (`source env.sh` trước khi dùng `gz topic`), dưới tên
  tiến trình `vtol_mapping_gazebo`.
- **Bộ nhớ.** Gazebo dùng khoảng 2,1 GB. Nếu MemAvailable xuống dưới 1 GB (`MIN_FREE_MB`), sim tự dừng.
- World không nạp `server.config` của PX4: GstCameraSystem của nó sẽ stream camera 15 MP và rò bộ nhớ với driver
  NVIDIA hiện tại.
- Mặt đất va chạm là hộp 6 × 6 km, không dùng `<plane>`: với DART, plane chỉ giữ được trong khoảng 1–2 km quanh gốc.
