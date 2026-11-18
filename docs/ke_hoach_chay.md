# Kế hoạch chạy — giai đoạn 2 (từ 17/11/2026)

Giai đoạn 1 (khả thi) xong: [ket_qua_kha_thi.md](ket_qua_kha_thi.md). Thông số cảm biến giữ theo
[tài liệu](../Thong_so_mo_phong_VTOL_mapping_rung.md) (RIEGL VUX‑240²⁴, ILX‑LR1 + 35 mm).

Phạm vi lần này:
- chạy có giao diện;
- một chu trình bay đủ, kết thúc bằng hạ cánh;
- pose ≥ 200 Hz;
- cấu hình chính của tài liệu (600 m, 10 tuyến);
- xuất LAS và dữ liệu cho SfM.

**Chưa làm:** tách đất CSF/PTD, DTM/CHM và các chỉ số (tài liệu 9.6), mô hình phản xạ và nhiều phản hồi.

Mọi lượt chạy trên laptop RTX 3060 / 15 GB RAM của người dùng. Script chỉ khởi động khi không có sim nào khác
đang chạy, và tự dừng nếu RAM trống dưới 1 GB.

## Từng lượt chạy

Kết quả: [ket_qua_giai_doan_2.md](ket_qua_giai_doan_2.md). Cả 6 lượt đều đạt (17/11/2026).

| # | Chức năng | Cấu hình | Lệnh | Thời gian | Đạt khi | Đầu ra |
|---|---|---|---|---|---|---|
| R1 | Pose 500 Hz | `sync_test`, rig 25 m/s | `scripts/sync_run.sh` | 2 phút | Pose ≥ 200 Hz, độ trễ 0; LiDAR 0 ± 1 ms; camera ≤ 2 ms | `out/r1_pose500/sync.json` |
| R2 | GUI, mô hình tĩnh | `forest_static`, 500 m, GUI | `GUI=1 scripts/static_run.sh` | 3 phút | GUI mở, RTF ≥ 0,7, RAM trống > 1 GB, số liệu như giai đoạn 1 | `out/r2_gui_static/report.md` + ảnh chụp màn hình |
| R3 | GUI, bay đủ chu trình | `forest`, 500 m, 1 tuyến, VTOL hạ cánh tại home, GUI | `GUI=1 ./run_sitl.sh` + `scripts/vtol_mission.py --lines 1` | 10 phút | Cất cánh, chuyển chế độ, bay tuyến, xuống độ cao, chuyển về MC, hạ cánh và disarm; node ghi đủ | `out/r3_gui_flight/` + log PX4 |
| R4 | Tốc độ 25 m/s | `forest`, 600 m, 1 tuyến, `AIRSPEED=25` | headless | 8 phút | Bám tuyến (lệch ngang < 5 m, roll < 3°) ở 25 m/s; nếu không đạt thì giữ 17,5 m/s cho R5 | `out/r4_25ms/flight_report.md` |
| R5 | Cấu hình chính | `forest`, 600 m, 10 tuyến cách 95 m, X = −427,5 + 95k, Y từ −650 đến 650 (+350 m vào tuyến mỗi đầu, sau R4), camera mỗi 60 m chỉ trong vùng rừng, hạ cánh | headless | ~30 phút | 10 tuyến đủ; chồng ngang ảnh ~83,7 %, chồng dọc ~84,5 % (tài liệu mục 7); mọi ô 1 m của vùng 800 × 800 m có ≥ 1 điểm LiDAR | `out/r5_main/` |
| R6 | Xuất dữ liệu | từ R5 | `scripts/export_dataset.py` | 5 phút | LAS 1.2 (gps_time, point_source_id = mã dải) đọc lại được; COLMAP (cameras/images.txt) chiếu đỉnh cây trùng ảnh; trajectory 500 Hz kèm strip_id và cờ đo/chuyển tuyến; bản đồ mật độ và độ phủ (tài liệu 9.4) | `out/r5_main/export/` |

Thứ tự là R1 → R2 → R3 → R4 → R5 → R6. Mỗi lượt chỉ chạy khi lượt trước đạt. Nếu R4 không đạt, R5 bay ở 17,5 m/s
và ghi rõ trong kết quả.

## Thay đổi cần làm trước khi chạy

- World: SceneBroadcaster `<dynamic_pose_hertz>500</dynamic_pose_hertz>`, cho pose 500 Hz (R1).
- `vtol_mission.py`: thêm `--lines`, `--spacing`, `--x0`, chiều bay xen kẽ, đoạn vào tuyến, xuống độ cao bằng vòng
  lượn rồi VTOL land (R3, R5).
- `camera_node.py`: thêm `--trigger-box`, chỉ kích chụp trong vùng rừng (R5).
- `export_dataset.py`: tách dải theo quỹ đạo; xuất LAS, COLMAP, bảng tia theo mục 8, bản đồ độ phủ (R6).
