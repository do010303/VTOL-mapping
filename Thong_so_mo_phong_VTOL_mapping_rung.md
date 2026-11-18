# Yêu cầu mô phỏng VTOL thu dữ liệu LiDAR và camera để lập bản đồ rừng

Ngày cập nhật và đối chiếu nguồn hãng: **12/11/2026**.

Tài liệu dùng để bàn giao cho người xây dựng môi trường mô phỏng trong đề tài nghiên cứu thực tập. Mục tiêu là có cảnh rừng, cho VTOL bay qua cảnh, thu dữ liệu cảm biến, sau đó dùng dữ liệu đó chạy thuật toán dựng bản đồ và đánh giá kết quả.

**Cấu hình tham chiếu được chọn gồm JOUAV CW‑30E, RIEGL VUX‑240²⁴, Applanix AP+50 và Sony ILX‑LR1 với ống kính Sony FE 35 mm F2.8 ZA.** Tiêu chí lựa chọn ưu tiên chất lượng dữ liệu ở 500–600 m AGL, tầm đo, độ phân giải ảnh, khả năng đồng bộ và tải mang. Đây là lựa chọn kỹ thuật cho kịch bản nghiên cứu hiện tại. Việc xác định thiết bị tối ưu cho mua sắm còn phụ thuộc ngân sách, độ chính xác đầu ra và cấu hình tích hợp thực tế.

## 1. Những thông tin cần bàn giao trước khi xây dựng

| Hạng mục | Thông tin cần gửi | Mục đích |
|---|---|---|
| Cảnh rừng | Kích thước cảnh, địa hình, chiều cao cây, mật độ và khe tán, hình học lá/cành, texture, đơn vị | Quyết định tia LiDAR nhìn thấy gì và ảnh có đủ đặc trưng để ghép hay không |
| LiDAR | Kiểu quét, hướng quét, góc quét, số đường/s, số tia/đường, lịch phát tia, giới hạn tầm, mô hình phản hồi | Sinh phép đo cự ly và góc theo thời gian |
| Camera và ống kính | Kích thước cảm biến, tiêu cự, kích thước ảnh, nội tại K, méo ảnh, hướng nhìn, nhịp chụp | Sinh ảnh có hình học và độ phủ xác định |
| Quỹ đạo UAV | Vị trí theo thời gian, độ cao AGL, tốc độ, hướng bay, tư thế, khoảng tuyến, đường chuyển tuyến | Gắn vị trí và hướng cho từng phép đo |
| Lắp đặt cảm biến | Phép biến đổi từ hệ LiDAR và camera sang hệ thân UAV | Tính tọa độ điểm và chiếu điểm lên ảnh |
| Đồng bộ | Một đồng hồ mô phỏng chung, timestamp từng tia và từng ảnh | Ghép đúng phép đo với tư thế UAV |
| Đầu ra | Phép đo gốc, ảnh, quỹ đạo, cấu hình, dữ liệu chuẩn | Chạy lại thuật toán và định lượng sai số |

Các bảng tiếp theo ghi riêng thông số hãng, giá trị tính toán và giá trị giả định cho lần mô phỏng đầu.

## 2. LiDAR được chọn — RIEGL VUX‑240²⁴

### Thông số thiết bị

| Thông số | Giá trị hãng công bố |
|---|---|
| Cơ chế | Gương đa giác quay, đường quét song song |
| FOV tối đa | 75°, tương ứng ±37,5° |
| Tốc độ quét | 40–600 đường/s |
| PRR | 300 / 600 / 1.200 / 1.800 / 2.400 kHz |
| Tầm xiên ở phản xạ ≥20% | 1.200 m tại 300 kHz, 880 m tại 600 kHz, 640 m tại 1.200 kHz |
| Tầm tối thiểu | 5 m |
| Accuracy / precision | 20 / 15 mm, theo điều kiện thử ở 150 m |
| Độ mở tia | 0,35 mrad |
| Phản hồi tối đa tại 600 kHz | 24 mục tiêu/xung |
| Kích thước máy quét | 292 × 162 × 185 mm |
| Khối lượng máy quét | Khoảng 4,3 kg |
| Nguồn và công suất điển hình | 18–34 V DC, 77 W, chưa gồm GNSS/INS ngoài và camera |
| Đồng bộ | PPS và thời gian GNSS, giao tiếp camera trigger/exposure |

Tầm công bố phụ thuộc phản xạ, kích thước mục tiêu và điều kiện đo. Trong rừng, năng lượng một xung có thể phân bố lên nhiều bề mặt. Số phản hồi thực tế phụ thuộc cảnh và khả năng phát hiện của cảm biến. Nguồn gồm bảng kỹ thuật và ghi chú của bản datasheet ngày 17/08/2026. [RIEGL VUX‑240²⁴](https://www.riegl.com/index.php?eID=downloadFile&f=690&token=5a5ad94afb715117eaff464516a7c172bd022db7)

### Giá trị đưa vào mô phỏng lý tưởng

| Trường | Thiết lập khởi đầu | Căn cứ |
|---|---|---|
| Hướng | Nadir, quét ngang tuyến bay | Lựa chọn mô phỏng |
| Góc quét sử dụng | −20° đến +20° | Theo kịch bản báo cáo |
| Chế độ phần cứng tham chiếu | 600 kHz | Cân bằng mật độ và biên tầm ở 600 m |
| Số đường quét | 200 đường/s | Lựa chọn trong dải thiết bị hỗ trợ |
| Số tia trong mỗi đường | 1.333 tia, phân bố đều theo góc | Xấp xỉ lấy mẫu, cần ghi trong cấu hình |
| Tốc độ tia hữu dụng | 266.600 tia/s | 1.333 × 200 |
| Bước góc xấp xỉ | 0,030° | 40° / 1.332 khoảng |
| Lịch phát tia trong một đường | Khoảng cách thời gian 1/600.000 s, cửa sổ quét khoảng 2,22 ms trong chu kỳ 5 ms | Mô hình thời gian giả định |
| Giới hạn khoảng cách | 5–880 m | Giới hạn hình học của lần chạy lý tưởng |
| Nhiễu cự ly và góc | 0 | Giả định lý tưởng |
| Mô hình tia lần đầu | Tia hình học mảnh, lấy giao điểm nhìn thấy gần nhất | Đủ kiểm tra quỹ đạo, tái dựng và phân loại |

Số tia 1.333/đường được chọn để tạo một mô hình lấy mẫu xác định. Nó gần với ước tính 600.000 × (5/6) × (40/75) / 200, trong đó 5/6 và phân bố đều theo góc là giả định. Cần đối chiếu lịch quét thật nếu sau này đánh giá độ trung thực với cảm biến. Mỗi tia phải sử dụng vị trí và tư thế UAV tại thời điểm phát tia của chính nó.

Với tia mảnh, tán cây phải có khe hở hình học để một phần tia tới đất. Nếu dùng bề mặt tán kín, phần đất bị che sẽ thiếu quan sát dù phép đo không có nhiễu. Khi bổ sung nhiều phản hồi, có thể dùng các tia con trong footprint và mô hình năng lượng/ngưỡng phát hiện. Cần xử lý che khuất trên từng tia con, thay vì coi mọi giao điểm phía sau một bề mặt đặc đều là phản hồi hợp lệ.

Độ mở tia cho footprint danh nghĩa khoảng 0,21 m ở cự ly 600 m. Lần chạy tia mảnh lưu thông số footprint để tham chiếu. Muốn kiểm tra khả năng nhìn qua tán hoặc đặc tính nhiều phản hồi, cần đưa footprint và vật liệu vào mô hình cảm biến.

## 3. Camera được chọn — Sony ILX‑LR1 và Sony SEL35F28Z

### Thông số thiết bị

| Thành phần | Thông số |
|---|---|
| Thân máy | Sony ILX‑LR1, CMOS full-frame khoảng 61 MP |
| Kích thước cảm biến | 35,7 × 23,8 mm |
| Khối lượng và kích thước thân | 243 g, 100 × 74 × 42,5 mm |
| Màn trập | Tiêu diện, cơ 1/4000–30 s, điện tử 1/8000–30 s |
| Điện cấp | 10–18 V DC |
| Ảnh full-frame 3:2 | 9.504 × 6.336 px ở kích thước L |
| Điều khiển ngoài | FOCUS, TRIGGER và tín hiệu EXPOSURE |
| Ống kính | Sonnar T* FE 35 mm F2.8 ZA, mã SEL35F28Z |
| Thông số ống kính | Tiêu cự 35 mm, khẩu tối đa f/2.8, ngàm Sony E |
| Khối lượng và kích thước ống | 120 g, đường kính × dài 61,5 × 36,5 mm |
| Tổng thân và ống kính | 363 g, chưa tính cáp và gá |

Nguồn hãng cho từng nhóm thông số. [Thân máy](https://helpguide.sony.net/ilc/2390/v1/en/contents/221h_specifications.html) · [Kích thước ảnh](https://helpguide.sony.net/ilc/2390/v1/en/contents/0404M_jpeg_image_size.html) · [Điều khiển và phơi sáng](https://helpguide.sony.net/ilc/2390/v1/en/contents/231h_power_control_terminal.html) · [Ống kính](https://www.sony.com/electronics/support/lenses-e-mount-lenses/sel35f28z/specifications)

### Nội tại và mô hình tạo ảnh

| Trường | Giá trị mô phỏng |
|---|---|
| Mô hình | Pinhole, ảnh rectilinear, full-frame |
| Hướng | Nadir, cạnh dài ảnh ngang tuyến bay |
| Độ phân giải | 9.504 × 6.336 px |
| Tiêu cự theo pixel | fx = fy ≈ 9.317,647 px |
| Tâm ảnh | cx = 4.751,5 px, cy = 3.167,5 px, pixel bắt đầu từ 0 |
| FOV ngang / dọc | Khoảng 54,043° / 37,556° |
| Méo ảnh | k1 = k2 = k3 = p1 = p2 = 0 |
| Phơi sáng lý tưởng | Toàn ảnh lấy tại cùng một thời điểm, không nhòe do chuyển động |
| Kích chụp | Mỗi 60 m trên đoạn thu dữ liệu |
| Nhịp chụp ở 25 m/s | 2,4 s/ảnh |

Các giá trị nội tại trên được tính từ kích thước cảm biến, tiêu cự danh nghĩa và độ phân giải. Chúng là tham số của camera ảo. Với camera thật, dùng kết quả hiệu chuẩn của chính bộ thân máy và ống kính. Mô hình phơi sáng đồng thời là giả định của lần mô phỏng lý tưởng. Màn trập của ILX‑LR1 được hãng mô tả là màn trập tiêu diện.

```text
K = [[9317.647,    0.000, 4751.500],
     [   0.000, 9317.647, 3167.500],
     [   0.000,    0.000,    1.000]]
```

Có thể dùng ảnh 4.752 × 3.168 px cho bước kiểm tra ban đầu và giữ cùng FOV. Khi đó fx = fy ≈ 4.658,824 px, cx = 2.375,5 px, cy = 1.583,5 px. Cần lưu đúng nội tại theo ảnh thực xuất. GSD tăng gấp đôi khi giảm một nửa số pixel theo mỗi chiều.

## 4. UAV cần những thông số gì?

### Mẫu tham chiếu JOUAV CW‑30E

| Thông số hãng | Giá trị |
|---|---|
| Kiểu máy bay | VTOL cánh bằng |
| Chiều dài thân | 2,4 m |
| Sải cánh | 4,4 m |
| Khối lượng cất cánh tối đa | 38 kg |
| Tải hữu ích tối đa | 8 kg |
| Tốc độ hành trình | 90 km/h, tương đương 25 m/s |
| Thời lượng công bố | Tối đa 480 phút, phụ thuộc điều kiện và cấu hình |

Mức tải 8 kg và thời lượng 480 phút là các thông số riêng. Thời lượng thực với cụm cảm biến đề xuất cần được xác nhận theo cấu hình bay. Khối lượng cất cánh thực phải gồm thân, nguồn năng lượng, cảm biến và phụ kiện. [JOUAV CW‑30E](https://www.jouav.com/products/cw-30e.html)

Với AP+50, datasheet RIEGL nêu khối lượng cụm scanner/GNSS‑INS khoảng 4,9 kg. Tổng với thân camera và ống kính là khoảng **5,263 kg**, để lại biên tải danh nghĩa **2,737 kg** cho các hạng mục còn lại trong giới hạn 8 kg. Cần cân cả bộ, kiểm tra điện cấp và trọng tâm khi triển khai thật. [Cấu hình GNSS/INS trong datasheet RIEGL, trang 6](https://www.riegl.com/index.php?eID=downloadFile&f=690&token=5a5ad94afb715117eaff464516a7c172bd022db7)

### Khi UAV bay theo quỹ đạo định trước để tạo dữ liệu

| Cần khai báo | Thiết lập hoặc dữ liệu cần có |
|---|---|
| Hình học UAV | Mô hình 3D đơn giản theo chiều dài và sải cánh, vị trí gá cảm biến không bị thân che |
| Quỹ đạo | Vị trí XYZ và tư thế theo thời gian, có đoạn bay thẳng và đoạn chuyển tuyến |
| Độ cao | 600 m AGL cho lần đầu, thêm 500 m AGL để so sánh |
| Tốc độ | 25 m/s trong điều kiện không gió, theo tốc độ hành trình tham chiếu |
| Tư thế trên tuyến | Roll = pitch = 0 trong mô hình lý tưởng, hướng theo tuyến |
| Chuyển tuyến | Quỹ đạo nối liên tục ngoài vùng đánh giá, gắn cờ để tách đoạn đo trên tuyến |
| Đồng hồ và nội suy | Lưu pose 200 Hz hoặc dày hơn, nội suy đến timestamp của từng tia và từng ảnh |
| Thời gian chạy | Theo tổng chiều dài đường bay và tốc độ, cộng đoạn chuyển tuyến |

17 m/s được giữ làm một kịch bản so sánh với báo cáo trước. Tốc độ này là giả định nghiên cứu, cần xác nhận dải tốc độ bay phù hợp nếu dùng mô hình khí động của CW‑30E.

### Khi cần UAV tự bay theo động lực học và bộ điều khiển

Nhóm mô phỏng cần thêm các dữ liệu dưới đây từ đội UAV hoặc một mô hình máy bay đã được kiểm chứng. Trang sản phẩm công khai chưa cung cấp đủ để dựng chính xác động lực học CW‑30E.

| Nhóm | Thông số cần bổ sung |
|---|---|
| Khối lượng | Khối lượng cất cánh thực, tâm khối lượng, tensor quán tính, thay đổi do tải và nhiên liệu |
| Khí động | Diện tích cánh, dây cung tham chiếu, các hệ số lực nâng, lực cản và moment theo góc tấn, góc trượt và tốc độ góc |
| Động cơ và cánh quạt | Vị trí, trục, lực đẩy/moment theo lệnh và vận tốc, độ trễ đáp ứng, giới hạn công suất |
| Bề mặt điều khiển | Cánh lái, vị trí, giới hạn góc và tốc độ dịch chuyển, hiệu lực điều khiển |
| Giới hạn bay | Tốc độ thất tốc/khai thác, roll cho phép, tốc độ leo/hạ, giới hạn chuyển chế độ VTOL |
| Điều khiển | Autopilot và tham số, logic chuyển chế độ, giao diện lệnh waypoint, mô hình phản hồi cảm biến |
| Năng lượng | Dung lượng pin/nhiên liệu và mô hình tiêu thụ theo chế độ nếu đánh giá thời lượng |

Đề xuất khởi đầu bằng quỹ đạo định trước vì đầu ra hiện tại là dữ liệu phục vụ thuật toán mapping. Mô hình động lực học có thể bổ sung khi chuyển sang nghiên cứu chất lượng bám tuyến hoặc ảnh hưởng của chuyến bay.

## 5. GNSS/INS, hệ tọa độ và gá cảm biến

AP+50 là cấu hình tham chiếu cho giai đoạn mở rộng. Datasheet RIEGL công bố tần số IMU 200 Hz, độ chính xác sau xử lý roll/pitch 0,005°, heading 0,010° và vị trí điển hình 0,02–0,05 m. Lần đầu dùng pose chuẩn và độ lệch thời gian bằng 0. Tần số IMU công bố và tần số lưu pose mô phỏng cần được ghi riêng trong metadata. [Datasheet RIEGL, trang 6](https://www.riegl.com/index.php?eID=downloadFile&f=690&token=5a5ad94afb715117eaff464516a7c172bd022db7)

Quy ước bàn giao đề xuất như sau.

| Hệ | Quy ước |
|---|---|
| Thế giới W | ENU địa phương, X đông, Y bắc, Z lên, đơn vị mét |
| Thân B | X trước, Y trái, Z lên |
| LiDAR L | Cùng chiều trục thân trong cấu hình lý tưởng, tia trung tâm hướng xuống |
| Camera C | X sang phải ảnh, Y xuống ảnh, Z theo trục quang học |
| Thời gian | Giây từ đầu lần chạy, một đồng hồ mô phỏng chung |
| Quaternion | Ghi theo thứ tự qx, qy, qz, qw và định nghĩa rõ hệ nguồn/hệ đích |

Ký hiệu T_WB biến đổi tọa độ thân sang thế giới, T_BL biến đổi LiDAR sang thân và T_BC biến đổi camera sang thân. Phép tính điểm dùng tọa độ thuần nhất.

```text
p_W(t) = T_WB(t) × T_BL × p_L(t)
p_C(t) = inverse(T_WB(t) × T_BC) × p_W
```

Trong cấu hình khởi đầu, lấy t_BL = t_BC = (0,0,0), R_BL = I. Đây là giả định đồng vị trí của các tâm đo. Khi có bản vẽ gá, thay bằng giá trị đo hoặc thiết kế và giữ cùng cấu hình ở bộ sinh dữ liệu lẫn bộ xử lý.

Để camera nhìn xuống và cạnh dài ảnh nằm ngang tuyến bay, có thể dùng phép quay camera sang thân sau.

```text
R_BC = [[ 0, -1,  0],
        [-1,  0,  0],
        [ 0,  0, -1]]
```

Với beta dương về bên trái UAV, tia LiDAR có hướng d_L = (0, sin(beta), −cos(beta)). Điểm đo trong hệ LiDAR là p_L = range × d_L. Các góc đưa vào hàm lượng giác dùng radian. Simulator dùng hệ trục khác cần chuyển đổi nhất quán trước khi xuất dữ liệu.

## 6. Môi trường rừng cần xây dựng

| Thành phần | Yêu cầu khởi đầu |
|---|---|
| Kích thước cảnh | Đề xuất 2 × 2 km, vùng đánh giá trung tâm 800 × 800 m |
| Địa hình | Cảnh kiểm tra đất phẳng và cảnh rừng dốc nhẹ, có mesh/heightmap chuẩn |
| Thực vật | Cây cao 10–30 m, có thân/cành/lá hoặc mô hình tán có khe hở, gồm vùng trống và vùng tán dày khác nhau |
| Ảnh | Texture có biến thiên và cấu trúc, ánh sáng cố định để tạo điểm liên kết SfM |
| Vật liệu và ray tracing | Hình học nhìn thấy trong ảnh và hình học tương tác với LiDAR nhất quán, ghi rõ cách xử lý texture lá có alpha |
| Chuyển động môi trường | Gió bằng 0, cây đứng yên, ánh sáng ổn định |
| Dữ liệu chuẩn | Nhãn đất/thực vật và mã cây lấy từ cảnh, DTM/DSM/CHM chuẩn trên cùng lưới |
| Cách tạo raster chuẩn | DTM từ bề mặt đất, DSM từ bề mặt trên cùng theo phương đứng, CHM = DSM − DTM, lưới thử 1 m |

Tạo dữ liệu chuẩn trực tiếp từ cảnh để có cơ sở đánh giá độc lập với point cloud thu được. Lưu seed hoặc phiên bản cảnh nếu cây được sinh ngẫu nhiên. Nếu dùng cây billboard, cần kiểm tra ray tracing và ảnh ở các góc nhìn trước khi thu toàn cảnh.

## 7. Tuyến bay và các giá trị hình học đã tính

Camera 35 mm đặt cạnh dài ngang hướng bay. Với đất phẳng, tán cao 30 m, tiêu cự và kích thước cảm biến ở trên, các giá trị tính được như sau.

| Đại lượng | 500 m AGL | 600 m AGL |
|---|---:|---:|
| Cự ly xiên LiDAR ở 20° | 532,09 m | 638,51 m |
| Bề rộng dải LiDAR ±20° | 363,97 m | 436,76 m |
| Khoảng tuyến cho LiDAR chồng 50% | 181,99 m | 218,38 m |
| GSD ảnh trên đất phẳng, ảnh đầy đủ | 5,37 cm/px | 6,44 cm/px |
| Footprint ảnh trên đỉnh tán, ngang × dọc | 479,4 × 319,6 m | 581,4 × 387,6 m |
| Chồng ngang ảnh với khoảng tuyến 95 m | 80,18% | 83,66% |
| Chồng dọc ảnh với khoảng chụp 60 m | 81,23% | 84,52% |

Công thức dùng cho bảng là R = H/cos(alpha), W = 2H tan(alpha), GSD = H × chiều rộng cảm biến / (tiêu cự × số pixel ngang). Footprint trên tán dùng H trừ chiều cao tán. Khi địa hình hoặc độ cao tán thay đổi, tính lại độ phủ theo từng vùng.

**Cấu hình chính** dùng 600 m AGL, 25 m/s, khoảng tuyến 95 m, kích chụp mỗi 60 m và LiDAR ±20°. Chạy 3 tuyến để kiểm tra hệ thống trước. Khi nghiệm thu vùng trung tâm 800 × 800 m, dùng khoảng 10 tuyến với các đường đo kéo dài qua biên vùng đánh giá để đủ ảnh chồng phủ ở đầu và cuối tuyến. Ví dụ trong cảnh phẳng, đặt 10 tuyến tại X = −427,5 + 95k m, k từ 0 đến 9, mỗi tuyến chạy dọc Y từ −650 đến +650 m hoặc chiều ngược lại.

Các đoạn chuyển tuyến được lập ngoài vùng đánh giá, bảo đảm quỹ đạo và tư thế liên tục. Bán kính quay thuộc cấu hình quỹ đạo. Nếu kiểm chứng động lực học, dùng giới hạn được đội UAV xác nhận. Diện tích nghiệm thu là phần thực sự có đủ ảnh và LiDAR hỗ trợ sau khi kiểm tra biên phủ.

## 8. Dữ liệu cần xuất

| Tệp hoặc nhóm dữ liệu | Nội dung tối thiểu |
|---|---|
| Cấu hình JSON/YAML | Phiên bản cảnh, seed, hệ trục, đơn vị, thông số camera/LiDAR, phép biến đổi gá, mức giảm mẫu nếu có |
| Quỹ đạo CSV | t_s, x_m, y_m, z_m, qx, qy, qz, qw, strip_id, trạng thái đoạn đo/chuyển tuyến |
| Phép đo LiDAR | t_s từng tia, pulse_id, strip_id, scanline_id, góc hoặc vector hướng, range_m, cờ có/không có phản hồi |
| Nhiều phản hồi nếu hỗ trợ | return_number, number_of_returns, mô tả mô hình tạo phản hồi |
| Point cloud | LAS/LAZ hoặc PLY/PCD kèm trường thời gian, mã dải nếu định dạng hỗ trợ. Xuất metadata bổ sung khi cần |
| Ảnh RGB | PNG hoặc JPEG, tên tệp liên kết với image_id và timestamp |
| Nội tại camera | Width, height, K, mô hình méo, hệ số méo, thời điểm tham chiếu của pose ảnh |
| Tham chiếu từ cảnh | Nhãn đất/tán/mã cây, XYZ chuẩn, DTM/DSM/CHM chuẩn và mặt nạ vùng đánh giá |

Giữ bản ghi tia không nhận được phản hồi để tính tỷ lệ thu nhận. Intensity chỉ có ý nghĩa khi bộ mô phỏng định nghĩa mô hình phản xạ và thang giá trị. Trong cấu hình hình học lý tưởng có thể bỏ trường này và ghi rõ trong metadata.

Tách nhãn chuẩn khỏi tập trường đưa vào thuật toán tách đất. Pose và hiệu chuẩn chuẩn được cấp cho thuật toán theo giả định lý tưởng. Dữ liệu ảnh dùng cho SfM/MVS được lưu cùng pose chuẩn để đánh giá kết quả hoặc làm ràng buộc, tùy thiết kế thí nghiệm đã ghi.

## 9. Kiểm tra trước khi thu toàn bộ dữ liệu

1. **Hình học LiDAR.** Trên đất phẳng, tia nadir ở 600 m AGL trả cự ly 600 m. Tia 20° cho khoảng 638,51 m. Dựng lại XYZ từ phép đo và đối chiếu với giao điểm chuẩn.
2. **Tư thế và gá cảm biến.** Chọn vài vật thể dễ nhận biết, kiểm tra vị trí trong point cloud và ảnh ở cả hai chiều bay.
3. **Đồng bộ.** Mỗi tia và ảnh có timestamp hợp lệ, dùng cùng mốc thời gian. Nội suy pose đúng tại thời điểm đo.
4. **Mật độ.** Đếm số đường/s, tia/đường, tia/s và điểm phản hồi/s. Lập bản đồ mật độ và độ phủ theo ô. Nếu giảm mẫu, cập nhật con số thực tế.
5. **Ảnh.** Kiểm tra kích thước ảnh, K, hướng ảnh, khoảng chụp và độ chồng phủ bằng pose đã lưu.
6. **Đầu ra mapping.** Dùng CSF/PTD để tách đất, nội suy DTM, dựng DSM và CHM. Báo precision/recall/F1 lớp đất, bias/RMSE bản đồ và tỷ lệ ô có dữ liệu trên cùng vùng đánh giá.

Dung sai kiểm tra XYZ cần đặt theo độ chính xác số và thuật toán giao tia của simulator. Nếu ảnh bị giảm kích thước hoặc LiDAR bị giảm số tia để tăng tốc, ghi cấu hình đó trong bảng kết quả. Simulator có thể chạy chậm hơn thời gian thực và vẫn giữ đúng thời gian mô phỏng.

Các lựa chọn giả định đã chốt để bắt đầu gồm quỹ đạo định trước, không gió, pose chính xác, camera pinhole phơi sáng tức thời, tia LiDAR mảnh, 200 đường/s và 1.333 tia/đường. Đây là phạm vi kiểm chứng quy trình lý tưởng. Đánh giá tầm thu thực trong rừng, nhiều phản hồi, nhiễu và động lực học cần các mô hình bổ sung tương ứng.
