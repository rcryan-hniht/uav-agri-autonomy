# Hardware Requirements & Tech Stack

Tài liệu đặc tả danh mục thiết bị phần cứng (Bill of Materials - BOM), sơ đồ phân bổ nguồn/tín hiệu và **Tech Stack** cho hệ thống **uav-agri-autonomy** qua hai giai đoạn: **Phase 1 (Simulation)** và **Phase 2 (Sim-to-Real Hardware)**.

---

## 1. Tech Stack 

| Tầng hệ thống (Layer) | Công nghệ / Thư viện | Vai trò trong hệ thống |
| :--- | :--- | :--- |
| **Flight Control Stack (Tier 1)** | **ArduPilot (ArduCopter)** + **MAVLink v2** | Điều khiển cân bằng mức thấp (400Hz–1000Hz Cascaded PID), hợp nhất cảm biến (EKF3: IMU + GPS + Compass + ToF Rangefinder), nhận lệnh vận tốc `SET_POSITION_TARGET_LOCAL_NED`. |
| **Simulation Stack (Phase 1)** | **ArduPilot SITL** + **Gazebo Harmonic** (`gpu_lidar`) | Mô phỏng động lực học Quadcopter, môi trường vườn cây nông nghiệp (`orchard_obstacles.sdf`), quét laser $360^\circ$ và luồng camera nhẹ phù hợp iGPU Intel UHD. |
| **Onboard Companion Core (Tier 2)** | **Python 3.11+**, `NumPy`, `pymavlink`, `pyserial` | Thực thi máy trạng thái tự hành (`AutonomyStateMachine`), bộ bù góc nghiêng IMU (`TiltCompensator`), và hai thuật toán né vật cản thời gian thực: **APF** (Mode 1) & **VFH+** (Mode 2) ở tần số 20Hz. |
| **Onboard Gateway & Streaming** | **FastAPI**, `websockets`, `uvicorn`, **OpenCV** | Cung cấp kênh truyền thông hai chiều độ trễ thấp (WebSocket JSON @ 20Hz), cơ chế Heartbeat/Watchdog an toàn (1.5s timeout), và luồng video MJPEG/HTTP. |
| **Ground Control Station (Tier 3)** | **TypeScript**, **React 19**, **Vite**, **Tauri v2**, **Leaflet**, **HTML5 Canvas** | Ứng dụng điều khiển Linux Desktop & Web siêu nhẹ: hiển thị HUD Telemetry, vẽ bản đồ Radar vật cản $360^\circ$ kèm vector vận tốc $\vec{v}_{\text{user}}$ vs. $\vec{v}_{\text{safe}}$, và chọn Waypoint trên bản đồ GPS. |
| **Crop Disease AI (Extended Module)** | **PyTorch** (Training trên Cloud GPU service) $\rightarrow$ **YOLOv8n / MobileNetV3** $\rightarrow$ **ONNX Runtime** (CPU INT8/FP16) | Nhận diện sớm dấu hiệu bệnh trên lá cây trồng (lúa, chuối, mai) tại trạm mặt đất mà không làm quá tải CPU của máy tính nhúng trên drone. |
| **DevOps & Verification** | **Docker Compose**, `pytest`, **Flight Blackbox CSV Logger** | Đóng gói môi trường mô phỏng SITL, kiểm thử đơn vị (Unit Tests $<1\text{s}$ cho Domain Logic), và ghi nhật ký chuyến bay phục vụ đánh giá định lượng Sim-vs-Real. |

---

## 2. Danh mục Thiết bị Phần cứng (Hardware Bill of Materials - BOM)

Ngân sách mục tiêu: **$\le 6,000,000\text{ VNĐ}$** (tối ưu xuống **$\sim 3,000,000\text{ VNĐ}$** khi tận dụng linh kiện sẵn có từ Phòng Lab / Không gian sáng chế).

| STT | Hạng mục linh kiện | Mã thiết bị / Thông số kỹ thuật khuyến nghị | Số lượng | Đơn giá dự kiến (VNĐ) | Vai trò & Lưu ý kỹ thuật |
| :-: | :--- | :--- | :-: | ---: | :--- |
| 1 | **Khung Drone (Frame)** | **F450** (nhựa PA66 sợi thủy tinh + PCB chia nguồn) hoặc **S500 Carbon** + Chân đáp cao | 1 bộ | 220,000 | Chịu lực cứng vững, chống rung cộng hưởng cơ khí vào IMU. **Tuyệt đối không in 3D khung chính**; chỉ in 3D gá đỡ (mount) cho LiDAR và Camera. |
| 2 | **Động cơ (Brushless Motor) + Cánh** | Combo 4x **Brushless 2212 920KV** + 8 cánh **1045 (10 inch)** | 1 bộ | 550,000 | Tạo lực nâng cho tổng trọng lượng cất cánh (AUW) khoảng $1.2\text{ kg} - 1.4\text{ kg}$ (bao gồm 4 cánh dự phòng). |
| 3 | **Điều tốc điện tử (ESC)** | 4x **ESC 30A BLHeli_S** (hoặc ESC 4-in-1 35A–45A) | 1 bộ | 400,000 | Điều khiển tốc độ động cơ theo tín hiệu PWM/DShot từ Flight Controller. |
| 4 | **Flight Controller (FC)** | **SpeedyBee F405 V3/V4** hoặc **Pixhawk 2.4.8** | 1 cái | 1,100,000 | Chứa IMU (Gyro/Accel), Barometer; chạy firmware **ArduCopter**; giao tiếp MAVLink v2 với Companion Computer qua UART (`921600 baud`). |
| 5 | **Định vị GPS + La bàn từ** | Module **M10 / M8N GPS** tích hợp Compass **QMC5883L** + Cọc chống nhiễu (GPS Mast) | 1 bộ | 320,000 | Cung cấp tọa độ toàn cục và góc phương vị (Heading) cho chế độ `LOITER` và `GUIDED` ngoài ruộng/vườn. Đặt cách xa dây nguồn $\ge 10\text{ cm}$. |
| 6 | **Cảm biến độ cao mặt đất** | Laser ToF Rangefinder **VL53L1X (4m, I2C)** | 1 cái | 80,000 | Hướng thẳng xuống đất để giữ độ cao bay tầm thấp ($1.5\text{m} - 2.0\text{m}$) chính xác hơn áp kế (Barometer). |
| 7 | **Cảm biến né vật cản 360°** | **2D Planar LiDAR LDRobot LD19** (DTOF 12m, kháng nắng 30k Lux) hoặc **RPLIDAR A1M8** | 1 cái | 450,000 | Quét laser $360^\circ$ mặt phẳng ngang ở tần số $10\text{Hz}$, đo khoảng cách chính xác tới thân cây chuối, cành mai, cột trụ với tải CPU $<1\%$. |
| 8 | **Companion Computer** | **Raspberry Pi 4B (2GB/4GB)** hoặc **Orange Pi** + Thẻ MicroSD 32GB A2 | 1 bộ | 1,000,000 | Chạy Linux, đọc UART LiDAR + Camera, chạy `drone-core` (APF/VFH+ 20Hz) và phát Wi-Fi WebSocket về trạm mặt đất. *(Ưu tiên mượn từ Lab IoT)*. |
| 9 | **Camera quan sát** | **Pi Camera CSI** hoặc **USB Webcam 720p** siêu nhẹ | 1 cái | 150,000 | Truyền hình ảnh trực tiếp về Ground Control App và chụp mẫu lá cây phục vụ chẩn đoán bệnh bằng ONNX. |
| 10 | **Tay cầm RC + Đầu thu RX** | **FlySky FS-i6 / FS-i6X** + Receiver **FS-iA6B (iBUS)** | 1 bộ | 650,000 | Kênh điều khiển độc lập 2.4GHz; dùng để bay thử cơ bản và làm công tắc **Emergency Override (chiếm quyền sang `LOITER` trong 0.1s)**. *(Ưu tiên mượn Lab)*. |
| 11 | **Hệ thống Nguồn & Sạc** | Pin **LiPo 3S 3300mAh 35C** + Sạc cân bằng B3/B6 + **Mạch hạ áp UBEC 5V/5A độc lập** + Còi báo pin | 1 gói | 750,000 | **Bắt buộc dùng mạch UBEC 5V/5A riêng** để cấp nguồn cho Raspberry Pi + LiDAR, tuyệt đối không rút dòng 5V từ BEC của Flight Controller để tránh sụt áp gây reset FC trên không. |
| **TỔNG** | **Phương án tự trang bị 100%** | | | **5,670,000 VNĐ** | Đạt yêu cầu ngân sách $\le 6\text{ triệu VNĐ}$. |
| **TỔNG** | **Phương án mượn Pi + RC/FC từ Lab** | | | **~3,000,000 VNĐ** | Tối ưu chi phí, dành dư ngân sách cho linh kiện dự phòng (cánh quạt, pin phụ). |

---

## 3. Nguyên tắc Đấu nối & An toàn Điện tử (Hardware Integration Rules)

1. **Cách ly nguồn cấp (Power Isolation):**
   - Pin LiPo $\rightarrow$ Mạch chia nguồn PDB $\rightarrow$ Cấp nguồn công suất cho 4 ESC và cấp $5\text{V}$ nội cho riêng Flight Controller.
   - Từ PDB trích một nhánh $12\text{V}$ qua **UBEC 5.1V/5A độc lập** để nuôi **Raspberry Pi 4** và motor quay của **2D LiDAR**.
2. **Đấu nối giao tiếp nối tiếp (UART & I2C Mapping):**
   - `Flight Controller UART (TELEM1/TELEM2)` $\leftrightarrow$ `Raspberry Pi GPIO UART (/dev/ttyAMA0 @ 921600 baud)`: nối chéo `TX -> RX`, `RX -> TX`, và chung `GND`.
   - `2D LiDAR LD19 / RPLIDAR A1` $\rightarrow$ `Raspberry Pi USB (/dev/ttyUSB0 @ 230400/115200 baud)`.
   - `ToF VL53L1X` + `Compass QMC5883L` $\rightarrow$ Bus `I2C` trên Flight Controller.
3. **Bố trí cơ khí (Mechanical Placement):**
   - Đặt **2D LiDAR** trên tầng cao nhất của khung drone (không bị cánh quạt hay cọc GPS che khuất tầm quét $360^\circ$ phía trước/hai bên).
   - Dán đệm xốp chống rung 3M dưới Flight Controller; xoắn đôi (twisted pair) dây nguồn từ pin để giảm nhiễu từ trường lên la bàn.
