# Software Requirements & System Specification

Tài liệu đặc tả yêu cầu phần mềm (Functional & Non-Functional Requirements), ngăn xếp công nghệ (**Tech Stack**), kiến trúc giao thức truyền thông và môi trường phát triển cho hệ thống **uav-agri-autonomy** (trọng tâm triển khai ở **Phase 1 — Simulation & Software Architecture** và định hướng mở rộng sang **Phase 2**).

---

## 1. Tech Stack Phần mềm

| Phân hệ (Subsystem) | Công nghệ / Thư viện | Phiên bản khuyến nghị | Vai trò kỹ thuật |
| :--- | :--- | :--- | :--- |
| **Operating System & Container** | **Ubuntu Linux**, **Docker**, **Docker Compose** | Ubuntu 24.04 / 26.04 LTS | Môi trường phát triển và đóng gói dịch vụ mô phỏng độc lập. |
| **Flight Stack & Simulation** | **ArduPilot SITL (ArduCopter)**, **MAVProxy**, **Gazebo Harmonic** | ArduCopter 4.5+, Gazebo Harmonic | Mô phỏng động lực học bay 6-DOF, hợp nhất cảm biến (EKF3) và mô phỏng cảm biến quét laser 2D $360^\circ$ (`gpu_lidar`). |
| **Onboard Autonomy Core (`drone-core`)** | **Python**, `NumPy`, `PyYAML` | Python $\ge 3.11$, NumPy $\ge 1.26$ | Thực thi tầng nghiệp vụ độc lập phần cứng: Máy trạng thái (`AutonomyStateMachine`), bù góc nghiêng IMU (`TiltCompensator`), và thuật toán né vật cản (**APF** + **VFH+**). |
| **Flight Communication Bridge** | **`pymavlink`** (MAVLink v2) | $\ge 2.4.41$ | Truyền nhận gói tin điều khiển vận tốc (`SET_POSITION_TARGET_LOCAL_NED`) và đọc Telemetry (`ATTITUDE`, `LOCAL_POSITION_NED`, `SYS_STATUS`). |
| **Companion API & Video Gateway** | **FastAPI**, `websockets`, `uvicorn`, **OpenCV** | FastAPI $\ge 0.115$ | Cung cấp cổng WebSocket hai chiều ($20\text{Hz}$), REST API chuyển đổi chế độ bay, Watchdog ngắt an toàn và luồng video MJPEG. |
| **Ground Control Station (`ground-control-app`)** | **TypeScript**, **React 19**, **Vite**, **TailwindCSS**, **Tauri v2**, **Leaflet**, **HTML5 Canvas** | Node.js $\ge 20$, Tauri $\ge 2.0$ | Giao diện điều khiển Linux Desktop & Web siêu nhẹ: vẽ bản đồ Radar vật cản $360^\circ$ (60FPS), vector vận tốc $\vec{v}_{\text{user}}$ / $\vec{v}_{\text{safe}}$, HUD Telemetry và bản đồ Waypoint. |
| **Crop Disease AI (`crop-ai-service`)** | **PyTorch** (trên **Cloud GPU service**), **YOLOv8n / MobileNetV3**, **ONNX Runtime** | ONNX Runtime CPU (INT8/FP16) | Huấn luyện mô hình trên đám mây và xuất sang định dạng `.onnx` để suy luận tốc độ cao trên CPU trạm mặt đất. |
| **Testing & Flight Logging** | **`pytest`**, `csv`, `pandas`, `matplotlib` | `pytest` $\ge 8.0$ | Kiểm thử đơn vị tự động cho tầng Domain và xuất dữ liệu hộp đen (`logs/flight_blackbox.csv`) phục vụ đánh giá thực nghiệm. |

---

## 2. Yêu cầu Chức năng (Functional Requirements - FR)

### 2.1. Phân hệ Điều khiển & Tự hành (`drone-core`)
- **FR-01 (Quản lý Chế độ Bay - State Machine):** Hỗ trợ chuyển đổi tức thời giữa 4 trạng thái hoạt động:
  1. `MANUAL`: Truyền trực tiếp lệnh vận tốc từ người điều khiển xuống Flight Controller.
  2. `ASSISTED_AVOID` (*Mode 1 — Hỗ trợ lái chống va chạm*): Kết hợp lệnh vận tốc của người dùng với trường lực đẩy **Artificial Potential Field (APF)** và vùng giảm tốc động để ngăn va chạm.
  3. `AUTO_WAYPOINT` (*Mode 2 — Tự hành điểm-tới-điểm*): Sử dụng biểu đồ cực **VFH+ (72 góc quạt)** có mở rộng bán kính an toàn của drone để tự động lách qua vật cản trên đường bay tới tọa độ đích.
  4. `EMERGENCY_BRAKE`: Hủy mọi lệnh di chuyển và giữ vị trí (Hover tại chỗ với $\vec{v} = (0,0,0)$).
- **FR-02 (Bù góc nghiêng Cảm biến - Tilt Compensation):** Chiếu dữ liệu quét laser 2D từ hệ tọa độ thân máy (*Body Frame*) sang mặt phẳng ngang (*Horizontal Local Frame*) dựa trên góc `Roll` và `Pitch` tức thời, đồng thời loại bỏ các tia chạm đất giả khi drone chúi mũi tăng tốc.
- **FR-03 (Giám sát Kết nối An toàn - Communication Watchdog):** Tự động kích hoạt phanh an toàn (`EMERGENCY_BRAKE`) nếu không nhận được gói tin Heartbeat/Command từ ứng dụng điều khiển trong khoảng thời gian cấu hình ($1.5\text{ giây}$) khi đang ở chế độ điều khiển từ xa.
- **FR-04 (Ghi Nhật ký Hộp đen - Flight Blackbox Logging):** Tự động ghi nhận chuỗi thời gian (`timestamp_ms`, `mode`, `roll`, `pitch`, `altitude_m`, `vx_safe`, `vy_safe`, `closest_obstacle_m`, `status_tag`) ra tệp CSV để phân tích định lượng.

### 2.2. Phân hệ Trạm Điều khiển Mặt đất (`ground-control-app`)
- **FR-05 (Giám sát Telemetry & HUD):** Hiển thị thời gian thực trạng thái kết nối, chế độ bay hiện hành, phần trăm pin, độ cao, vận tốc và góc tư thế (Roll/Pitch/Yaw).
- **FR-06 (Trực quan hóa Radar Vật cản $360^\circ$):** Vẽ trực tiếp đám mây điểm khoảng cách 72 góc quạt quanh tâm drone, vòng tròn cảnh báo ($d_{\text{warn}}$), vòng tròn dừng khẩn cấp ($d_{\text{stop}}$), cùng hai vector trực quan: vận tốc mong muốn ($\vec{v}_{\text{user}}$) và vận tốc an toàn sau khi thuật toán can thiệp ($\vec{v}_{\text{safe}}$).
- **FR-07 (Điều khiển Đa phương thức):** Cho phép gửi lệnh điều khiển vận tốc qua bàn phím (`WASD` / phím mũi tên) hoặc cần điều khiển ảo, chuyển đổi chế độ bay và chọn điểm đến (Waypoint) trên bản đồ số.
- **FR-08 (Мо-đun Mở rộng — Nhận diện Bệnh Cây trồng):** Hiển thị luồng hình ảnh từ camera và cho phép gửi khung hình qua dịch vụ ONNX để khoanh vùng (Bounding Box) dấu hiệu bệnh trên lá cây.

---

## 3. Yêu cầu Phi Chức năng & Ràng buộc Kỹ thuật (Non-Functional Requirements - NFR)

| Mã NFR | Hạng mục | Chỉ số yêu cầu (Target Metric) | Phương pháp kiểm chứng |
| :--- | :--- | :--- | :--- |
| **NFR-01** | **Độ trễ tính toán né vật cản (Domain Latency)** | $\le 5\text{ ms}$ cho mỗi chu kỳ `step()` trên CPU tiêu chuẩn | Đo thời gian thực thi hàm `AutonomyStateMachine.step()` bằng `pytest` / profiler. |
| **NFR-02** | **Tần số vòng lặp điều khiển & Telemetry** | Duy trì ổn định ở $20\text{ Hz}$ (chu kỳ $50\text{ ms}$) | Kiểm tra khoảng cách `timestamp_ms` liên tiếp trong `flight_blackbox.csv`. |
| **NFR-03** | **Tối ưu tài nguyên máy phát triển (Low-Spec Friendly)** | Tổng mức tiêu thụ RAM của SITL + Gazebo (Low-spec/Headless) + `drone-core` + GCS App $\le 4.5\text{ GB}$ | Giám sát qua `htop` trên máy cấu hình CPU tích hợp (Intel UHD Graphics). |
| **NFR-04** | **Tính độc lập phần cứng (Hexagonal Portability)** | $0$ câu lệnh import thư viện mô phỏng/phần cứng (`pymavlink`, Gazebo, serial) bên trong thư mục `drone-core/src/domain/` | Kiểm tra tĩnh mã nguồn và chạy bộ Unit Test độc lập không cần bật simulator. |
| **NFR-05** | **Độ trễ suy luận AI trên CPU (ONNX Inference)** | $\le 80\text{ ms} / \text{frame}$ trên CPU ở độ phân giải $640 \times 640$ | Đo thời gian phản hồi của `CropDiseaseONNXInspector.analyze_frame()`. |

---

## 4. Yêu cầu Môi trường Phát triển (Development Environment Prerequisites)

1. **Hệ điều hành:** Linux (Ubuntu 24.04 LTS / 26.04 LTS hoặc tương đương).
2. **Công cụ hệ thống:**
   - `python3` ($\ge 3.11$), `pip` / `uv` / `venv`.
   - `nodejs` ($\ge 20.x$) & `npm` / `pnpm` (cho `ground-control-app`).
   - `docker` & `docker-compose` (tùy chọn để chạy nhanh môi trường SITL).
3. **Kiểm tra nhanh môi trường phần mềm cốt lõi:**
   ```bash
   cd drone-core
   PYTHONPATH=. python3 -c "import tests.test_avoidance_domain as t; t.test_tilt_compensation_rejects_ground_hit(); t.test_apf_repulses_forward_obstacle(); t.test_vfh_bypasses_front_obstacle(); print('SOFTWARE DOMAIN VERIFIED')"
   ```
