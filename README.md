# uav-agri-autonomy

**Tiếng Việt** | [English](README-us.md)

**Nghiên cứu và phát triển hệ thống điều khiển và tự hành né tránh vật cản cho thiết bị bay không người lái (UAV) hướng tới ứng dụng giám sát nông nghiệp thông minh.**

*Research and Development of an Autonomous Obstacle Avoidance and Control System for Unmanned Aerial Vehicles (UAVs) Toward Smart Agriculture Monitoring.*

---

## 1. Tổng quan

`uav-agri-autonomy` là dự án nghiên cứu triển khai Hệ thống Tự hành Phân cấp 3 Tầng (3-Tier Hierarchical Autonomy System) cho thiết bị bay Quadcopter hoạt động trong môi trường nông nghiệp bán cấu trúc (ruộng lúa, vườn cây ăn trái, vườn cảnh).

Hệ thống được thiết kế theo **Kiến trúc Lục giác (Hexagonal / Ports and Adapters Architecture)** nhằm đảm bảo toàn bộ tầng nghiệp vụ cốt lõi (`AutonomyStateMachine`, `TiltCompensator`, `APFAvoidanceController`, và `VFHPlusPlanner`) hoàn toàn độc lập với môi trường thực thi:
- **Giai đoạn 1 (Trọng tâm hiện tại — Mô phỏng & Kiến trúc Phần mềm):** Nghiên cứu, xây dựng và kiểm chứng định lượng toàn bộ thuật toán né tránh vật cản cùng giao diện điều khiển trên môi trường mô phỏng vật lý (**ArduPilot SITL + Gazebo Harmonic**) qua giao thức UDP MAVLink (`udpin:0.0.0.0:14550`).
- **Giai đoạn 2 (Định hướng mở rộng — Tích hợp Phần cứng Thực tế):** Chuyển đổi hệ thống từ môi trường mô phỏng sang nguyên mẫu UAV vật lý thông qua việc gắn thêm các bộ chuyển đổi (Hardware Adapters) tương thích với giao diện trừu tượng sẵn có mà không cần viết lại tầng thuật toán cốt lõi.

### Danh mục Tài liệu
- [Đặc tả Yêu cầu Phần mềm & Tech Stack](docs/requirements/software.md)
- [Yêu cầu Phần cứng & BOM Dự kiến (Định hướng mở rộng)](docs/requirements/hardware.md)
- [Lộ trình Phát triển Giai đoạn 1 (Mô phỏng & Kiến trúc Phần mềm)](docs/dev/phases/phase_1.md)
- [Sơ đồ Kiến trúc Tương tác Archify — Phase 1 (Simulation)](architecture/phase/phase_1/phase_1.html)

---

## 2. Kiến trúc Hệ thống Chi tiết (Phase 1)

### 2.1. Kiến trúc Tự hành Phân cấp 3 Tầng (3-Tier Hierarchical Autonomy)

```mermaid
flowchart TB
    subgraph Tier3["TẦNG 3: TRẠM ĐIỀU KHIỂN MẶT ĐẤT & AI (Linux Desktop / Web)"]
        UI["Ứng dụng Điều khiển TypeScript<br/>(React 19 + Vite + Tauri v2)"]
        RadarWidget["Bản đồ Radar 2D LiDAR & HUD Canvas<br/>(LidarRadarCanvas.tsx)"]
        MapWidget["Bản đồ Tọa độ Waypoint GPS<br/>(Leaflet - MissionMap.tsx)"]
        CropAI["Dịch vụ AI Nhận diện Bệnh Cây trồng<br/>(ONNX Runtime - YOLOv8n INT8)"]
        UI --- RadarWidget
        UI --- MapWidget
        UI <-->|"HTTP REST (Phân tích Khung hình)"| CropAI
    end

    subgraph Tier2["TẦNG 2: LÕI TỰ HÀNH ONBOARD COMPANION (Docker Container / Edge Process)"]
        Gateway["Cổng Giao tiếp Companion Gateway<br/>(FastAPI WebSocket :8000 + MJPEG)"]
        FSM["Máy Trạng thái Tự hành (FSM)<br/>(MANUAL | ASSISTED_AVOID | AUTO_WAYPOINT | EMERGENCY_BRAKE)"]
        Perception["Bộ Bù Góc nghiêng & Lọc Mặt đất<br/>(Roll/Pitch Projection)"]
        Avoidance["Bộ Thuật toán Né vật cản Kép<br/>(Trường thế APF + VFH+ 72 Góc quạt)"]
        LidarPort["Bộ chuyển đổi ILidarPort<br/>(SimGazeboLidarAdapter)"]
        MavBridge["Bộ chuyển đổi IFlightBridgePort<br/>(MavlinkFlightBridge)"]
        Blackbox["Hộp đen Ghi Nhật ký Chuyến bay<br/>(logs/flight_blackbox.csv)"]

        Gateway <-->|"Lệnh điều khiển & Telemetry"| FSM
        Gateway -->|"Ghi chỉ số thực nghiệm"| Blackbox
        LidarPort -->|"Dữ liệu quét 360° thô (r, theta)"| Perception
        Perception -->|"Mảng khoảng cách ngang đã lọc"| FSM
        FSM <-->|"Tính toán Vận tốc An toàn"| Avoidance
        Avoidance -->|"VelocityCommand (vx, vy, vz, yaw_rate)"| MavBridge
        MavBridge -->|"Góc nghiêng (Roll/Pitch/Yaw) & Độ cao"| Perception
        MavBridge -->|"DroneState"| Gateway
    end

    subgraph Tier1["TẦNG 1: ĐIỀU KHIỂN BAY & CẢM BIẾN (Môi trường Mô phỏng SITL / Phần cứng mở rộng)"]
        FC["Bộ Điều khiển Bay ArduPilot<br/>(ArduCopter SITL)"]
        LidarSim["Cảm biến Quét Laser 2D 360°<br/>(Gazebo gpu_lidar Plugin)"]
        Dynamics["Mô hình Động lực học Quadcopter<br/>(IMU + GPS + Rangefinder + Môi trường Vườn cây)"]

        FC <--> Dynamics
    end

    UI <==>|"WebSocket JSON (Telemetry & Lệnh 20Hz)"| Gateway
    LidarSim -->|"Luồng dữ liệu quét 360° (10Hz)"| LidarPort
    MavBridge <==>|"MAVLink v2 (UDP :14550)"| FC
```

### 2.2. Chu trình Điều khiển Thời gian thực & Luồng Dữ liệu (Sequence Diagram)

```mermaid
sequenceDiagram
    autonumber
    participant Operator as Ứng dụng Điều khiển (TypeScript)
    participant Gateway as Companion Gateway (FastAPI)
    participant FSM as Máy trạng thái & Bộ Né vật cản
    participant Lidar as Bộ chuyển đổi 2D LiDAR (ILidarPort)
    participant FC as Flight Controller (ArduPilot SITL)

    loop Mỗi 100ms (Thu thập Cảm biến 10Hz)
        Lidar->>FSM: 1. Đọc mảng 360 khoảng cách [r_0 ... r_359]
        FC->>FSM: 2. Đọc góc tư thế (roll, pitch, yaw) và độ cao (h)
        Note over FSM: 3. TiltCompensator chiếu tia laser về mặt phẳng ngang<br/>và loại bỏ tia chạm đất khi r * sin(tilt) >= h - tol
    end

    loop Mỗi 50ms (Vòng lặp Điều khiển & Telemetry 20Hz)
        Operator->>Gateway: 4. Gửi vận tốc mong muốn hoặc tọa độ Waypoint (lat, lon)
        Gateway->>FSM: 5. step(raw_scan, drone_state, user_vx, user_vy, goal_dx, goal_dy)
        Note over FSM: 6. Thực thi chế độ hiện hành:<br/>- ASSISTED_AVOID: Trường lực đẩy APF + Phanh giảm tốc động<br/>- AUTO_WAYPOINT: Biểu đồ cực VFH+ 72 góc quạt
        FSM->>FC: 7. Gửi MAVLink SET_POSITION_TARGET_LOCAL_NED (vx_safe, vy_safe, vz, yaw_rate)
        FSM->>Gateway: 8. Trả về VelocityCommand + closest_obstacle_m + active_sectors
        Gateway->>Operator: 9. Phát gói tin WebSocket TelemetrySnapshot JSON
    end
```

### 2.3. Sơ đồ Lớp Kiến trúc Lục giác (Hexagonal Domain & Adapters)

```mermaid
classDiagram
    class AutonomyStateMachine {
        +AutonomyMode mode
        +TiltCompensator compensator
        +APFAvoidanceController apf
        +VFHPlusPlanner vfh
        +set_mode(new_mode)
        +step(raw_scan, drone_state, user_vx, user_vy, goal_dx, goal_dy) VelocityCommand
    }

    class TiltCompensator {
        +float ground_tolerance_m
        +float max_range_m
        +compensate(scan, state) LaserScan360
    }

    class APFAvoidanceController {
        +float warn_dist_m
        +float stop_dist_m
        +float k_repulsive
        +compute_assisted_velocity(scan, user_vx, user_vy, user_vz) VelocityCommand
    }

    class VFHPlusPlanner {
        +int num_sectors
        +float safe_radius_m
        +plan_toward_goal(scan, goal_dx_m, goal_dy_m) VelocityCommand
    }

    class ILidarPort {
        <<interface>>
        +get_latest_scan() LaserScan360
        +is_healthy() bool
    }

    class IFlightBridgePort {
        <<interface>>
        +get_drone_state() DroneState
        +send_velocity_ned(cmd)
        +arm_and_takeoff(target_altitude_m) bool
    }

    AutonomyStateMachine *-- TiltCompensator
    AutonomyStateMachine *-- APFAvoidanceController
    AutonomyStateMachine *-- VFHPlusPlanner
    ILidarPort <|.. SimGazeboLidarAdapter : Phase 1 (Mô phỏng)
    ILidarPort <|.. HardwareLidarAdapter : Phase 2 (Định hướng mở rộng)
    IFlightBridgePort <|.. MavlinkFlightBridge : MAVLink v2 Bridge
```

---

## 3. Cấu trúc Thư mục (Repository Structure)

```text
uav-agri-autonomy/
├── README.md                               # Tài liệu tổng quan & thiết kế hệ thống (Tiếng Việt)
├── README-us.md                            # Tài liệu tổng quan & thiết kế hệ thống (Tiếng Anh)
├── docker-compose.sim.yml                  # Khởi chạy môi trường mô phỏng ArduPilot SITL + Companion Core
├── architecture/
│   └── phase/
│       └── phase_1/
│           └── phase_1.html                # Sơ đồ kiến trúc tương tác Archify cho Phase 1
├── configs/
│   ├── simulation.yaml                     # Cấu hình mô phỏng Phase 1 (UDP :14550 + Gazebo LiDAR)
│   └── avoidance_params.yaml               # Tham số thuật toán APF, VFH+ và bù góc nghiêng
├── docs/
│   ├── requirements/
│   │   ├── hardware.md                     # Đặc tả phần cứng dự kiến & Tech Stack
│   │   └── software.md                     # Đặc tả yêu cầu phần mềm, FR/NFR & Tech Stack
│   └── dev/
│       └── phases/
│           └── phase_1.md                  # Phân rã công việc & tiêu chí hoàn thành Phase 1
├── drone-core/                             # Gói phần mềm tự hành cốt lõi (Python)
│   ├── pyproject.toml
│   ├── scripts/
│   │   └── plot_blackbox_metrics.py        # Phân tích định lượng & xuất chỉ số từ flight_blackbox.csv
│   ├── src/
│   │   ├── config.py                       # Composition Root: Nạp YAML & tiêm phụ thuộc (DI)
│   │   ├── domain/                         # Nghiệp vụ cốt lõi thuần túy (Độc lập phần cứng)
│   │   │   ├── models.py                   # LaserScan360, DroneState, VelocityCommand
│   │   │   ├── tilt_compensator.py         # Chiếu góc Roll/Pitch & loại bỏ tia chạm đất
│   │   │   ├── apf_avoidance.py            # Mode 1: Bộ điều khiển Trường thế Nhân tạo (APF)
│   │   │   ├── vfh_planner.py              # Mode 2: Bộ hoạch định VFH+ 72 góc quạt
│   │   │   └── state_machine.py            # Chuyển đổi chế độ bay & tổng hợp lệnh an toàn
│   │   ├── ports/                          # Giao diện trừu tượng (Ports)
│   │   │   ├── lidar_port.py
│   │   │   └── flight_bridge_port.py
│   │   ├── adapters/                       # Các Adapter mô phỏng cho Phase 1
│   │   │   ├── mavlink_bridge.py
│   │   │   └── sim_lidar_adapter.py
│   │   ├── api/                            # Máy chủ FastAPI WebSocket & REST Gateway
│   │   │   └── server.py
│   │   └── logger/                         # Bộ ghi nhật ký hộp đen chuyến bay (.csv)
│   │       └── flight_logger.py
│   └── tests/                              # Kiểm thử đơn vị cho tầng Domain
│       └── test_avoidance_domain.py
├── crop-ai-service/                        # Mô-đun mở rộng: Nhận diện bệnh cây trồng (Edge AI / ONNX)
│   ├── models/
│   │   └── labels.json                     # Danh mục lớp bệnh lá cây & cấu hình ngưỡng NMS
│   └── inference_onnx.py                   # Pipeline tiền xử lý Letterbox, suy luận ONNX CPU & NMS
├── ground-control-app/                     # Ứng dụng Trạm điều khiển Linux Desktop & Web (TypeScript)
│   ├── package.json
│   └── src/
│       ├── components/
│       │   ├── LidarRadarCanvas.tsx        # Vẽ bản đồ Radar 360° & vector vận tốc trực quan
│       │   └── MissionMap.tsx              # Bản đồ tọa độ Waypoint GPS (Leaflet)
│       ├── hooks/
│       │   └── useDroneSocket.ts           # Hook kết nối WebSocket thời gian thực 20Hz
│       └── types/
│           └── protocol.ts                 # Định nghĩa cấu trúc gói tin đồng bộ với Backend
└── simulation-worlds/
    └── orchard_obstacles.sdf               # Môi trường vườn cây nhẹ trong Gazebo phục vụ kiểm thử
```

---

## 4. Hướng dẫn Khởi chạy & Định hướng Phát triển

### 4.1. Chạy Kiểm thử Đơn vị (Domain Unit Tests — Phase 1)
Các thuật toán né tránh vật cản cốt lõi (`TiltCompensator`, `APFAvoidanceController`, `VFHPlusPlanner`) không phụ thuộc vào thư viện ngoài và có thể kiểm chứng trực tiếp bằng Python 3:

```bash
cd drone-core
PYTHONPATH=. python3 -c "import tests.test_avoidance_domain as t; t.test_tilt_compensation_rejects_ground_hit(); t.test_apf_repulses_forward_obstacle(); t.test_vfh_bypasses_front_obstacle(); print('ALL DOMAIN TESTS PASSED')"
```

### 4.2. Trọng tâm Phase 1 và Hướng Mở rộng sang Phase 2
- **Giai đoạn 1 (Hiện tại):** Tập trung hoàn thiện và đánh giá thuật toán trên môi trường mô phỏng (`configs/simulation.yaml`) với bộ chuyển đổi `SimGazeboLidarAdapter` và kết nối MAVLink qua UDP.
- **Giai đoạn 2 (Định hướng tiếp theo):** Nhờ thiết kế tách biệt qua `ILidarPort` và `IFlightBridgePort`, toàn bộ phần lõi thuật toán và giao diện điều khiển từ Giai đoạn 1 được giữ nguyên khi mở rộng sang thử nghiệm trên hệ thống nhúng thực tế ở giai đoạn sau.
