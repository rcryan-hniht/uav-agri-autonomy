# uav-agri-autonomy

**Tiếng Việt** | [English](README-us.md)

**Nghiên cứu và phát triển hệ thống điều khiển và tự hành né tránh vật cản cho thiết bị bay không người lái (UAV) hướng tới ứng dụng giám sát nông nghiệp thông minh.**

*Research and Development of an Autonomous Obstacle Avoidance and Control System for Unmanned Aerial Vehicles (UAVs) Toward Smart Agriculture Monitoring.*

---

## 1. Tổng quan

`uav-agri-autonomy` là một monorepo nghiên cứu kéo dài trong hai học kỳ, triển khai Hệ thống Tự hành Phân cấp 3 Tầng (3-Tier Hierarchical Autonomy System) cho thiết bị bay Quadcopter hoạt động trong môi trường nông nghiệp bán cấu trúc (ruộng lúa, vườn chuối, vườn mai).

Hệ thống được thiết kế theo **Kiến trúc Lục giác (Hexagonal / Ports and Adapters Architecture)** nhằm đảm bảo toàn bộ tầng nghiệp vụ cốt lõi (`AutonomyStateMachine`, `TiltCompensator`, `APFAvoidanceController`, và `VFHPlusPlanner`) hoàn toàn độc lập với phần cứng qua cả hai giai đoạn:
- **Giai đoạn 1 (Niên luận cơ sở ngành — Mô phỏng):** Kiểm chứng toàn diện thuật toán và kiến trúc truyền thông trên **ArduPilot SITL + Gazebo Harmonic** qua giao thức UDP MAVLink (`udpin:0.0.0.0:14550`).
- **Giai đoạn 2 (Niên luận chuyên ngành — Phần cứng thực tế):** Triển khai trực tiếp lên máy tính nhúng **Raspberry Pi 4B** kết nối với mạch điều khiển bay **SpeedyBee F405 / Pixhawk** qua cổng UART (`921600 baud`) và cảm biến **2D Planar LiDAR $360^\circ$ (LDRobot LD19 / RPLIDAR A1)**.

### Danh mục Tài liệu
- [Yêu cầu Phần cứng, Bảng dự toán BOM & Ngăn xếp Công nghệ](docs/requirements/hardware.md)
- [Lộ trình Phát triển Giai đoạn 1 (Mô phỏng & Kiến trúc Phần mềm)](docs/dev/phases/phase_1.md)
- [Lộ trình Phát triển Giai đoạn 2 (Tích hợp Phần cứng Sim-to-Real)](docs/dev/phases/phase_2.md)
- [Sơ đồ Kiến trúc Tương tác Archify HTML](.archify/architecture-uav-agri-autonomy-20261105-214900/uav-agri-autonomy.html)

---

## 2. Kiến trúc Hệ thống Chi tiết

### 2.1. Kiến trúc Tự hành Phân cấp 3 Tầng (3-Tier Hierarchical Autonomy)

```mermaid
flowchart TB
    subgraph Tier3["TẦNG 3: TRẠM ĐIỀU KHIỂN MẶT ĐẤT & AI (Linux Desktop / Web)"]
        UI["Ứng dụng Điều khiển TypeScript<br/>(React 19 + Vite + Tauri v2)"]
        RadarWidget["Bản đồ Radar 2D LiDAR & HUD Canvas<br/>(LidarRadarCanvas.tsx)"]
        MapWidget["Bản đồ Tọa độ Waypoint GPS<br/>(Leaflet)"]
        CropAI["Dịch vụ AI Nhận diện Bệnh Cây trồng<br/>(ONNX Runtime - YOLOv8n INT8)"]
        UI --- RadarWidget
        UI --- MapWidget
        UI <-->|"HTTP REST (Phân tích Khung hình)"| CropAI
    end

    subgraph Tier2["TẦNG 2: MÁY TÍNH NHÚNG TRÊN DRONE (Raspberry Pi 4B / Docker Container)"]
        Gateway["Cổng Giao tiếp Companion Gateway<br/>(FastAPI WebSocket :8000 + MJPEG)"]
        FSM["Máy Trạng thái Tự hành (FSM)<br/>(MANUAL | ASSISTED_AVOID | AUTO_WAYPOINT)"]
        Perception["Bộ Bù Góc nghiêng & Lọc Mặt đất<br/>(Roll/Pitch Projection)"]
        Avoidance["Bộ Thuật toán Né vật cản Kép<br/>(Trường thế APF + VFH+ 72 Góc quạt)"]
        LidarPort["Bộ chuyển đổi ILidarPort<br/>(Gazebo Sim / Serial LD19)"]
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

    subgraph Tier1["TẦNG 1: ĐIỀU KHIỂN BAY & CẢM BIẾN (Mô phỏng Phase 1 / Phần cứng Phase 2)"]
        FC["Mạch Điều khiển Bay ArduPilot<br/>(SITL / SpeedyBee F405)"]
        LidarHW["Cảm biến 2D Planar LiDAR 360°<br/>(Gazebo gpu_lidar / LDRobot LD19)"]
        Sensors["GPS M10 + La bàn + ToF VL53L1X<br/>+ 4x ESC 30A + Động cơ 2212"]
        RC["Tay cầm Điều khiển RC An toàn<br/>(FlySky FS-i6 Chiếm quyền Khẩn cấp)"]

        FC <--> Sensors
        RC -.->|"2.4GHz iBUS Ưu tiên Chiếm quyền"| FC
    end

    UI <==>|"WebSocket JSON (Telemetry & Lệnh 20Hz)"| Gateway
    LidarHW -->|"ROS2 Topic / UART 230400"| LidarPort
    MavBridge <==>|"MAVLink v2 (UDP :14550 | UART /dev/ttyAMA0)"| FC
```

### 2.2. Chu trình Điều khiển Thời gian thực & Luồng Dữ liệu (Sequence Diagram)

```mermaid
sequenceDiagram
    autonumber
    participant Operator as Ứng dụng Điều khiển (TypeScript)
    participant Gateway as Companion Gateway (FastAPI)
    participant FSM as Máy trạng thái & Bộ Né vật cản
    participant Lidar as Bộ chuyển đổi 2D LiDAR (ILidarPort)
    participant FC as Flight Controller (ArduPilot)

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
    ILidarPort <|.. SerialLD19LidarAdapter : Phase 2 (Phần cứng)
    IFlightBridgePort <|.. MavlinkFlightBridge : UDP / UART MAVLink v2
```

---

## 3. Cấu trúc Thư mục (Repository Structure)

```text
uav-agri-autonomy/
├── README.md                               # Tài liệu chính (Tiếng Việt)
├── README-us.md                            # Tài liệu tiếng Anh (English)
├── docker-compose.sim.yml                  # Tổ hợp container SITL + Companion cho Phase 1
├── configs/
│   ├── simulation.yaml                     # Cấu hình Phase 1 (UDP :14550 + Gazebo LiDAR)
│   └── avoidance_params.yaml               # Tham số thuật toán APF, VFH+ và bù góc nghiêng
├── docs/
│   ├── requirements/
│   │   └── hardware.md                     # Tech Stack, dự toán BOM Phase 2 (<= 6M VNĐ), quy tắc đấu nối
│   └── dev/
│       └── phases/
│           ├── phase_1.md                  # Phân rã công việc Phase 1 (Mô phỏng & Kiến trúc - Đang triển khai)
│           └── phase_2.md                  # Phân rã công việc Phase 2 (Hướng phát triển Tích hợp Phần cứng)
├── drone-core/                             # Gói phần mềm tự hành trên Companion Computer (Python)
│   ├── pyproject.toml
│   ├── src/
│   │   ├── domain/                         # Nghiệp vụ cốt lõi thuần túy (Độc lập phần cứng)
│   │   │   ├── models.py                   # LaserScan360, DroneState, VelocityCommand
│   │   │   ├── tilt_compensator.py         # Chiếu góc Roll/Pitch & loại bỏ tia chạm đất
│   │   │   ├── apf_avoidance.py            # Mode 1: Bộ điều khiển Trường thế Nhân tạo (APF)
│   │   │   ├── vfh_planner.py              # Mode 2: Bộ hoạch định VFH+ 72 góc quạt
│   │   │   └── state_machine.py            # Chuyển đổi chế độ bay & tổng hợp lệnh an toàn
│   │   ├── ports/                          # Giao diện trừu tượng (Ports cho Phase 1 & Phase 2)
│   │   │   ├── lidar_port.py
│   │   │   └── flight_bridge_port.py
│   │   ├── adapters/                       # Các Adapter hiện hành cho Phase 1 (Simulation)
│   │   │   ├── mavlink_bridge.py
│   │   │   └── sim_lidar_adapter.py
│   │   ├── api/                            # Máy chủ FastAPI WebSocket & REST Gateway
│   │   │   └── server.py
│   │   └── logger/                         # Bộ ghi nhật ký hộp đen chuyến bay (.csv)
│   │       └── flight_logger.py
│   └── tests/                              # Kiểm thử đơn vị cho tầng Domain
│       └── test_avoidance_domain.py
├── crop-ai-service/                        # Mô-đun mở rộng: Nhận diện bệnh cây trồng (ONNX)
│   └── inference_onnx.py
├── ground-control-app/                     # Ứng dụng Trạm điều khiển Linux Desktop & Web (TypeScript)
│   ├── package.json
│   └── src/
│       ├── components/
│       │   └── LidarRadarCanvas.tsx        # Vẽ bản đồ Radar 360° & vector vận tốc trực quan
│       ├── hooks/
│       │   └── useDroneSocket.ts           # Hook kết nối WebSocket thời gian thực 20Hz
│       └── types/
│           └── protocol.ts                 # Định nghĩa cấu trúc gói tin đồng bộ với Backend
└── simulation-worlds/
    └── orchard_obstacles.sdf               # Môi trường vườn cây nhẹ trong Gazebo phục vụ kiểm thử
```

---

## 4. Hướng dẫn Khởi chạy & Định hướng Chuyển đổi Phase 2

### 4.1. Chạy Kiểm thử Đơn vị (Domain Unit Tests — Phase 1)
Các thuật toán né tránh vật cản cốt lõi (`TiltCompensator`, `APFAvoidanceController`, `VFHPlusPlanner`) không phụ thuộc vào thư viện phần cứng bên ngoài và có thể kiểm chứng trực tiếp bằng Python 3:

```bash
cd drone-core
PYTHONPATH=. python3 -c "import tests.test_avoidance_domain as t; t.test_tilt_compensation_rejects_ground_hit(); t.test_apf_repulses_forward_obstacle(); t.test_vfh_bypasses_front_obstacle(); print('ALL DOMAIN TESTS PASSED')"
```

### 4.2. Chạy Mô phỏng (Phase 1) và Hướng Mở rộng sang Phần cứng Thực tế (Phase 2)
- **Trọng tâm hiện tại — Chế độ Mô phỏng (Phase 1):** Hệ thống khởi chạy với tệp cấu hình `configs/simulation.yaml` (kết nối `udpin:0.0.0.0:14550` tới ArduPilot SITL và bộ chuyển đổi `SimGazeboLidarAdapter`).
- **Hướng phát triển — Chế độ Phần cứng Thực tế (Phase 2):** Nhờ giao diện trừu tượng `ILidarPort` và `IFlightBridgePort` đã định nghĩa sẵn, khi bước sang Học kỳ 2 chỉ cần bổ sung adapter phần cứng (`SerialLD19LidarAdapter` đọc cổng `/dev/ttyUSB0`) và đổi URI kết nối MAVLink sang cổng UART (`/dev/ttyAMA0` ở tốc độ `921600` baud) trên Raspberry Pi 4B mà không cần sửa đổi tầng `domain/`.
