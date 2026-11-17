# Docking robot bằng Camera 3D (ROS 2 Jazzy)

Phần mềm giúp robot (AMR / xe nâng) **tự tiến sát vào một vật** như pallet, rack hay trạm sạc, chỉ dựa vào **đám mây điểm (PointCloud2) từ bất kỳ camera 3D nào**. Nó được tối ưu để chạy trên máy tính yếu: chỉ dùng numpy/scipy, không cần GPU, không cần huấn luyện mạng nơ-ron.

```
Camera 3D ─PointCloud2─▶ pose_estimator ─detected_dock_pose─▶ Nav2 docking_server ─▶ cmd_vel
                         (template + ICP + Kalman)          └▶ docking_controller ─▶ cmd_vel
```

## Kết quả kiểm chứng

| Hạng mục | Kết quả |
|---|---|
| Unit + integration test | **102 test pass** (`camera3d_docking`), 2 test (`camera3d_docking_sim`) |
| Benchmark tổng hợp: pallet 1.0–2.5 m, ±30°, nhiễu stereo | **40/40** thành công.<br>Sai số vị trí: trung vị **1.1 mm**, P95 **3.7 mm**.<br>Sai số yaw: trung vị **0.04°**, P95 **0.15°** |
| Thời gian (i7-14700F) | Tìm kiếm toàn cục ~100 ms/khung, **bám ~12 ms/khung** |
| Docking E2E trong Gazebo (ground truth) | **4/4 DOCKED**. Ngang ≤ 0.3 cm, yaw ≤ 1.8°. Với quãng tiếp cận ≥ 1.2 m: yaw ≤ 0.93° |

> Số liệu trên đo bằng dữ liệu mô phỏng. Trên robot thật cần đo lại theo [hướng dẫn triển khai](docs/HUONG_DAN_TRIEN_KHAI.md), bước B6 và B8.

## Cấu trúc thư mục

```
Docking using Camera 3D/
├── README.md                       ← file này
├── env.sh                          ← source trước khi làm việc (ROS + workspace + PYTHONNOUSERSITE=1)
├── docs/
│   ├── HUONG_DAN_TRIEN_KHAI.md      ← checklist đưa lên robot thật (B1–B10) + xử lý sự cố
│   └── superpowers/specs|plans/    ← thiết kế chi tiết + kế hoạch triển khai
├── tools/
│   ├── install_ros2_jazzy.sh        ← cài ROS 2 Jazzy (+ --robot, --camera realsense|orbbec|oakd)
│   └── pin_apps_to_dock.sh          ← ghim RViz2/rqt/Gazebo/PlotJuggler/CloudCompare/Docking Sim
├── references/                     ← 6 repo tham khảo (chỉ đọc)
└── ros2_ws/src/
    ├── camera3d_docking/            ← gói chính
    │   ├── camera3d_docking/core/   ← thuật toán thuần numpy (không ROS, test được độc lập)
    │   ├── camera3d_docking/ros/    ← node ROS 2 (estimator, controller, cloud_saver)
    │   ├── camera3d_docking/tools/  ← make_template, benchmark
    │   ├── config/ launch/ templates/ rviz/ test/
    └── camera3d_docking_sim/        ← mô phỏng Gazebo + test E2E
```

## Bắt đầu nhanh

```bash
cd ~/Desktop/"Docking using Camera 3D" && source env.sh
```

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
```

```bash
cd src/camera3d_docking && python3 -m pytest
```

**Xem mô phỏng:** bấm nút **Docking Sim** trên taskbar, hoặc chạy lệnh sau rồi gọi `start`:

```bash
ros2 launch camera3d_docking_sim sim.launch.py
```

```bash
ros2 service call /docking_controller/start std_srvs/srv/Trigger
```

**Chạy test E2E docking trong Gazebo (headless):**

```bash
bash ros2_ws/src/camera3d_docking_sim/scripts/run_e2e.sh
```

**Trên robot thật:** làm theo [docs/HUONG_DAN_TRIEN_KHAI.md](docs/HUONG_DAN_TRIEN_KHAI.md). Hai lệnh khởi chạy chính:

- Không có Nav2: `ros2 launch camera3d_docking docking.launch.py points_topic:=<topic cloud>`.
- Có Nav2: `ros2 launch camera3d_docking nav2_docking.launch.py points_topic:=<topic cloud>`.

## Thuật toán (tóm tắt)

1. **Cân bằng sàn bằng RANSAC.** Tự sửa sai số pitch/roll/độ cao khi lắp camera.
2. **Cắt ROI và voxel.** Xóa sàn và giảm số điểm.
3. **Tìm kiếm toàn cục 2.5D đa dải độ cao.** Tương quan bằng FFT, có phạt "vùng phải trống" (ví dụ hốc pallet) để tường phẳng không giả làm được pallet.
4. **ICP point-to-plane, 4 bậc tự do (x, y, z, yaw).** Kernel Tukey, thô → tinh. Tương ứng tra bằng **lưới O(1)** cho CPU yếu.
5. **Cổng chất lượng:** coverage, inlier, RMSE, σ, **khả năng quan sát** (suy biến) và **mơ hồ** (hai vật giống nhau). Thà từ chối còn hơn xuất pose sai.
6. **Kalman giữ pose vật đứng yên trong `odom`.** Robot vẫn vào đúng chỗ khi camera quá gần, không còn thấy vật (vùng mù).
7. **Điều khiển:** luật Park–Kuipers (giống Nav2 graceful controller) cùng máy trạng thái có vùng mù và lùi lại thử lại.

Chi tiết và lý do từng lựa chọn (gồm bài học từ 2 bài báo và 6 repo) nằm trong [spec thiết kế](docs/superpowers/specs/2026-11-16-camera3d-docking-design.md).

## Lưu ý quan trọng

- **numpy 2.x trong `~/.local`** xung đột với ROS 2 Jazzy và scipy của apt. Các node tự xử lý, nhưng khi tự chạy Python/rqt hãy `source env.sh` trước.
- **Mặt dock phẳng hoàn toàn** sẽ bị báo `DEGENERATE`, vì không đo được độ lệch ngang. Hãy dùng **dock rãnh chữ V** (`make_template vdock`).
- **YOLO chưa tích hợp.** Camera có thể chỉ cho point cloud, và YOLO là khâu nặng nhất. Nếu cần tìm vật trong cảnh rất lộn xộn, điểm móc nối là `vision_msgs/Detection2DArray` (từ `yolo_ros`) để thu hẹp ROI. Xem spec mục 4.
