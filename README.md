# Docking robot bằng Camera 3D (ROS 2 Jazzy)

Phần mềm giúp robot (AMR / xe nâng) **tự tiến sát vào một vật** như pallet, rack hay trạm sạc, chỉ dựa vào **đám mây điểm (PointCloud2) từ bất kỳ camera 3D nào**. Nó được tối ưu để chạy trên máy tính yếu: chỉ dùng numpy/scipy, không cần GPU, không cần huấn luyện mạng nơ-ron.

```
Camera 3D ─PointCloud2─▶ pose_estimator ─detected_dock_pose─▶ Nav2 docking_server ─▶ cmd_vel
                         (template + ICP + Kalman)          └▶ docking_controller ─▶ cmd_vel
```

## Kết quả kiểm chứng

| Hạng mục | Kết quả |
|---|---|
| Unit + integration test | **117 test pass** (`camera3d_docking`), 2 test (`camera3d_docking_sim`) |
| Benchmark tổng hợp: pallet 1.0–2.5 m, ±30°, nhiễu stereo | **40/40** thành công.<br>Sai số vị trí: trung vị **1.1 mm**, P95 **3.7 mm**.<br>Sai số yaw: trung vị **0.04°**, P95 **0.15°** |
| Thời gian (i7-14700F) | Tìm kiếm toàn cục ~100 ms/khung, **bám ~12 ms/khung** |
| Docking E2E trong Gazebo (ground truth) | **4/4 DOCKED**. Ngang ≤ 0.3 cm, yaw ≤ 1.8°. Với quãng tiếp cận ≥ 1.2 m: yaw ≤ 0.93° |
| Cảnh có vật lạ sát pallet: tường ngay sau, hàng chồng trên, giữa dãy hàng (`benchmark --seg-study`) | Chỉ hình học: **nhận sai** 8, 8 và 1 lần / 40. Có mặt nạ segmentation từng pixel: 40/40, 0 sai, P95 ≤ 6 mm. Xem mục Segmentation bên dưới |

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

## Segmentation có giúp chính xác hơn không?

Đo bằng `ros2 run camera3d_docking benchmark --seg-study`: 4 cảnh × 40 tư thế (pallet cách 1.0–2.5 m, lệch ±30°), nhiễu stereo. Mặt nạ "hoàn hảo" lấy từ ground truth của cảnh tổng hợp, tức là **trần trên của mọi mạng segmentation**. Các mặt nạ lỗi mô phỏng mạng thật.

| Cảnh | Không mặt nạ | Mặt nạ hoàn hảo | Mặt nạ lỗi 3 px (lem / hụt / lệch) | Chỉ có bbox |
|---|---|---|---|---|
| Sạch (chỉ pallet) | 40/40, 0 sai | 40/40, 0 sai | 39–40/40, 0 sai | 40/40, 0 sai |
| Tường sát sau pallet (5 cm) | 22/40, **8 sai** | 40/40, 0 sai | 39–40/40, 0 sai | 40/40, 0 sai |
| Pallet có hàng chồng, nhô ra 2 cm | 8/40, **8 sai** | 40/40, 0 sai | 34–40/40, 0–1 sai | 30/40, **2 sai** |
| Giữa dãy hàng (khe 5 cm) + tường | 1/40, **1 sai** | 40/40, 0 sai | 20–40/40, 0–1 sai | 8/40, **5 sai** |

`x/40` là số lần báo OK. "Sai" là báo OK nhưng lệch hơn 2 cm hoặc 2°, tức là nguy hiểm. Với mặt nạ hoàn hảo, sai số vị trí trung vị khoảng 1 mm và P95 ≤ 6 mm ở mọi cảnh.

- **Cảnh sạch: không.** Chỉ hình học đã đạt khoảng 1 mm.
- **Cảnh có vật lạ sát pallet: có, rất lớn.** Lý do là bản chỉ-hình-học hiện **chưa chịu được** vật lạ sát vật: điểm của tường hoặc hàng hóa bị khớp nhầm, có lúc vẫn báo OK với pose sai.
- **Mặt nạ phải khớp từng pixel và dùng cho cả tìm kiếm lẫn ICP** (`mask_mode: hard`). Chỉ dùng mặt nạ cho bước tìm kiếm thì không giúp được (cảnh tường: 25/40, 12 sai). Chỉ có bbox của detector thì vẫn nhận sai.
- **Cách rẻ hơn, không cần mạng nơ-ron:** hạ trần ROI xuống ngay trên vật (`roi.max` z = 0.20 với pallet). Cách này sửa được cảnh tường và cảnh hàng chồng (40/40, 0 sai), nhưng ở cảnh giữa dãy hàng lại nhận sai 20/40 lần, nên chưa an toàn để làm mặc định.
- **Cái giá của segmentation:** cần camera có ảnh màu căn với depth, và một mạng (YOLO-seg, SegFormer...). Đây thường là khâu nặng nhất trên máy yếu. Cách bật: [hướng dẫn triển khai, mục B11](docs/HUONG_DAN_TRIEN_KHAI.md).

## Lưu ý quan trọng

- **numpy 2.x trong `~/.local`** xung đột với ROS 2 Jazzy và scipy của apt. Các node tự xử lý, nhưng khi tự chạy Python/rqt hãy `source env.sh` trước.
- **Mặt dock phẳng hoàn toàn** sẽ bị báo `DEGENERATE`, vì không đo được độ lệch ngang. Hãy dùng **dock rãnh chữ V** (`make_template vdock`).
- **Điểm yếu hiện tại: vật lạ sát vật cần dock** (tường ngay sau, hàng chồng trên pallet, pallet giữa dãy hàng). Bản chỉ-hình-học có thể báo OK với pose sai (xem mục Segmentation). Ở những chỗ như vậy, hãy bật mặt nạ segmentation (B11), hoặc hạ trần ROI nếu vật lạ chỉ nằm phía trên/phía sau vật, rồi nghiệm thu kỹ theo B8.
