# Hướng dẫn triển khai docking bằng camera 3D lên robot thật

Tài liệu này là **Phần B** của [kế hoạch](superpowers/plans/2026-11-16-camera3d-docking.md), viết ở dạng checklist.

- Làm **lần lượt từ B1 đến B10**.
- Mỗi bước có **tiêu chí nghiệm thu**. Chưa đạt thì không chuyển sang bước sau.
- Thiết kế và lý do kỹ thuật nằm trong [spec](superpowers/specs/2026-11-16-camera3d-docking-design.md).

```
B1 phần cứng → B2 cài đặt → B3 driver camera → B4 hiệu chuẩn → B5 template
→ B6 đo độ chính xác tĩnh → B7 tinh chỉnh điều khiển → B8 nghiệm thu 20 lần
→ B9 tích hợp hệ thống → B10 tối ưu hiệu năng
```

---

## B1. Chọn và gắn phần cứng

**Camera.** Nên chọn loại **tính depth ngay trên camera**, để máy tính robot không phải gánh phần này.

| Camera | Depth tính ở đâu | Tầm tốt | Ghi chú |
|--------|------------------|---------|---------|
| Intel RealSense D435i / D455 | Trên camera (ASIC) | 0.3–3 m (D455 xa hơn) | Phổ biến, driver apt `ros-jazzy-realsense2-camera` |
| Orbbec Gemini 2 / 335 | Trên camera | 0.2–3 m | Driver apt `ros-jazzy-orbbec-camera` |
| Luxonis OAK-D Pro | Trên camera (có NPU) | 0.3–3 m | Driver apt `ros-jazzy-depthai-ros`; NPU chạy được YOLO nếu cần |
| Stereolabs ZED 2i / X | **GPU của máy chủ** | 0.3–15 m | Bắt buộc NVIDIA (Jetson) — không hợp máy yếu |

**Cách gắn camera:**

- Cao 0.25–0.5 m, chúc xuống 5–15° (phải thấy được mép dưới của vật).
- Gắn **cứng, không rung**.
- Cáp USB3 ngắn, có khóa.
- Tránh nắng chiếu thẳng vào camera stereo IR.

**Máy tính:**

- Raspberry Pi 5 (8 GB): đủ cho 5–10 Hz.
- Jetson Orin Nano / mini PC Intel N100: thoải mái.

**Nghiệm thu:** ở mọi khoảng cách từ 0.6 đến 2.5 m, camera thấy trọn mặt trước của vật.

## B2. Cài phần mềm trên robot

1. Cài Ubuntu **24.04** (bản Server là đủ).
2. Chép thư mục dự án lên robot.
3. Cài đặt và build:

```bash
bash tools/install_ros2_jazzy.sh --robot --camera realsense
```

```bash
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
```

```bash
source env.sh
```

4. Kiểm tra bằng bộ test, rồi đo tốc độ **ngay trên robot**:

```bash
cd ros2_ws/src/camera3d_docking && python3 -m pytest
```

```bash
ros2 run camera3d_docking benchmark --quick
```

**Nghiệm thu:**

- Toàn bộ test pass.
- Benchmark báo thời gian chế độ bám (tracking) ≤ 50 ms/khung. Nếu vượt, xem B10.

> Lưu ý: luôn `source env.sh` trước khi làm việc. File này đặt `PYTHONNOUSERSITE=1` để tránh numpy 2.x trong `~/.local` xung đột với ROS 2 Jazzy. Các node của dự án tự bảo vệ trước xung đột này, nhưng các công cụ khác như `rqt` thì không.

## B3. Cấu hình driver camera

Mục tiêu: phát `PointCloud2` **organized** ở độ phân giải thấp và 15 FPS, đồng thời bật bộ lọc chạy trên camera.

Tên tham số khác nhau giữa các phiên bản driver. Kiểm tra trước bằng `ros2 launch <gói> <file> --show-args`.

```bash
# RealSense (realsense2_camera 4.5x)
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=424x240x15 \
  pointcloud.enable:=true pointcloud.ordered_pc:=true \
  spatial_filter.enable:=true temporal_filter.enable:=true
```

- Xem tên topic cloud bằng `ros2 topic list | grep points`. Thường là `/camera/camera/depth/color/points`.
- Đo tần số bằng `ros2 topic hz <topic>`.

**Nghiệm thu:** tần số ổn định khoảng 15 Hz, CPU dành cho driver dưới 25%.

## B4. Hiệu chuẩn vị trí camera (ngoại tham số)

1. Đo vị trí và góc của camera so với `base_link`, rồi phát TF tĩnh tới **frame gốc của driver**. Với RealSense, frame gốc thường là `camera_link`:

```bash
ros2 run tf2_ros static_transform_publisher --x 0.25 --y 0 --z 0.30 --roll 0 --pitch 0.1745 --yaw 0 --frame-id base_link --child-frame-id camera_link
```

2. Chạy estimator, rồi xem trường `floor` trên topic `/pose_estimator/status`:

```bash
ros2 launch camera3d_docking estimator.launch.py points_topic:=<topic cloud>
```

   - `tilt_deg` > 1° nghĩa là pitch/roll khai báo đang sai. Sửa lại TF tĩnh.
   - `floor_leveling` sẽ tự bù sai số nhỏ (≤ 10°), nhưng nên đưa sai số về < 1° ngay từ đầu.

3. Đặt một hộp vuông vức ở vị trí đo bằng thước, ví dụ x = 1.00 m. So với điểm cloud trong RViz (fixed frame `base_link`).

**Nghiệm thu:** vị trí hộp lệch < 1 cm ở khoảng cách 1 m; `tilt_deg` < 1°.

## B5. Tạo template cho từng loại vật

Frame vật có gốc tại **tâm mặt trước, trên sàn**. Trục +x hướng vào trong vật, +z hướng lên.

**Cách A — mô hình tham số:** đo kích thước thật của vật rồi tạo template.

```bash
ros2 run camera3d_docking make_template pallet --depth 1.2 --width 0.8 --block-widths 0.10 0.145 0.10 -o ~/templates/pallet_kho_A.npz
```

- Với kệ dùng `make_template rack --bay-width ... --beam-levels ...`.
- Với trạm sạc dùng `make_template vdock ...`. **Nên làm dock rãnh chữ V**: mặt phẳng trơn sẽ bị từ chối (DEGENERATE).

**Cách B — quét thực tế:** chính xác nhất, vì template mang luôn đặc tính của camera.

1. Đặt robot nhìn thẳng vào vật, cách 1.0–1.5 m. Đo pose của mặt trước vật trong `base_link` bằng thước hoặc AprilTag (xem B6).
2. Lưu 5 cloud:

```bash
ros2 run camera3d_docking cloud_saver --ros-args -r points:=<topic> -p count:=5 -p output_dir:=scan1
```

3. Lặp lại ở 2–4 vị trí khác (lệch ±20°). Gộp tất cả, với `--object-pose` là pose mặt trước vật đo được cho mỗi lần quét:

```bash
ros2 run camera3d_docking make_template from-cloud scan1/*.pcd --object-pose 1.20 0.02 1.5 --camera-pos 0.25 0 0.30 -o vat.npz
```

4. Kiểm tra: xuất sang PCD rồi mở trong CloudCompare (đã ghim trên taskbar).

```bash
ros2 run camera3d_docking make_template export-pcd vat.npz vat.pcd
```

**Nghiệm thu:** trong RViz, đám điểm `/pose_estimator/aligned_template` (màu xanh) khớp lên cloud thật, ở nhiều góc nhìn khác nhau.

## B6. Đo độ chính xác tĩnh (ground truth bằng AprilTag)

1. In tag `tag36h11` cạnh 0.16 m, dán **đúng tâm mặt trước** vật.
2. Chạy `apriltag_ros` (đã cài sẵn) trên ảnh màu cùng `camera_info`. Pose tag cộng với offset đã biết cho ra **pose thật** của vật.
3. Ghi `/pose_estimator/pose_raw` và pose tag bằng `ros2 bag record`, trên lưới vị trí:
   - Khoảng cách {0.8, 1.2, 1.8, 2.5} m.
   - Góc {−30, −15, 0, 15, 30}°.
   - 100 khung mỗi điểm.
4. Tính sai số vị trí và yaw.

**Nghiệm thu:**

- Trung vị ≤ 1.5 cm / 1°; P95 ≤ 3 cm / 2°.
- **Không có** kết quả `OK` nào sai quá 10 cm.

## B7. Tinh chỉnh điều khiển

1. **Thử trong mô phỏng trước.** Bấm nút **Docking Sim** trên taskbar, rồi:

```bash
ros2 service call /docking_controller/start std_srvs/srv/Trigger
```

2. Trên robot thật, bắt đầu với `law.v_max: 0.1` trong `config/controller.yaml`.
3. Các tham số chính:

| Tham số | Ý nghĩa | Gợi ý |
|---------|---------|-------|
| `dock_offset_x` | Vị trí dừng của `base_link` so với mặt vật | −(khoảng từ base_link tới mũi xe + khe hở) |
| `backward` | Lùi vào (xe nâng càng sau, camera sau) | `true` nếu camera gắn phía sau |
| `fsm.blind_zone` | Từ đây đi theo pose giữ (camera có thể không thấy vật) | ≥ tầm gần nhất của camera + 0.2 m |
| `fsm.goal_tol_x` | Sai số dừng dọc trục | 0.01 m |
| `law.k_phi`, `law.k_delta` | Độ "gắt" của quỹ đạo | Giảm nếu robot lắc; tăng nếu vào chậm khớp hướng |
| `fsm.acc_v`, `fsm.acc_w` | Giới hạn gia tốc | Theo khả năng đế robot |

4. Dùng PlotJuggler (đã ghim trên taskbar) để vẽ `/cmd_vel` và các trường trong `/pose_estimator/status`.
5. **Bắt đầu docking từ đủ xa: ≥ 1.2 m trước điểm dừng.** Đo trong Gazebo với xuất phát lệch −0.3 m / −20°:

   | Quãng tiếp cận tới điểm dừng | Sai số yaw cuối | Sai số ngang cuối |
   |---|---|---|
   | 0.95 m | 1.77° | 0.3 cm |
   | 1.25 m | 0.93° | 0.0 cm |

   - Với Nav2, `staging_x_offset: -1.6` đã đặt sẵn: staging cách mặt vật 1.6 m, tức 1.25 m trước điểm dừng.

**Nghiệm thu:** robot không dao động, và không chạm vật trước điểm dừng.

## B8. Nghiệm thu docking

1. Thực hiện 20 lần liên tiếp. Xuất phát ngẫu nhiên quanh staging pose (±0.2 m, ±15°).
2. Ghi lại cho mỗi lần: trạng thái cuối, sai số cuối (đo bằng thước hoặc AprilTag), thời gian.

**Nghiệm thu:** ≥ 19/20 thành công; sai số cuối ≤ 2 cm / 1.5°.

## B9. Tích hợp hệ thống

**Robot có Nav2.** Sửa danh sách `docks` (pose gần đúng trên map) trong `config/nav2_docking.yaml`, rồi:

```bash
ros2 launch camera3d_docking nav2_docking.launch.py points_topic:=<topic>
```

```bash
ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot "{use_dock_id: true, dock_id: pallet_1}"
```

**Robot không có Nav2.**

- Khởi chạy: `ros2 launch camera3d_docking docking.launch.py points_topic:=<topic>`.
- Hệ RCS/FMS gọi `/docking_controller/start` và `/docking_controller/cancel`, rồi đọc `/docking_controller/state`.

**Tiết kiệm CPU.** Đặt `start_enabled: false`, rồi chỉ bật estimator khi cần docking:

```bash
ros2 service call /pose_estimator/enable std_srvs/srv/SetBool "{data: true}"
```

**An toàn** (bắt buộc ngoài phần mềm này):

- Nút dừng khẩn.
- Laser an toàn hoặc collision monitor.
- Giới hạn tốc độ.
- Nên có cảm biến tiếp xúc, phát lên topic `/docking_controller/contact` (`std_msgs/Bool`).

## B10. Tối ưu hiệu năng (khi máy yếu)

Chạy `ros2 run camera3d_docking benchmark` trên robot, rồi chỉnh theo thứ tự:

1. `cloud_stride: 2 → 3`, `voxel_size: 0.02 → 0.025`.
2. Giảm độ phân giải trên driver camera.
3. `max_rate_hz: 10 → 5`. Docking chậm vẫn đủ chính xác.
4. `search.yaw_step_deg: 3 → 4`, `search.yaw_range_deg: 45 → 30`. Chỉ ảnh hưởng lúc tìm kiếm.
5. Vẫn chưa đủ thì port `core/registration.py` sang C++ (pybind11), hoặc dùng `small_gicp`.

**Nghiệm thu:** ≥ 10 Hz ở chế độ bám, CPU < 50% một nhân.

## B11. (Tùy chọn) Mặt nạ segmentation cho cảnh có vật lạ sát vật

Mặc định hệ thống chỉ dùng hình học của đám mây điểm, và ở cảnh sạch như vậy là đủ (~1 mm). Nhưng khi có vật lạ **sát** vật cần dock (tường ngay phía sau, hàng chồng trên pallet, pallet nằm giữa dãy hàng), bản chỉ-hình-học có thể từ chối liên tục, hoặc tệ hơn là **báo OK với pose sai**. Số liệu nằm trong README, mục Segmentation. Mặt nạ segmentation cho biết điểm nào thuộc vật, để loại vật lạ trước khi tìm kiếm và ICP.

**Cần có:**

- Camera có ảnh màu, và cloud **organized** căn theo ảnh màu. Với RealSense: thêm `align_depth.enable:=true` vào lệnh ở B3 (đã có `pointcloud.ordered_pc:=true`).
- Một node segmentation phát `sensor_msgs/Image` làm mặt nạ:
  - pixel khác 0 = vật cần dock;
  - cùng kích thước hoặc cùng tỉ lệ khung với cloud, stamp = stamp của ảnh màu;
  - encoding: mono8/8UC1, mono16/16UC1, 32SC1, 32FC1, rgb8/bgr8/8UC3, rgba8/bgra8.
- Ví dụ: YOLO-seg qua `yolo_ros`, cộng một node nhỏ vẽ mặt nạ của lớp `pallet` thành ảnh mono8. Mặt nạ phải là **từng pixel**: chỉ có bbox thì vẫn nhận sai khi có hàng hóa sát vật.
- Mạng là khâu nặng nhất (~50–150 ms/khung trên CPU cỡ RPi). Nên chạy trên NPU (OAK-D) hoặc GPU (Jetson).

**Bật** trong file tham số estimator (bản sao của `config/estimator.yaml`, truyền vào bằng `estimator_params:=...`):

```yaml
    mask_topic: /pallet_mask   # topic mặt nạ
    mask_mode: hard            # giữ hard: mặt nạ dùng cho cả tìm kiếm và ICP
    mask_max_age: 0.25         # lệch stamp tối đa giữa mặt nạ và cloud (s)
```

**Kiểm tra:** chạy `ros2 topic echo /pose_estimator/status` và xem trường `"mask"`:

| Giá trị | Ý nghĩa |
|---|---|
| `used` | Đã dùng mặt nạ |
| `too_few` | Mặt nạ có quá ít điểm trong ROI → tự quay về chỉ dùng hình học |
| `no_mask` | Không có mặt nạ đủ mới (lệch stamp > `mask_max_age`) → chỉ dùng hình học |
| `size_mismatch` | Mặt nạ khác tỉ lệ khung với cloud (ảnh màu chưa căn với depth) |
| `unorganized` | Cloud không organized → không ghép được pixel |
| `off` | Chưa bật (`mask_topic` rỗng) |

Nếu mạng segmentation hỏng hoặc chậm, docking vẫn chạy bằng hình học, không phụ thuộc hoàn toàn vào mạng.

**Không có ảnh màu, hoặc máy quá yếu:** nếu vật lạ chỉ nằm phía trên hoặc phía sau vật, hạ `roi.max` z xuống ngay trên vật (pallet: 0.20). Trong benchmark, cách này sửa được cảnh tường và cảnh hàng chồng. **Không** dùng khi có hàng sát hai bên vật: benchmark cho thấy khi đó nhận sai nhiều hơn. Luôn nghiệm thu lại theo B8.

**Nghiệm thu:** `ros2 run camera3d_docking benchmark --seg-study --quick` chạy được trên robot; ở chỗ docking thật, phần lớn khung có `"mask": "used"` và B8 đạt.

---

## Xử lý sự cố

Các trạng thái dưới đây đọc từ topic `/pose_estimator/status`.

| Trạng thái / hiện tượng | Nguyên nhân thường gặp | Cách xử lý |
|---|---|---|
| `NO_POINTS` | Sai topic, thiếu TF `base_link→camera`, ROI không chứa vật | Kiểm tra `ros2 topic hz`, `ros2 run tf2_tools view_frames`, `roi.*` |
| `NOT_FOUND` | Vật ngoài ROI, template sai loại | Nới ROI, kiểm tra template bằng CloudCompare |
| `LOW_QUALITY` (coverage thấp) | Template khác vật thật, camera nhiễu, sai ngoại tham số | Quét template thật (B5 cách B), hiệu chuẩn lại (B4), `min_points_per_voxel: 2` |
| `DEGENERATE` | Mặt vật phẳng, thiếu đặc trưng ngang | Làm dock chữ V, thêm vật mốc; hoặc chấp nhận rủi ro bằng cách hạ `icp.min_observability` |
| `AMBIGUOUS` | Hai vật giống nhau trong ROI (pallet xếp cạnh nhau) | Thu hẹp `roi`, bật `use_expected_pose` |
| `BLIND_HOLD` | Bình thường khi robot đã rất gần vật | — |
| Báo `OK` nhưng robot vào lệch, khi có tường/hàng hóa sát vật | Điểm của vật lạ bị khớp nhầm (điểm yếu của bản chỉ-hình-học) | Bật mặt nạ segmentation, hoặc hạ trần ROI (B11) |
| `"mask": "no_mask"` hoặc `"size_mismatch"` | Mặt nạ đến trễ, hoặc ảnh màu chưa căn với depth | Tăng `mask_max_age`, bật `align_depth` (B11) |
| Cảnh báo "Chưa tra được TF" | Lệch đồng hồ, chưa có odom/TF | Kiểm tra `use_sim_time`, nguồn TF `odom→base_link` |
| Lỗi numpy / scipy `dtype size changed` | numpy 2.x trong `~/.local` | `source env.sh` (đặt `PYTHONNOUSERSITE=1`) |
| Gazebo "Unable to find file" | Đường dẫn có dấu cách | Đã xử lý trong `sim.launch.py` (truyền tên world + `GZ_SIM_RESOURCE_PATH`) |
