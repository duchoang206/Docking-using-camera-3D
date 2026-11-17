# Docking bằng Camera 3D — Kế hoạch triển khai

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Gói ROS 2 Jazzy nhận `PointCloud2` từ bất kỳ camera 3D nào, ước lượng chính xác pose của vật cần dock (pallet, rack, trạm sạc… dựa trên template), xuất `detected_dock_pose` cho Nav2 hoặc cho bộ điều khiển docking riêng. Kèm mô phỏng Gazebo để kiểm thử end-to-end. Chạy được trên CPU yếu.

**Architecture:** Lõi thuật toán thuần numpy/scipy (`core/`, không import ROS) gồm các khâu:

1. Cân bằng sàn bằng RANSAC.
2. Cắt ROI và voxel.
3. Tìm kiếm tương quan 2.5D đa dải có phạt vùng-phải-trống.
4. ICP point-to-plane có thành phần tiếp tuyến, 4 bậc tự do, tra cứu tương ứng bằng lưới O(1).
5. Cổng chất lượng: coverage, suy biến, mơ hồ.
6. Kalman giữ pose vật trong `odom`.

Các node ROS là lớp vỏ mỏng bọc quanh lõi. Bộ điều khiển dùng luật Park–Kuipers (giống Nav2 graceful controller) với máy trạng thái có vùng mù và thử lại.

**Tech Stack:** Ubuntu 24.04, ROS 2 Jazzy (rclpy, tf2_ros, sensor_msgs, geometry_msgs, std_srvs), Python 3.12, numpy 1.26 + scipy 1.11 (apt), pytest, Gazebo Harmonic (`ros_gz`), Nav2 `opennav_docking`.

**Spec:** [`docs/superpowers/specs/2026-11-16-camera3d-docking-design.md`](../specs/2026-11-16-camera3d-docking-design.md)

## Global Constraints

- **Nền tảng:** ROS 2 **Jazzy** trên Ubuntu **24.04**. Đơn vị SI (m, rad). Frame theo REP-103.
- **Phụ thuộc:**
  - `core/` chỉ import `numpy`, `scipy`, thư viện chuẩn. **Không** dùng Open3D, PCL, PyTorch, ultralytics, small_gicp.
  - Mọi node và test chạy với **`PYTHONNOUSERSITE=1`**. Entry point tự re-exec nếu thiếu biến này.
- **Quy ước frame vật:** gốc tại tâm mặt trước, trên sàn (z = 0); +x hướng vào trong vật; +z lên.
- **Đầu ra cho Nav2:** `detected_dock_pose` là `geometry_msgs/PoseStamped`, frame `odom`.
- **Từ chối thay vì xuất sai:** mọi kết quả phải qua cổng chất lượng. Ngưỡng mặc định:
  - coverage ≥ 0.35
  - inlier_ratio ≥ 0.5
  - rmse ≤ 0.015 m
  - σ_xy ≤ 0.02 m, σ_yaw ≤ 1.5°
  - tỷ số suy biến ≥ 1e-3
  - tỷ số mơ hồ < 0.9
- **Thông báo và log** viết bằng tiếng Việt. Tên định danh trong code dùng tiếng Anh.
- **Đường dẫn workspace có dấu cách** (`Docking using Camera 3D`). Phải kiểm chứng `colcon build` chạy được; nếu không, build qua symlink `~/docking_ws`.
- **Không commit git** trừ khi người dùng yêu cầu. Mỗi task kết thúc bằng mốc "chạy toàn bộ test".

## Review Focus

1. **Cloud rỗng, toàn NaN, hoặc organized mà mọi điểm không hợp lệ:** estimator trả `NO_POINTS`, không ném exception. Test nằm ở Task 7 và Task 10.
2. **TF chưa sẵn sàng tại stamp của cloud** (lúc khởi động hoặc stamp lệch): node bỏ khung, cảnh báo có giới hạn tần suất, không crash. Test nằm ở Task 11.
3. **Vật lọt vào vùng mù** (quá gần, ra ngoài FOV): pose vẫn được giữ và phát (hold), FSM không báo mất dấu. Test nằm ở Task 8, Task 9 và Task 11.
4. **Hai vật giống nhau trong ROI** (pallet xếp cạnh nhau): trả `AMBIGUOUS` thay vì chọn bừa. Thu hẹp ROI thì trả `OK`. Test nằm ở Task 7.
5. **Ngoại tham số camera lệch pitch 2°:** cân bằng sàn giữ sai số ≤ 2 cm. Test nằm ở Task 7.

---

## Cấu trúc file

```
Docking using Camera 3D/
├── README.md                      # Tổng quan + cách chạy nhanh            (Task 16)
├── env.sh                         # source ROS + workspace + PYTHONNOUSERSITE (Task 0)
├── docs/HUONG_DAN_TRIEN_KHAI.md   # Phần B: triển khai lên robot thật      (Task 16)
├── tools/install_ros2_jazzy.sh    # (đã có) cài ROS 2 Jazzy
├── tools/pin_apps_to_dock.sh      # tạo launcher + ghim taskbar             (Task 0b)
├── references/                    # 6 repo tham khảo (chỉ đọc)
└── ros2_ws/src/
    ├── camera3d_docking/
    │   ├── package.xml, setup.py, setup.cfg, pytest.ini, resource/camera3d_docking
    │   ├── camera3d_docking/
    │   │   ├── _env.py                  # re-exec với PYTHONNOUSERSITE=1      (Task 0)
    │   │   ├── core/geometry.py         # SE(3), quaternion, yaw               (Task 1)
    │   │   ├── core/cloud.py            # voxel, crop, normals, RANSAC sàn     (Task 2)
    │   │   ├── core/shapes.py           # mô hình hộp, ray-cast, camera ảo     (Task 3)
    │   │   ├── core/fileio.py           # PCD / STL / OBJ                      (Task 4)
    │   │   ├── core/template.py         # Template + 3 cách tạo                (Task 4)
    │   │   ├── core/registration.py     # lưới NN + ICP + chỉ số chất lượng    (Task 5)
    │   │   ├── core/search.py           # tìm kiếm tương quan 2.5D             (Task 6)
    │   │   ├── core/estimator.py        # pipeline + cổng chất lượng           (Task 7)
    │   │   ├── core/kalman.py           # StaticPoseKF                         (Task 8)
    │   │   ├── core/control.py          # luật điều khiển + FSM                (Task 9)
    │   │   ├── ros/conversions.py       # PointCloud2/Pose <-> numpy           (Task 10)
    │   │   ├── nodes/pose_estimator_node.py                                    (Task 11)
    │   │   ├── nodes/docking_controller_node.py                                (Task 12)
    │   │   ├── nodes/cloud_saver_node.py                                       (Task 13)
    │   │   ├── tools/make_template.py   # CLI tạo template                     (Task 13)
    │   │   └── tools/benchmark.py       # CLI đo độ chính xác + thời gian      (Task 7)
    │   ├── config/{estimator,controller,nav2_docking}.yaml
    │   ├── launch/{estimator,docking,nav2_docking}.launch.py
    │   ├── templates/{euro_pallet,rack,v_dock}.npz
    │   ├── rviz/docking.rviz
    │   └── test/test_*.py
    └── camera3d_docking_sim/
        ├── package.xml, setup.py, setup.cfg, resource/
        ├── camera3d_docking_sim/generate_world.py   # SDF từ core.shapes   (Task 14)
        ├── camera3d_docking_sim/e2e_docking_test.py # kịch bản E2E          (Task 15)
        ├── worlds/docking_world.sdf, models/docking_bot/{model.sdf,model.config}
        ├── config/{bridge.yaml,sim_estimator.yaml,sim_controller.yaml}
        └── launch/sim.launch.py
```

**Lệnh test chuẩn:**

- Lõi: `cd ros2_ws/src/camera3d_docking && PYTHONNOUSERSITE=1 python3 -m pytest test -q`
- Test có ROS: chạy sau khi `source env.sh`.

---

# PHẦN A — PHẦN MỀM

### Task 0: Workspace, gói, xử lý môi trường

**Files:**
- Create: `ros2_ws/src/camera3d_docking/{package.xml,setup.py,setup.cfg,pytest.ini,resource/camera3d_docking}`
- Create: `camera3d_docking/__init__.py`, `core/__init__.py`, `ros/__init__.py`, `nodes/__init__.py`, `tools/__init__.py`, `_env.py`
- Create: `env.sh`
- Test: `test/test_env.py`

**Interfaces:**
- Produces: `camera3d_docking._env.ensure_clean_python() -> None`. Nếu `PYTHONNOUSERSITE != "1"` thì đặt biến này và gọi `os.execv(sys.executable, [sys.executable, *sys.argv])`. Mọi entry point gọi hàm này **trước** khi import numpy.

- [ ] **Step 1: Viết test.** `test_env_flag_set_under_pytest` khẳng định `os.environ.get("PYTHONNOUSERSITE") == "1"` và `numpy.__version__.startswith("1.")`.
- [ ] **Step 2: Chạy.** `PYTHONNOUSERSITE=1 python3 -m pytest test/test_env.py -q`. Kỳ vọng: FAIL vì chưa có gói.
- [ ] **Step 3: Tạo gói.** `package.xml` dạng format 3, build_type `ament_python`, exec_depend: rclpy, sensor_msgs, geometry_msgs, nav_msgs, std_msgs, std_srvs, tf2_ros, python3-numpy, python3-scipy. `setup.py` khai báo entry points `pose_estimator`, `docking_controller`, `cloud_saver`, `make_template`, `benchmark`, và data_files cho config/launch/templates/rviz. `env.sh` gồm `source /opt/ros/jazzy/setup.bash`, source `install/setup.bash` nếu có, `export PYTHONNOUSERSITE=1`.
- [ ] **Step 4: Chạy lại test.** Kỳ vọng PASS. Sau đó `cd ros2_ws && colcon build --symlink-install`. Kỳ vọng build thành công. Nếu lỗi do dấu cách trong đường dẫn: `ln -s "<project>/ros2_ws" ~/docking_ws`, build từ đó và ghi lại vào README.

### Task 0b: Ghim ứng dụng lên taskbar

**Files:** Create: `tools/pin_apps_to_dock.sh`

- [ ] **Step 1: Tạo launcher.** Tạo `~/.local/share/applications/ros2-{rviz2,rqt,gazebo,plotjuggler}.desktop`:
  - `Exec=bash -c 'source /opt/ros/jazzy/setup.bash && PYTHONNOUSERSITE=1 <cmd>'`
  - Icon tìm bằng `find /opt/ros/jazzy`, có icon dự phòng.
- [ ] **Step 2: Cập nhật favorites.** Đọc `gsettings get org.gnome.shell favorite-apps`, nối thêm các launcher trên và `cloudcompare_cloudcompare.desktop`, bỏ trùng, giữ nguyên thứ tự cũ.
- [ ] **Step 3: Kiểm tra.** `gsettings get …` phải chứa 5 mục mới. `desktop-file-validate` không báo lỗi.

### Task 1: `core/geometry.py`

**Interfaces — Produces:**
- `wrap_angle(a) -> float | ndarray` (về khoảng [−π, π))
- `rot_z(yaw) -> (3,3)`, `so3_exp(w:(3,)) -> (3,3)`
- `make_T(R, t) -> (4,4)`, `T_from_xyz_yaw(x, y, z, yaw)`, `T_from_xyz_rpy(x, y, z, roll, pitch, yaw)`, `inv_T(T)`
- `yaw_of(T_or_R) -> float`, `pose2d(T) -> (x, y, yaw)`, `transform_points(T, P:(N,3)) -> (N,3)`
- `quat_from_R(R) -> (x, y, z, w)`, `R_from_quat(q)`
- `R_LINK_FROM_OPTICAL = [[0,0,1],[-1,0,0],[0,-1,0]]`

- [ ] **Step 1: Viết test** (`test/test_geometry.py`):
  - `test_inv_T_roundtrip`: `inv_T(T) @ T ≈ I` (atol 1e-12).
  - `test_yaw_of_rot_z`: `yaw_of(rot_z(0.3)) == 0.3`.
  - `test_quat_roundtrip`: 50 R ngẫu nhiên, sai khác ≤ 1e-9.
  - `test_wrap_angle`: `wrap_angle(3π/2) ≈ −π/2`.
  - `test_so3_exp_matches_rot_z`: `so3_exp([0,0,0.7]) ≈ rot_z(0.7)`.
  - `test_optical_convention`: `R_LINK_FROM_OPTICAL @ [0,0,1] == [1,0,0]` và `@ [1,0,0] == [0,-1,0]`.
- [ ] **Step 2: Chạy.** `pytest test/test_geometry.py`. Kỳ vọng FAIL (ImportError).
- [ ] **Step 3: Cài đặt.** Rodrigues cho `so3_exp`. Quaternion dùng thuật toán Shepperd, ổn định số.
- [ ] **Step 4: Chạy lại.** Kỳ vọng PASS.

### Task 2: `core/cloud.py`

**Interfaces — Produces:**
- `finite_points(P) -> P`
- `crop_box_mask(P, lo:(3,), hi:(3,)) -> bool[N]`
- `voxel_downsample(P, voxel, min_points=1) -> (Q, counts)`: trọng tâm mỗi voxel.
- `estimate_normals(P, k=10, viewpoint=None) -> (N,3)`: PCA + `cKDTree`, quay về phía viewpoint.
- `PlaneFit(normal, d, inliers)` với quy ước n·p + d = 0, n_z > 0.
- `fit_plane_ransac(P, threshold=0.01, iterations=100, rng=None) -> PlaneFit | None`
- `leveling_transform(normal, d, floor_z=0.0) -> (4,4)`

- [ ] **Step 1: Viết test:**
  - `test_voxel_downsample_reduces_and_preserves_centroid`: lưới 10k điểm, voxel 0.05 → ít điểm hơn, trọng tâm lệch < 1e-3.
  - `test_voxel_min_points_removes_isolated`: điểm lẻ bị loại khi `min_points=2`.
  - `test_estimate_normals_plane_oriented_to_viewpoint`: mặt z = 0, viewpoint (0,0,1) → mọi n ≈ (0,0,1), sai lệch < 2°.
  - `test_ransac_plane_with_outliers`: 70% điểm trên mặt nghiêng 5°, 30% nhiễu → pháp tuyến lệch < 0.5°.
  - `test_leveling_transform_flattens_tilted_floor`: sàn nghiêng 3°, cao 0.05 m → sau biến đổi \|z\| < 2 mm.
- [ ] **Step 2: Chạy.** Kỳ vọng FAIL.
- [ ] **Step 3: Cài đặt.**
  - Voxel: khóa nguyên `floor(P/v)` gộp vào một int64 (21 bit mỗi trục, có offset) → `np.unique(return_inverse)` → `np.bincount` theo từng trục.
  - RANSAC: lấy mẫu 3 điểm theo lô vector hóa, sau đó refit bằng SVD trên tập inlier.
  - Leveling: xoay nhỏ nhất đưa n về +z, rồi dịch z.
- [ ] **Step 4: Chạy lại.** Kỳ vọng PASS.

### Task 3: `core/shapes.py` (mô hình hộp, ray-cast, camera ảo)

**Interfaces — Produces:**
- `Box(center:(3,), size:(3,), yaw=0.0)`, frozen dataclass, dùng tuple.
- `euro_pallet_boxes(depth=1.2, width=0.8, block_h=0.078, board_t=0.022, block_widths=(0.10,0.145,0.10), block_depth=0.145) -> list[Box]`. Vào từ mặt rộng `width`; mặt trước tại x = 0.
- `rack_boxes(bay_width=1.0, depth=0.9, height=2.0, upright=(0.08,0.08), beam_levels=(0.15, 1.2), beam_h=0.10) -> list[Box]`
- `v_dock_boxes(opening=0.5, depth=0.2, height=0.25, thickness=0.03) -> list[Box]`
- `place_boxes(boxes, x, y, yaw) -> list[Box]`: đặt mô hình lên sàn world.
- `PinholeCamera(width, height, fx, fy, cx, cy)` với `.from_hfov(w, h, hfov_deg)` và `.ray_dirs(stride=1) -> (H',W',3)` trong frame optical.
- `raycast_boxes(origin:(3,), dirs:(N,3), boxes, floor_z=None, max_range=inf) -> (t:(N,), normal:(N,3))`, phương pháp slab trong hệ cục bộ của từng hộp.
- `DepthNoise(sigma_const=0.001, sigma_quad=0.002)` với σ(z) = c + q·z².
- `render_cloud(cam, T_world_optical, boxes_world, floor_z=0.0, noise=None, rng=None, max_range=6.0, stride=1) -> (H',W',3) float32` trong frame optical, NaN ở chỗ không trúng.

- [ ] **Step 1: Viết test:**
  - `test_raycast_hits_box_front_face`: hộp tâm (2,0,0.5) cỡ 1 → tia +x từ gốc có t = 1.5, normal (−1,0,0).
  - `test_raycast_rotated_box`: hộp xoay 90°, cỡ (1, 0.2, 1) → t = 1.9.
  - `test_raycast_floor`: tia chúc xuống 45° từ z = 1 trúng sàn ở t = √2.
  - `test_render_points_lie_on_surfaces`: không nhiễu → mọi điểm cách bề mặt hộp hoặc sàn < 1e-6.
  - `test_euro_pallet_geometry`: bao (0..1.2, ±0.4, 0..0.144). Ở z = 0.06 trên mặt x = 0, có hai khe rộng 0.2275 tại y = ±0.18625.
- [ ] **Step 2: Chạy.** Kỳ vọng FAIL.
- [ ] **Step 3: Cài đặt.** Vector hóa theo (B hộp × N tia). Nếu cần thì chia N thành khối 50k tia.
- [ ] **Step 4: Chạy lại.** Kỳ vọng PASS.

### Task 4: `core/fileio.py` + `core/template.py`

**Interfaces — Produces:**
- `read_pcd(path) -> dict[str, ndarray]`: ascii và binary. Gặp `binary_compressed` thì ném `ValueError` có hướng dẫn.
- `write_pcd(path, points, normals=None, binary=True)`
- `read_mesh(path) -> (V:(M,3), F:(K,3))`: STL ascii/binary, OBJ (v/f, tam giác hóa đa giác).
- `Template(points, normals, name="", meta=dict)` với `.bounds`, `.save(path)` (.npz/.pcd), `Template.load(path)`, `.facing_mask(cam_pos_obj) -> bool[N]`.
- `template_from_boxes(boxes, name, spacing=0.01, cam_heights=(0.25,0.6), distances=(1.2,2.0), angles_deg=(-40,-20,0,20,40)) -> Template`: chỉ giữ điểm thấy được; normal lấy chính xác từ ray-cast.
- `template_from_points(points_obj, viewpoint_obj, voxel=0.01, k_normals=12, name="", meta=None) -> Template`
- `template_from_mesh(V, F, spacing=0.01, approach_dir=(1,0,0), name="") -> Template`

- [ ] **Step 1: Viết test:**
  - `test_template_from_boxes_only_visible`: với hộp đặc sâu 0.5, không có điểm nào có x > 0.49.
  - `test_template_save_load_npz_roundtrip`
  - `test_pcd_roundtrip_ascii_and_binary`
  - `test_read_binary_stl_cube`: 12 tam giác, đỉnh trong [0,1]³.
  - `test_template_from_mesh_culls_back_faces`: không có normal nào có n_x > 0.
- [ ] **Step 2–4:** FAIL → cài đặt → PASS.

### Task 5: `core/registration.py`

**Interfaces — Produces:**
- `NearestTemplateGrid(points, resolution=0.01, margin=0.25, max_cells=4_000_000)` với `.query(Q:(N,3)) -> idx:int64[N]` (−1 nếu ngoài lưới). Lưới được dựng bằng `cKDTree` lúc khởi tạo.
- `IcpParams`:
  - Tầng và tương ứng: `max_corr_dists=(0.15,0.06,0.03)`, `max_iters=15`, `dof="xyz_yaw"` (một trong `xy_yaw` / `xyz_yaw` / `6dof`), `tangent_weight=0.05`.
  - Tối ưu: `kernel="tukey"`, `kernel_min_scale=0.01`, `lm_lambda=1e-4`, `eps_t=1e-4`, `eps_r=1e-4`.
  - Chỉ số: `inlier_dist=0.02`, `coverage_radius=0.03`, `crop_margin=0.2`, `cov_inflation=25.0`, `yaw_scale_len=0.5`.
- `IcpResult`: `T_base_obj (4,4)`, `converged`, `iterations`, `rmse`, `inlier_ratio`, `coverage`, `n_corr`, `cov_xyyaw (3,3)`, `sigma_xy`, `sigma_yaw`, `eig_ratio`, `degenerate`, `score = coverage*inlier_ratio`.
- `icp_point_to_plane(scene_base, template, T_init, params, nn=None, camera_pos_base=None, scene_tree=None) -> IcpResult`

- [ ] **Step 1: Viết test.** Dữ liệu sinh bằng `render_cloud` với pallet EUR, camera cao 0.3 m, chúc 10°.
  - `test_nn_grid_matches_kdtree_within_resolution`: khoảng cách tới điểm grid trả về ≤ khoảng cách đúng + 0.87·res.
  - `test_icp_recovers_pallet_pose`: thật (1.5, 0.2, 0.0, 10°), khởi tạo lệch (−0.10, +0.08, −6°) → sai < 5 mm và < 0.3°, `converged`.
  - `test_icp_lateral_offset_recovered`: nhìn thẳng, lệch ngang 6 cm → sai ngang < 1 cm.
  - `test_icp_flags_degenerate_for_flat_wall`: template tấm phẳng → `degenerate is True`.
  - `test_icp_quality_low_for_wrong_object`: cảnh là rack, template là pallet → `coverage < 0.35` hoặc `inlier_ratio < 0.5`.
- [ ] **Step 2: Chạy.** Kỳ vọng FAIL.
- [ ] **Step 3: Cài đặt.** Gọi q = T_vật←base·p, e = q − m.
  - Jacobian pháp tuyến (theo [t, ω]): `[nᵀ, (q×n)ᵀ]`.
  - Jacobian tiếp tuyến: `[I, −[q]×]` với trọng số ε.
  - Cập nhật trái `T ← ΔT·T`. Chọn cột theo DOF.
  - Covariance (δ trong hệ vật) đổi sang (x, y, yaw) của vật trong base theo `J = [[−R_bo(2×2), 0], [0, −1]]`.
- [ ] **Step 4: Chạy lại.** Kỳ vọng PASS.

### Task 6: `core/search.py`

**Interfaces — Produces:**
- `SearchParams(grid_res=0.03, yaw_range_deg=45, yaw_step_deg=3, n_bands=3, blur_sigma_cells=1.0, free_weight=0.5, free_margin=0.05, top_k=5, nms_dist=0.15, nms_yaw_deg=10)`
- `Hypothesis(x, y, yaw, score)`
- `CorrelativeSearch(template, params)` với `.search(scene_base, roi_lo:(2,), roi_hi:(2,)) -> list[Hypothesis]`, sắp giảm dần theo score.

- [ ] **Step 1: Viết test:**
  - `test_search_finds_pallet`: pallet (2.0, −0.3, 12°) + tường phía sau + hộp nhiễu → giả thuyết tốt nhất lệch < 0.06 m và < 4°.
  - `test_search_wall_scores_lower_than_pallet`: điểm tốt nhất khi chỉ có tường < 0.6 × điểm khi có pallet.
  - `test_search_hypotheses_are_distinct`: mọi cặp cách nhau ≥ `nms_dist` hoặc ≥ `nms_yaw`.
- [ ] **Step 2–4:** FAIL → cài đặt (`scipy.signal.fftconvolve(L, P[::-1, ::-1], mode="same")`; cache template theo từng góc; giữ cực đại cục bộ bằng `maximum_filter` rồi NMS) → PASS.

### Task 7: `core/estimator.py` + `tools/benchmark.py`

**Interfaces — Produces:**
- `Status(str, Enum)`: `OK, NO_POINTS, NOT_FOUND, LOW_QUALITY, DEGENERATE, AMBIGUOUS`
- `QualityParams(min_coverage=0.35, min_inlier_ratio=0.5, max_rmse=0.015, max_sigma_xy=0.02, max_sigma_yaw_deg=1.5, min_eig_ratio=1e-3, ambiguity_ratio=0.9, ambiguity_dist=0.05, ambiguity_yaw_deg=3.0, track_jump_xy=0.25, track_jump_yaw_deg=15)`
- `EstimatorParams`:
  - ROI và sàn: `roi_min=(0.25,-1.5,0.03)`, `roi_max=(3.5,1.5,2.0)` (z tính trên sàn), `floor_z=0.0`.
  - Tiền xử lý: `voxel_size=0.02`, `min_points_per_voxel=1`, `min_scene_points=50`.
  - Cân bằng sàn: `floor_leveling=True`, `floor_band=0.15`, `floor_ransac_thresh=0.015`, `floor_max_tilt_deg=10`, `floor_max_dz=0.10`, `floor_ema=0.3`.
  - Tham số con: `search: SearchParams`, `icp: IcpParams`, `quality: QualityParams`.
- `Detection(status, T_base_obj|None, cov_xyyaw|None, quality:dict, timings:dict, n_points:int, message:str)`
- `DockPoseEstimator(template, params)` với `.process(points_base, camera_pos_base=None, prior_T_base_obj=None) -> Detection`, `.reset()`, `.leveling_T`
- `tools/benchmark.py`: `main(argv=None) -> int`, có cờ `--quick`. In bảng sai số và thời gian từng khâu.

- [ ] **Step 1: Viết test:**
  - `test_estimator_detects_pallet_from_scratch`: nhiễu stereo, pitch 10° → sai < 1.5 cm và < 1°, `status == OK`.
  - `test_estimator_tracking_with_prior`: có prior → `OK`, thời gian < chế độ tìm kiếm.
  - `test_estimator_empty_and_nan_cloud`: mảng rỗng và mảng toàn NaN → `NO_POINTS`, không exception.
  - `test_estimator_not_found_on_wall_only` → status khác `OK`.
  - `test_estimator_floor_leveling_corrects_pitch_error`: điểm có lỗi pitch ngoại tham số 2° → sai < 2 cm.
  - `test_estimator_ambiguous_two_pallets`: hai pallet tại y = ±0.65 → `AMBIGUOUS`. Thu ROI về y > 0 → `OK`, đúng pallet trái.
  - `test_benchmark_quick_runs`: `main(["--quick"]) == 0`.
- [ ] **Step 2–4:** FAIL → cài đặt → PASS.

### Task 8: `core/kalman.py`

**Interfaces — Produces:** `StaticPoseKF(q_xy=0.005, q_yaw=radians(0.2), gate_chi2=14.16, reset_after=5)`:
- Phương thức: `.update(z:(3,), R:(3,3), stamp:float) -> bool`, `.predict(stamp)`, `.reset()`.
- Thuộc tính: `.initialized`, `.x`, `.P`, `.last_update`.

- [ ] **Step 1: Viết test:**
  - `test_kf_converges_under_noise`: 50 phép đo σ = 1 cm → sai < 4 mm.
  - `test_kf_rejects_outlier`: lệch 1 m → trả `False`, trạng thái không đổi.
  - `test_kf_yaw_wraparound`: đo quanh ±π → yaw ước lượng ≈ π, không về 0.
  - `test_kf_reinitializes_after_consistent_outliers`: 5 phép đo nhất quán ở vị trí mới → chuyển sang vị trí mới.
- [ ] **Step 2–4:** FAIL → cài đặt → PASS.

### Task 9: `core/control.py`

**Interfaces — Produces:**
- `ControlLawParams(k_phi=3.0, k_delta=2.0, beta=0.4, lam=2.0, v_max=0.25, v_min=0.02, w_max=0.75, slowdown_radius=0.25)`
- `smooth_control(x, y, yaw, p, backward=False) -> (v, w)`: đích tính trong hệ robot.
- `DockState(str, Enum)`: `IDLE, WAIT_DETECTION, APPROACH, BLIND, DOCKED, RETREAT, FAILED`
- `FsmParams`:
  - Dung sai và vùng mù: `goal_tol_x=0.01`, `blind_zone=0.6`.
  - Timeout: `detection_timeout=5.0`, `lost_timeout=1.0`, `approach_timeout=60.0`.
  - Thử lại: `max_retries=2`, `retreat_distance=0.5`, `retreat_speed=0.1`.
  - Giới hạn gia tốc: `acc_v=0.5`, `acc_w=1.5`.
  - `backward=False`.
- `DockingFSM(law, p)`:
  - Phương thức: `.start(now)`, `.cancel()`, `.step(now, target_base:(x,y,yaw)|None, detection_age:float|None, contact=False) -> (v, w)`.
  - Thuộc tính: `.state`, `.message`, `.retries`.

- [ ] **Step 1: Viết test** (mô phỏng xe một bánh, dt = 0.05):
  - `test_control_converges_from_offsets` (parametrize lệch ngang ±0.3 m, lệch hướng ±20°, cách 1.5 m) → \|ngang\| < 1 cm, \|yaw\| < 1°, luôn có v ≥ 0.
  - `test_control_backward_docking`: v ≤ 0 suốt quá trình; sai số cuối < 1 cm / 1°.
  - `test_fsm_happy_path_reaches_docked`
  - `test_fsm_fails_without_detection`: sau `detection_timeout` → `FAILED`.
  - `test_fsm_blind_zone_ignores_lost_detection`
  - `test_fsm_retreat_and_retry`: mất dấu ngoài vùng mù → `RETREAT` → `WAIT_DETECTION`, `retries == 1`.
- [ ] **Step 2–4:** FAIL → cài đặt theo công thức ở spec §9 → PASS.

### Task 10: `ros/conversions.py` (cần ROS)

**Interfaces — Produces:**
- `pointcloud2_to_xyz(msg, stride=1) -> float32[N,3]`: hiểu `fields`, `point_step`, cờ big-endian, cloud organized; bỏ NaN.
- `xyz_to_pointcloud2(P, frame_id, stamp) -> PointCloud2`
- `transform_to_T(tf_msg.transform)`, `pose_to_T(pose)`, `T_to_pose(T)`

- [ ] **Step 1: Viết test:**
  - `test_pc2_roundtrip`
  - `test_pc2_organized_stride`: cloud 4×6 với stride 2 → 6 điểm đúng vị trí.
  - `test_pc2_skips_nan`
  - `test_pc2_xyzrgb_padding`: cloud có trường rgb và padding vẫn đọc đúng.
  - `test_pose_roundtrip`
- [ ] **Step 2–4:** FAIL → cài đặt (`np.frombuffer` với dtype dựng từ `fields`) → PASS.

### Task 11: `nodes/pose_estimator_node.py` + config + launch + RViz

**Interfaces:**
- **Consumes:** Task 5–10.
- **Produces:**
  - Topic `detected_dock_pose` (`PoseStamped` @ `fixed_frame`), `~/pose_raw`, `~/aligned_template`, `~/roi_cloud`, `~/status` (JSON).
  - Service `~/reset`, `~/enable`.
  - Tham số theo spec §11, nạp từ `config/estimator.yaml`.
  - Launch `estimator.launch.py`, với các arg `points_topic`, `template`, `params_file`, `use_sim_time`.

- [ ] **Step 1: Viết test tích hợp** (`test/test_estimator_node.py`, rclpy chạy trong tiến trình). Một node giả phát `PointCloud2` render pallet tại (1.6, 0.1, 8°), TF tĩnh `base_link→camera_optical`, TF `odom→base_link` đơn vị.
  - `test_node_publishes_detected_pose`: trong 10 s nhận pose sai < 2 cm và < 1.5°.
  - `test_node_survives_missing_tf`: phát cloud trước khi có TF → không crash, vẫn ra pose sau khi TF xuất hiện.
  - `test_node_holds_pose_when_cloud_stops`: ngừng phát cloud → pose vẫn được phát trong `hold_timeout`.
- [ ] **Step 2–4:** FAIL → cài đặt → PASS. Callback chỉ xử lý khung mới nhất (QoS depth 1) và tôn trọng `max_rate_hz`. Timer 10 Hz phát pose giữ (hold) / vùng mù.

### Task 12: `nodes/docking_controller_node.py` + cấu hình Nav2

**Interfaces:**
- **Consumes:** `DockingFSM`, `smooth_control` (Task 9); topic `detected_dock_pose`.
- **Produces:**
  - Topic `cmd_vel`, `~/state`.
  - Service `~/start`, `~/cancel`.
  - `config/controller.yaml`, `config/nav2_docking.yaml` (plugin `opennav_docking::SimpleNonChargingDock`, `use_external_detection_pose: true`, `filter_coef: 0.0`).
  - Launch `docking.launch.py` (estimator + controller) và `nav2_docking.launch.py`.

- [ ] **Step 1: Viết test.** `test_controller_node_drives_towards_pose`: phát pose đích cách 1 m → gọi `~/start` → nhận `cmd_vel` với v > 0, `~/state == APPROACH`.
- [ ] **Step 2–4:** FAIL → cài đặt (timer 20 Hz; đích = T_odom_vật ⊕ offset, đổi sang `base_link` qua TF) → PASS.

### Task 13: Công cụ template: `make_template` CLI + `cloud_saver_node`

**Interfaces — Produces:**
- `tools/make_template.py:main(argv) -> int` với các lệnh con:
  - `pallet | rack | vdock` (tham số kích thước) `-o file.npz`
  - `from-mesh FILE -o`
  - `from-cloud FILE.pcd --object-pose x y yaw [--crop ...] -o`
  - `export-pcd IN.npz OUT.pcd`
- `nodes/cloud_saver_node.py`: lưu N cloud kế tiếp (đã đổi về `base_frame`) thành `.pcd`.
- Sinh sẵn `templates/euro_pallet.npz`, `rack.npz`, `v_dock.npz`.

- [ ] **Step 1: Viết test.**
  - `test_make_template_pallet_cli`: chạy `main(["pallet","-o",tmp])` → file nạp được, có > 500 điểm.
  - `test_make_template_from_cloud_cli`: dùng PCD tổng hợp.
- [ ] **Step 2–4:** FAIL → cài đặt → PASS. Sau đó chạy CLI để sinh 3 template mặc định.

### Task 14: Gói mô phỏng `camera3d_docking_sim`

**Files:** `generate_world.py` (đọc `core.shapes`), `worlds/docking_world.sdf`, `models/docking_bot/model.sdf` (diff drive + `rgbd_camera` 424×240 @ 15 Hz, HFOV 87°, depth 0.2–6 m, nhiễu σ 5 mm, cao 0.30 m, chúc 10°, plugin PosePublisher làm ground truth), `config/bridge.yaml`, `launch/sim.launch.py` (các arg: `headless`, `rviz`, `x`, `y`, `yaw`, `target`).

- [ ] **Step 1:** Sinh world bằng `python3 -m camera3d_docking_sim.generate_world`. Kiểm tra bằng `gz sdf -k worlds/docking_world.sdf` → "Valid".
- [ ] **Step 2:** Khởi chạy headless `ros2 launch camera3d_docking_sim sim.launch.py headless:=true`. Kiểm tra:
  - `ros2 topic hz /camera/points` ≈ 15 Hz.
  - `/odom` và TF `odom→base_link` có dữ liệu.
- [ ] **Step 3:** **Xác minh quy ước frame của cloud Gazebo.** Điểm sàn phải có z ≈ 0 sau khi đổi về `base_link`. Chỉnh TF tĩnh `camera_link ↔ frame cloud` cho đúng.
- [ ] **Step 4:** Chạy estimator trên sim. `~/status` phải báo `OK`, pose lệch < 2 cm so với vị trí pallet trong world.

### Task 15: Test end-to-end trong Gazebo

**Files:** `camera3d_docking_sim/e2e_docking_test.py` (entry point `e2e_docking_test`), `scripts/run_e2e.sh`.

- [ ] **Step 1:** Kịch bản cho từng pose xuất phát trong `[(−1.4, 0.0, 0°), (−1.5, 0.25, 15°), (−1.3, −0.3, −20°)]` (tương đối so với đích):
  1. Spawn robot.
  2. Gọi `/docking_controller/start`.
  3. Chờ `DOCKED` hoặc `FAILED`, tối đa 90 s.
  4. So ground truth với pose đích.
- [ ] **Step 2:** Chạy `bash scripts/run_e2e.sh`. Kỳ vọng ≥ 3/3 PASS với \|ngang\| ≤ 2 cm và \|yaw\| ≤ 2°. In bảng kết quả.

### Task 16: README + hướng dẫn triển khai + launcher mô phỏng

- [ ] **Step 1:** `README.md` gồm: cấu trúc thư mục, build, chạy test, chạy mô phỏng, chạy trên robot.
- [ ] **Step 2:** `docs/HUONG_DAN_TRIEN_KHAI.md`, toàn bộ Phần B dưới đây ở dạng checklist chi tiết.
- [ ] **Step 3:** Thêm launcher "Docking Sim" lên taskbar (Task 0b, mục bổ sung).
- [ ] **Step 4:** Chạy toàn bộ test lần cuối và ghi kết quả.

---

# PHẦN B — TRIỂN KHAI LÊN ROBOT THẬT (người dùng thực hiện, có tiêu chí nghiệm thu)

| Bước | Việc cần làm | Nghiệm thu |
|------|-------------|-----------|
| **B1. Chọn & gắn phần cứng** | Camera tính depth **trên camera**: RealSense D435i/D455, Orbbec Gemini 2/335, OAK-D Pro. Tránh ZED nếu không có Jetson. Gắn chắc chắn, cao 0.25–0.5 m, chúc 5–15°, cổng USB3. Máy tính: RPi 5 (8 GB), Jetson Orin Nano, hoặc mini PC N100. | Camera thấy được mặt trước vật ở mọi khoảng cách 0.6–2.5 m. |
| **B2. Cài phần mềm** | Cài Ubuntu 24.04 → `bash tools/install_ros2_jazzy.sh --robot --camera <realsense\|orbbec\|oakd>` → copy `ros2_ws`, rồi `colcon build`. | `ros2 launch camera3d_docking estimator.launch.py` chạy được. |
| **B3. Cấu hình driver camera** | Độ phân giải depth thấp (424×240 hoặc 480×270), 15 FPS. Bật bộ lọc trên camera (decimation, spatial, temporal). Phát `PointCloud2` organized. | `ros2 topic hz` ổn định; CPU của driver < 25%. |
| **B4. Hiệu chuẩn ngoại tham số** | Đo vị trí và góc camera → `static_transform_publisher base_link→camera`. Kiểm tra bằng `~/status` (độ nghiêng sàn ước lượng < 1°) và đặt hộp ở vị trí biết trước. | Sai lệch vị trí hộp < 1 cm ở 1 m. |
| **B5. Tạo template** | Với mỗi loại vật: dùng mô hình tham số (đo kích thước thật), **hoặc** quét bằng `cloud_saver` từ 3–5 vị trí, rồi `make_template from-cloud`. Xem lại trong CloudCompare. | Overlay `~/aligned_template` trên RViz khớp với cloud thật. |
| **B6. Đo độ chính xác tĩnh** | Dán AprilTag lên vật làm ground truth (`apriltag_ros` đã cài). Đo lưới: khoảng cách {0.8, 1.2, 1.8, 2.5} m × góc {−30, −15, 0, 15, 30}°, mỗi điểm 100 khung. | Trung vị ≤ 1.5 cm / 1°, phân vị 95 ≤ 3 cm / 2°. Không có kết quả `OK` nào sai > 10 cm. |
| **B7. Tinh chỉnh điều khiển** | Chạy trong mô phỏng trước, sau đó trên robot ở `v_max` = 0.1 m/s. Chỉnh `blind_zone`, `dock_offset_x` và gia tốc. | Không dao động; không chạm vật trước điểm dừng. |
| **B8. Nghiệm thu docking** | 20 lần docking liên tiếp từ staging pose ngẫu nhiên (±0.2 m, ±15°). | ≥ 19/20 thành công; sai số cuối ≤ 2 cm / 1.5°. |
| **B9. Tích hợp hệ thống** | Có Nav2: `nav2_docking.launch.py` với database dock và staging pose. Không có Nav2: gọi `~/start` từ RCS/FMS. Lớp an toàn: e-stop, collision monitor, giới hạn tốc độ. | Chạy hết kịch bản nhận hàng → dock → rời dock. |
| **B10. Tối ưu hiệu năng** | Chạy `ros2 run camera3d_docking benchmark` trên robot. Nếu vượt 50 ms/khung: tăng `cloud_stride` hoặc `voxel_size`, giảm độ phân giải. Nếu vẫn chậm: port `registration.py` sang C++ (pybind11), hoặc dùng `small_gicp`. | ≥ 10 Hz ở chế độ bám, CPU < 50% một nhân. |
