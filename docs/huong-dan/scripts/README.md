# Script tạo hình cho sổ tay (`docs/huong-dan`)

Các hình trong sổ tay được sinh bằng chính code của gói `camera3d_docking`, nên khi thuật toán hoặc tham số thay đổi thì nên tạo lại.

| Script | Làm gì | Đầu vào | Đầu ra |
|---|---|---|---|
| `figs.py` | 13 hình số liệu f01–f12 (bản sáng + tối) | `core/` của gói; `data/run1.jsonl` cho f11 | `../img/f*-{light,dark}.webp` |
| `record_run.py` | Ghi pose thật của robot, trạng thái controller/estimator, `cmd_vel` trong lúc chạy mô phỏng | các topic ROS 2 | file JSONL (`data/run1.jsonl`) |
| `postproc.py` | Cắt ảnh chụp màn hình, đánh số chú thích, ghép video Gazebo + RViz | `data/sim_raw/` (ảnh chụp và video thô, không đưa vào git) | `../img/s*.webp`, `../img/dock-run.mp4` |

## Tạo lại hình số liệu

```bash
source env.sh
```

```bash
cd docs/huong-dan/scripts && python3 figs.py all
```

Chỉ tạo vài hình: `python3 figs.py f05 f06`. Hình f10 đo thời gian trên chính máy đang chạy nên số sẽ khác đôi chút.

## Tạo lại ảnh mô phỏng và video

1. Chạy mô phỏng: `ros2 launch camera3d_docking_sim sim.launch.py x:=1.1 y:=0.35 yaw:=-15`
2. Chụp màn hình vào `data/sim_raw/shot_*.png` (ví dụ `import -window root …`) và quay `data/sim_raw/raw_run1.mp4` bằng `ffmpeg -f x11grab`.
3. Song song: `python3 record_run.py data/run1.jsonl`, rồi gọi `ros2 service call /docking_controller/start std_srvs/srv/Trigger`.
4. `python3 postproc.py` — tọa độ cắt và vị trí số chú thích trong file được đặt cho màn hình 1920×1080 và bố cục cửa sổ lúc chụp; chụp lại thì phải chỉnh.
