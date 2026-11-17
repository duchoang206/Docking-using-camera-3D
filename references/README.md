# Repo tham khảo

Sáu repo dưới đây được clone vào thư mục này để **đọc và học**. Dự án không cần chúng khi build hay chạy.
Chúng không được đưa vào git của dự án: tổng cộng khoảng 940 MB, và mỗi thư mục là một repo git riêng.
Bài học rút ra từ từng repo nằm trong [spec thiết kế](../docs/superpowers/specs/2026-11-16-camera3d-docking-design.md).

| Thư mục | Repo gốc | Commit đã đọc (ngày commit) |
|---|---|---|
| `realtime-pose-estimation` | [snenyl/realtime-pose-estimation](https://github.com/snenyl/realtime-pose-estimation) | `01d1430d6a3b` (2026-02-26) |
| `yolo_ros` | [mgonzs13/yolo_ros](https://github.com/mgonzs13/yolo_ros) | `efcb2242a6ef` (2026-11-16) |
| `l2i_fusion_detection` | [AbdullahGM1/l2i_fusion_detection](https://github.com/AbdullahGM1/l2i_fusion_detection) | `be7c3776bb40` (2025-01-20) |
| `ROS-pose-estimation-module` | [HanjiangHu/ROS-pose-estimation-module](https://github.com/HanjiangHu/ROS-pose-estimation-module) | `aa4a1069fdb1` (2020-10-03) |
| `pallet_detection_and_segmentation` | [davidakhihiero/pallet_detection_and_segmentation](https://github.com/davidakhihiero/pallet_detection_and_segmentation) | `f26314785b89` (2024-11-18) |
| `opennav_docking` | [open-navigation/opennav_docking](https://github.com/open-navigation/opennav_docking) | `44ad8f877c45` (2026-01-20) |

Tải lại đúng các phiên bản trên (chạy trong thư mục `references/`):

```bash
clone() { git clone "https://github.com/$1.git" && git -C "${1#*/}" checkout --quiet "$2"; }
clone snenyl/realtime-pose-estimation                  01d1430d6a3b8c4951d2acd0621efcdd390b9862
clone mgonzs13/yolo_ros                                efcb2242a6ef10ab033dd45e8b84c2130dde90c8
clone AbdullahGM1/l2i_fusion_detection                 be7c3776bb40e91fbefbe196ce60ab5b2c81f043
clone HanjiangHu/ROS-pose-estimation-module            aa4a1069fdb1ddc393ffd858f09c084cd12eac89
clone davidakhihiero/pallet_detection_and_segmentation f26314785b895365871ccdf10093fbe47ccf8042
clone open-navigation/opennav_docking                  44ad8f877c4567698b4e9735126dd22072826276
```
