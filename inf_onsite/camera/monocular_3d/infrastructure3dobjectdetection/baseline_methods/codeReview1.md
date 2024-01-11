main

Trainer()

Trainer.train()

-> train_one_epoch()

::3d loss

| loss_name          | keys                                                                         | desc |
| ------------------ | ---------------------------------------------------------------------------- | ---- |
| seg loss           | heatmap                                                                      |      |
| offset 2d          | offset_2d + mask_2d + indices                                                |      |
| size 2d            | size_2d + mask_2d + indices                                                  |      |
| offset 3d          | offset_3d + mask_3d + indeices                                               |      |
| size3d             | size_3d + mask_3d + indices                                                  |      |
| depth              | depth + indices + mask_3d                                                    |      |
| heading            | heading_bin + heading_res + indices + mask_2d                                |      |
| centerkpt offset   | center2kpt_offset_target + mask_center2kpt_offset + mask_3d                  |      |
| kpt_heatmap        | kpt_heatmap_target                                                           |      |
| kpt_heatmap offset | kpt_heatmap_offset_target + mask_kpt_heatmap_offset  + mask_2d + indices_kpt |      |

::label

input image

target

| label      | source |
| ---------- | ------ |
| heatmap    |        |
| offset_2d  |        |
| size_2d    |        |
| offset_3d  |        |
| size_3d    |        |
| depth      |        |
| heading    |        |
| center2kpt |        |
| heatmap    |        |
| heatmap    |        |

TODO:: calibration :
写死 -> 文件对应

```python
# dair_dataset.py

class DAIR_Dataset_MonoCon().__get_item__()
class DAIR_Dataset_MonoCon().get_calib()
DAIRCalibration(calib_file)
```

```python
# utils.py

def get_dair_calib_from_file()
    P2
    # 1x9 list
    # [cu 0  fu
    #  0  cv fv
    #  0  0  1 ]
    R0 # [I]
    return
class DAIRCalibration():
    self.P2
    self.R0
```

数据

```
/home/server/dataset/exported_dataset
├── training
│   ├── calib
│   ├── groundplane
│   ├── images
│   └── labels
├── validation
│   ├── calib
│   ├── groundplane
│   ├── images
│   └── labels
└── test
    ├── calib
    ├── groundplane
    └── images
```

测试: point.ipynb

# 3D 中心点代码

1. label 里要读两个中心点。
2. 2d 中心由 bbox 计算 -> center_2d
3. 3d 中心点: proj_center_2d 由 xyz 计算
   1. 生成 八个关键点 corners_3d
   2. 生成 center_3d : \[X,Y\] 图像平面
   3. 生成center_heatmap

# Loss 计算

1. 3d_center->heat map 或者2d_center->heat map
2. offset_3d: center_3d - center_heatmap: 如果用3d作为中心类型强转的 offset， 如果用2d作为中心，2d中心到3d中心的offset
3. offset_2d: center_2d -center_heatmap 使用2d作为中心的话会有offset。
4.
