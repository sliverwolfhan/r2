# 理想模型 GICP 自动对齐（`auto_align_gicp.py`）

有一张**画在 map 系、原点落在红/蓝角的理想点云模型**(CAD 采样图 / 之前对齐好的成品图),
就可以让 GICP 把现场新建的雷达系 PCD 直接配到它上面——**配准解出来的变换就是 `T_map_lio`**,
不用 CloudCompare 点锚点、不要求场地是矩形、不要求有梅花桩。

> 需要手点锚点的通用方法见上级目录 [../README.md](../README.md)。

## 原理

- target = 理想模型(map 系),source = 现场 LIO 图(雷达系)。
- 用 6D 初始化位姿 `init_pose_map [x,y,z,roll,pitch,yaw]` 造名义变换 T0 当 GICP 初值,
  调用 [small_gicp](https://github.com/koide3/small_gicp) 精修,返回的 `T_target_source`
  就是 **`T_map_lio`**(map ← lio)。旋转约定与在线重定位节点的 `init_pose` 一致
  (ZYX:`Rz(yaw)·Ry(pitch)·Rx(roll)`)。
- 复用重定位包同一个 small_gicp C++ 库,外面用纯 Python(`pcd_io.py`)读写点云和套变换。

## ⚠️ 两个必须知道的点

1. **GICP 是局部优化,必须给初值** —— 这就是 6D `init_pose_map` 的作用。给不好初值会掉进
   局部极小、配歪还不自知。
2. **对称场地纯几何破不了 90/180 度对称** —— 也是靠 `init_pose_map` 提供正确朝向来破。
   所以这个方法**并没有免掉"要一个大致起姿"**,只是把"手点 6 个锚点"换成了"给一个位姿 + 自动精修"。
   质量提示(**只打印 WARNING,不阻止出图**):内点比 < `inlier_ratio_min`、未收敛、或精修相对
   T0 漂移 > `max_shift_m` / `max_yaw_deg`,只是提醒这次配准可能不准,`T_map_lio` 和对齐图照常输出。

## 构建 shim（只需一次）

```bash
cd ~/wulin_r2
cmake -S tools/auto/gicp -B tools/auto/gicp/build
cmake --build tools/auto/gicp/build
# 产出 tools/auto/gicp/build/small_gicp_align
```

## 用法

路径(标准地图 `prior_model`、待配准 `input_pcd`、输出 `output_pcd` / `matrix_out`)都写在
yaml 里,跑的时候不用再敲:

```bash
cd tools/auto
cp gicp.example.yaml my_gicp.yaml     # 填一次:三条路径 + init_pose_map
# 现场建完图后:
python3 auto_align_gicp.py --config my_gicp.yaml --zone blue
```

也可以用命令行覆盖 yaml 里的任一路径:

```bash
python3 auto_align_gicp.py --config my_gicp.yaml \
  --in  ../../src/location/point_lio/PCD/scans1.pcd \
  --out ../../src/location/point_lio/PCD/scans1_map.pcd
```

只想看变换不出图就把 `output_pcd` 留空(或不给 `--out`)。不写 `--zone` 时用 `default_zone`。

## 填 my_gicp.yaml

| 字段 | 含义 |
|---|---|
| `prior_model` | 红区理想模型 PCD 路径(相对配置目录),**必须已画在红区 map 系** |
| `prior_model_blue` | 蓝区模型(可选);留空则 `--zone blue` 时自动把红区模型 y 取反当蓝区目标 |
| `input_pcd` / `output_pcd` / `matrix_out` | 待配准的 LIO 图 / 对齐后输出图 / 变换矩阵路径(相对配置目录,可被 CLI 覆盖) |
| `init_pose_map` | 机器人在 map 系的 **6D 起姿** `[x,y,z,roll,pitch,yaw]`,GICP 初值 + 破对称(ZYX 约定,同在线节点 `init_pose`;红蓝镜像自动) |
| `downsampling_resolution` / `max_correspondence_distance` / `max_iterations` | GICP 参数,一般默认 |
| `inlier_ratio_min` / `max_shift_m` / `max_yaw_deg` | 质量提示阈值(只打印 WARNING,不阻止出图) |

> **z 轴朝下的 LIO 图**:若你的雷达系 z 是朝下的,`init_pose_map` 的 `roll` 填 `3.14`(≈180°)
> 把它翻成 map 系的 z-上;`z` 分量填地面抬升量(LIO 地面高度)。这不是错误,是必要的翻转。

## 看懂输出 / 故障排查

```
GICP: converged=True iters=6  inliers=27719/27719 (100%)  mean_res=0.0525 m
Refinement moved 0.328 m / +5.0deg from the init-pose seed
T_map_lio: translation=[1.998, -1.004, 0.163]  yaw=24.994deg
```

> 下面都是 **WARNING(不阻止出图)**,只提示这次配准可能不准,`T_map_lio` 和对齐图照样写出来。

| 现象 | 含义 / 处理 |
|---|---|
| `low inlier ratio ...` | 两云重叠少 → 起姿差 / 选错模型 / 红蓝区搞反,**或 LIO 图比标准图大很多**(场外点稀释,先裁到赛场范围) |
| `large drift from seed ...` | 精修跑飞(翻面/落错吸引域)→ 核对 `init_pose_map` 或 `--zone` |
| `GICP did not converge` | 初值太差 / 参数不合 → 调 `init_pose_map` 或加大 `max_correspondence_distance` |
| `Refinement moved` 只有几 cm/几度 | 正常,说明初值就很准,GICP 只做微调 |

> **inlier% 会被场外点稀释**:LIO 图若比标准图大很多,场外点永远配不上,inlier 天生偏低,不代表没配好。
> 判对不对以**对齐图和标准图叠起来贴不贴**为准,别只看百分比。

## 配到重定位包

输出的 map 系 PCD 路径填到
[small_gicp_relocalization_launch.py:52](../../src/location/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py#L52)
的 `prior_pcd_file`,下游导航零改动。

## 文件

```
tools/auto/
├── auto_align_gicp.py      # 主脚本:理想模型 GICP 对齐
├── gicp.example.yaml       # 配置样例
├── gicp/                   # C++ shim(链 small_gicp,构建一次)
│   ├── small_gicp_align.cpp
│   └── CMakeLists.txt
├── pcd_io.py               # 纯 Python PCD 读写 + LZF 解压
└── README.md               # 本文
```

## 依赖

- Python:`numpy` + `pyyaml`(系统已有)。PCD 读写和 LZF 解压都是纯 Python,
  **不依赖 PCL 命令行工具**(系统 `pcl_*` 工具链链接经常坏)。
- C++ shim:**small_gicp 库**(系统已装,重定位包 `small_gicp_relocalization` 也用它)。
  shim 只链 small_gicp、**不碰 PCL**,构建一次即可。

## 局限（诚实说明）

- 理想模型**必须真的画在正确 zone 的 map 系**(原点在红/蓝角、朝向对)。模型错了,配得再准也是错的。
- 精度上限是**图的质量**(LIO 漂移、墙没扫全、现场杂物),不是算法;GICP 已是精修的甜点,
  换更花哨的算法收益递减。真要"不给初值"得加全局配准前端(FPFH+RANSAC / TEASER++),
  但对称+杂物场地反而更易被骗,当前用 `init_pose_map` 播种更稳。
