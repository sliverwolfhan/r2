# PCD 对齐到 map 坐标系（align_pcd_to_map.py）

把 Point-LIO 建出来的 PCD 通过若干已知锚点对齐到目标 `map` 坐标系，输出可以直接喂给 `small_gicp_relocalization` 的先验地图。

> **比赛现场没时间手点锚点？** 如果场地是**矩形**且建图起姿大致可控，用 [auto/](auto/) 目录里的 `auto_align_walls.py`，一条命令出图，不用 CloudCompare。本节的锚点法保留给非矩形场地 / 高精度需求。

## 解决什么问题

Point-LIO 启动那一刻机器人当前位置就是原点 (0,0,0)，朝向就是坐标轴方向，**每次开机原点都不一样**。但导航和行为树要用的是固定的 `map` 系（比如以场地某个角为原点）。本脚本通过场地里几个已知坐标的物理锚点，求出从 LIO 系到 map 系的刚体变换 `T_map_lio`，并把整张 PCD 套上这个变换。

## 文件

```
tools/
├── align_pcd_to_map.py        # 主脚本
├── anchors.example.yaml       # 锚点表样例
└── README.md                  # 本文
```

## 依赖

| 工具 | 用途 | 安装 |
|---|---|---|
| Python 3 + numpy + pyyaml | 求解变换矩阵 | 系统自带 / `pip3 install numpy pyyaml` |
| `pcl_transform_point_cloud` | 套变换到 PCD | `sudo apt install libpcl-tools` |

## 完整工作流

### 1. 布置锚点（一次性工作）

在场地里挑 **4~8 个** 几何特征明确、不会移动的位置作为锚点，例如：

- 墙体内拐的柱角
- 贴在墙上的反光板 / 二维码
- 货架金属支柱的角点
- 标定基座的圆心

布点原则：
- **平面分散**：尽量四角分布，覆盖整个场地，避免集中在一边
- **z 方向有差异**：不要全在地面，至少 2~3 个在不同高度（墙面贴标记 / 柱子上不同高度的角点），否则绕水平轴旋转解算会病态
- **避免共线**：3 个以上点不要在一条直线上

### 2. 测量锚点的 map 系真值

用卷尺、全站仪、CAD 图纸等手段测出每个锚点在你定义的 `map` 系下的精确坐标 (x, y, z)，单位 **米**。

map 系的定义由你决定（比如以场地左下角为原点，x 朝东，y 朝北，z 朝上）。一旦定好不要再改，否则所有定位都对不上。

### 3. 用 Point-LIO 建图

正常跑 Point-LIO，过程中确保每个锚点都被 LiDAR 扫到（**走近一点、多扫几遍**，点云密度足够才能在 CloudCompare 里点准）。

建完得到 PCD，比如 `src/location/point_lio/PCD/scans1.pcd`。

### 4. 在 CloudCompare 里读出锚点的 LIO 系坐标

```bash
# 启动
cloudcompare.CloudCompare              # snap 安装
# 或 flatpak run org.cloudcompare.CloudCompare
```

操作步骤：

1. `File → Open` 打开 PCD
2. 顶部工具栏点 **Point picking**（手指点小球的图标），或菜单 `Tools → Point picking`
3. 鼠标点击点云上的锚点位置，弹窗显示 `(x, y, z)`
4. 把这三个数抄下来，对应该锚点的 `lio:` 字段

点选小贴士：
- 点云密度小不好选时，先用 `Edit → Segment` 裁掉无关区域
- 显示设置里把点尺寸调小（1~2 像素），更容易看清结构
- 同一个锚点可以多点几次取平均，提高精度
- 注意 PCD 的视角，避免点到目标后面的点

### 5. 写锚点 yaml

复制示例改名：

```bash
cd tools
cp anchors.example.yaml my_anchors.yaml
```

按你测的数据填：

```yaml
anchors:
  - name: corner_A
    map: [0.000, 0.000, 0.000]      # 卷尺/CAD 量得（map 系，米）
    lio: [1.235, -0.412, 0.057]     # CloudCompare 读得（LIO 系，米）

  - name: corner_B
    map: [10.000, 0.000, 0.000]
    lio: [11.218, -0.689, 0.043]

  # ... 至少 3 个，建议 4~8 个
```

### 6. 跑脚本

**只算矩阵看残差**（先确认标定准了再变换）：

```bash
cd ~/wulin_r2
python3 tools/align_pcd_to_map.py \
  --anchors tools/my_anchors.yaml
```

**一步到位算矩阵 + 输出对齐后的 PCD**：

```bash
python3 tools/align_pcd_to_map.py \
  --anchors tools/anchors.example.yaml \
  --in  src/location/point_lio/PCD/scans.pcd \
  --out src/location/point_lio/PCD/scans_map.pcd \
  --matrix-out src/location/point_lio/PCD/T_map_lio.yaml
```

### 7. 配到重定位包

把对齐后的 PCD 路径填到 [small_gicp_relocalization_launch.py:52](../src/location/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py#L52)：

```python
"prior_pcd_file": "/home/zk/wulin_r2/src/location/point_lio/PCD/scans1_map.pcd",
```

启动重定位后，机器人 odom 输出的位姿就是 `map` 系下的位姿，行为树和导航可以直接用。

## 命令行参数

| 参数 | 必填 | 默认 | 说明 |
|---|---|---|---|
| `--anchors` | ✓ | - | 锚点 yaml 路径 |
| `--in` | ✗ | - | 待变换的输入 PCD（LIO 系） |
| `--out` | ✗ | - | 输出 PCD 路径（map 系），与 `--in` 配套 |
| `--matrix-out` | ✗ | `T_map_lio.yaml` | 4×4 变换矩阵保存路径 |
| `--allow-scale` | ✗ | false | 解相似变换（含尺度）；默认刚体 |

## 看懂输出

```
Solved T_map_lio (scale = 1.000000):
[[ 0.866 -0.500  0.000  5.000]
 [ 0.500  0.866  0.000 -2.000]
 [ 0.000  0.000  1.000  0.100]
 [ 0.000  0.000  0.000  1.000]]

Per-anchor residuals (m):
  corner_A     0.012
  corner_B     0.015
  corner_C     0.011
  corner_D     0.014
RMS: 0.013    max: 0.015
```

**残差** = 把每个锚点的 lio 坐标用算出来的 `T_map_lio` 变换后，与给定 map 真值的欧氏距离。这是你的标定精度评估。

| RMS 范围 | 评价 | 处理 |
|---|---|---|
| < 0.05 m | 良好 | 直接用 |
| 0.05 ~ 0.15 m | 可用但偏弱 | 找单点偏大的修正后重跑 |
| > 0.15 m | 不可用 | 必须排查，见下文 |

## 故障排查

### 某个锚点残差比其他大一个数量级

```
corner_A     0.013
corner_B     0.234   ← 离群
corner_C     0.011
```

→ 这个点 LIO 系坐标点错了，或 map 系真值量错了。回 CloudCompare 重新点 / 重新量这一个。**不要堆点数压低 RMS**，错点没排掉只是被平均掉。

### 多个锚点残差一致偏大

```
corner_A     2.31
corner_B     2.35
corner_C     0.74
corner_D     0.62
```

→ A 和 B 有共同的系统性错误。常见原因：
- map 坐标单位混了（mm 当 m，或反过来）
- xyz 顺序写反（特别是 z 和 y）
- A 和 B 的坐标互相填反
- 在 CloudCompare 里把相邻的相似特征点错了

### 全部残差都米级

→ 整套数据有系统性问题：
- map 系朝向理解错了（你以为 x 朝东，实际定义 x 朝北）
- lio 字段和 map 字段整体写反
- 用了错误的 PCD 文件（比如老的）
- 单位错误

### 残差均匀偏大但不离谱（5~15 cm）

→ 通常是测量精度本身的问题：
- 锚点的物理点位不够清晰（比如选了个大圆柱表面而不是角点）
- 卷尺测量误差累积
- LIO 建图本身漂移了，那次建图质量不好

可以加更多锚点缓解，或者重新建图。

### scale 显著偏离 1.0

加 `--allow-scale` 跑，如果输出的 scale 不在 0.99~1.01 之间，说明 LIO 有尺度偏差（IMU 内参问题）。治本是回去检查 IMU 标定，临时凑合可以保留 scale。

## 几条工程经验

- **锚点表存档**：`my_anchors.yaml` 和 `T_map_lio.yaml` 都进 git 仓库或场地档案，下次重建图能复用 / 对照
- **同一场地复用**：物理锚点不动，map 系定义不变，下次只需要重新填 `lio:` 字段（重新建图坐标会变），`map:` 不用改
- **变更场地后重标**：场地里柱子被挪、墙被砸了，必须重新测对应锚点的 map 真值
- **多次建图取最佳**：建图质量受人为影响大，建议同一场地建 2~3 次 PCD，对每次都跑对齐脚本看 RMS，选最好那次作为先验地图

## 算法原理（可选阅读）

脚本核心是 **Umeyama 算法**（封闭形式 SVD 解），求解：

```
min_T  Σᵢ ‖ T · pᵢ_lio − pᵢ_map ‖²
```

其中 `T` 是 4×4 刚体变换（默认）或相似变换（`--allow-scale`）。3 个不共线的锚点理论上能唯一求解；4 个以上做最小二乘。

实现见 [align_pcd_to_map.py:31-58](align_pcd_to_map.py#L31-L58)。

---

> 矩形场地的**无标定自动对齐**已挪到 [auto/](auto/) 目录，见 [auto/README.md](auto/README.md)。
