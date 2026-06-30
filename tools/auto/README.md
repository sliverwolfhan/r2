# 无标定自动对齐

现场新建完图,**一条命令**把雷达系 PCD 自动对齐到 map 系,不用 CloudCompare、不用手点锚点。

| 脚本 | 基准 | 适用 | 推荐 |
|---|---|---|---|
| **`auto_align_blocks.py`** | 12 个梅花桩 | 有梅花桩、要厘米级、原点要落在「红方右下角/蓝方左下角」 | ✅ **首选** |
| `auto_align_walls.py` | 矩形四面墙 | 桩被严重遮挡时兜底 | 兜底 |

> 需要手点锚点的通用方法(非矩形场地 / 任意场景)见上级目录 [../README.md](../README.md)。

---

# 方法一:梅花桩对齐(`auto_align_blocks.py`,首选)

场地里 4×3 共 12 个梅花桩,高度图案完全不对称,它们的 map 坐标就写在你现成的权威表
[block_red.yaml](../../src/action/at_r2_bt/yaml/block_red.yaml) /
[block_blue.yaml](../../src/action/at_r2_bt/yaml/block_blue.yaml) 里(每个坐标是方块中心,
来源是「手动锚点对齐出来的 map 系下方块中心」)。所以本脚本本质是:**用梅花桩自动复现你
当年手动锚点定义的那套 map 系**——精度上限就是当年那次手动锚点的精度;输出图和手动法
落在同一套 map,下游 GICP 零差异。

## 适用前提

- 现场扫到了 4×3 梅花桩(边对边拼成的阶梯地形)。
- 建图**起姿可控到 ±0.2m / ±10°**,且每次停同一固定起点 —— 把那个起点写进 `start_pose_map`。

## 原理(两阶段)

1. **高度图案粗对齐**:以 `start_pose_map` 为名义变换 T0 把点云摆到 map 系附近,在 12 个格心采样高度,
   与权威表比对、网格搜出朝向 + 大致位置。高度图案不对称 → **朝向唯一确定,天然破 90° 对称**。
2. **台阶棱精修**:相邻不同高度桩之间的竖直台阶棱,map 坐标已知(如 x=3.8/5.0 之间 → x=4.4)。
   脚本定位实测棱位,由宽到窄收窗(0.55→0.35→0.20→0.15)、取各轴**中值**校正平移(yaw 锁死),
   把图锁到 **~1.3cm**。
3. **收敛护栏**:高度图案不匹配 或 棱残差过大 → **报错拒绝出图**,不会写出会误导 GICP 的坏图。

**实测精度**:对已对齐成品图施加 ±0.2m/±10° 扰动回归,朝向误差 ~0°、平移误差 **~1.3cm**、
12/12 桩落在权威表真值。1.3cm 是 z 中点交叉定位的物理下限(系统偏置 + 散布),
对 GICP 先验图够用,达不到亚厘米。

## 用法

```bash
cd tools/auto
cp map.example.yaml my_map.yaml     # 按场地填一次(主要是 start_pose_map)
# 现场建完图后:
python3 auto_align_blocks.py \
  --map my_map.yaml \
  --zone blue \
  --in  ../../src/location/point_lio/PCD/scans1.pcd \
  --out ../../src/location/point_lio/PCD/scans1_map.pcd \
  --matrix-out ../../src/location/point_lio/PCD/T_map_lio.yaml
```

只想看变换不出图就省掉 `--out`。不写 `--zone` 时用 `my_map.yaml` 里的 `default_zone`。

## 填 my_map.yaml

方块真值**不用填**(直接读权威表)。需要你管的只有:

| 字段 | 含义 |
|---|---|
| `blocks_red` / `blocks_blue` | 权威表路径(相对本配置目录),样例已指好 |
| `default_zone` | 默认 red / blue,命令行 `--zone` 可覆盖 |
| `start_pose_map` | **建图开机时机器人在 map 系的起姿 `[x,y,yaw]`** —— 硬要求,见下 |
| 其余检测参数 | 一般保持默认(见样例注释) |

### ⚠️ `start_pose_map` 是硬要求

粗对齐靠它把点云摆到 map 系附近。**它必须接近真实开机起姿**(误差 ≲ ±0.3m/±10°),
否则远端的桩落不进检测窗、对齐不收敛,脚本会**报错拒绝出图**。最稳的做法是每次都停在
同一个固定起点,把那个点在 map 系的坐标写死在这里。红蓝区起姿也是镜像的(蓝区 y、yaw 取反)。

## 红区 / 蓝区一键切换

红蓝区关于 map 的 **x 轴镜像**(红 y>0、蓝 y<0),脚本按 `--zone` 直接读对应的权威表
(`block_blue.yaml` = `block_red.yaml` 的 y 取反),**零数据重复**。`start_pose_map` 按所建区填。

## 看懂输出 / 故障排查

| 现象 | 含义 / 处理 |
|---|---|
| `Height pattern does not match` | `--zone` 填反、差 90°、或 `start_pose_map` 离真实起姿太远 |
| `NOT CONVERGED (edge ...)` | 粗对齐位置太偏,桩没进窗 → 核对 / 收紧 `start_pose_map` |
| 平移残留 ~1-2cm | 正常 —— 台阶棱定位精度的下限,GICP 会在此基础上收敛 |

## 配到重定位包

输出的 map 系 PCD 路径填到
[small_gicp_relocalization_launch.py:52](../../src/location/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py#L52)
的 `prior_pcd_file`,下游导航零改动。

---

## 文件

```
tools/auto/
├── auto_align_blocks.py    # 方法一主脚本:梅花桩对齐(首选)
├── map.example.yaml        # 方法一配置样例
├── auto_align_walls.py     # 方法二主脚本:矩形墙对齐(兜底)
├── field.example.yaml      # 方法二配置样例
├── pcd_io.py               # 纯 Python PCD 读写 + LZF 解压(两脚本共用)
└── README.md               # 本文
```

## 依赖

只要 `numpy` + `pyyaml`(系统已有)。PCD 读写和 LZF 解压都是纯 Python,
**不依赖 PCL 命令行工具**(系统 `pcl_*` 工具链链接经常坏)。

## 验证局限(必须告知)

精度数字 ~1.3cm/~0° 全部基于唯一一张**已对齐成品图** `rc_2026_blue.pcd` 的扰动回归——
算法正确性已证,**真·现场原始图**(雷达系随机原点、墙更乱、桩可能扫不全)的鲁棒性未证。
建议下次练习时存一张原始图 + 当次手动 T 矩阵,做一次现场回归。

---

# 方法二:矩形墙对齐(`auto_align_walls.py`,兜底)

桩被严重遮挡、扫不全时,退而用规整的矩形四面墙对齐。

> 此法只定矩形的位置/朝向、有 90° 对称需靠起姿打破,**精度和定原点能力都不如梅花桩法**,仅作兜底。

## 适用前提

- 场地是**矩形**(四面墙规则、长宽规则给定)。
- 建图开机那一刻机器人的**起姿大致可控**(停在明确指向某个角的位置,误差几十度无所谓)。
- 墙在某个高度区间能被雷达扫到。

## 原理

1. 从新建 PCD 按高度区间裁出墙体点,用「最小面积外接矩形」自动拟合中心、长宽、朝向(扫角 0–90°,
   用 1/99 百分位 bbox 抗少量越界/障碍噪点)。
2. 把这个矩形贴到 `field.yaml` 里写死的 map 系矩形上。
3. 矩形有 90° 对称,产生 4 个候选变换。**用「建图大致起姿」打破对称**:LIO 原点 (0,0,facing +x)
   经过候选变换后应落在你给的起姿附近,选最接近的那个。
4. 按 `ground_z_map` 把 z 抬到地面对齐。

## 用法

```bash
cd tools/auto
cp field.example.yaml my_field.yaml   # 按场地填一次
# 现场建完图后:
python3 auto_align_walls.py \
  --field my_field.yaml \
  --in  ../../src/location/point_lio/PCD/scans1.pcd \
  --out ../../src/location/point_lio/PCD/scans1_map.pcd \
  --matrix-out ../../src/location/point_lio/PCD/T_map_lio.yaml
```

只想看变换不出图就省掉 `--out`。

## 填 field.yaml

样例见 [field.example.yaml](field.example.yaml),都是规则给定 / 你定义 map 系后写死的数,
不用现场测量:

| 字段 | 含义 |
|---|---|
| `field.length` / `field.width` | 矩形长边 / 短边真实长度(米) |
| `field.center_map` | 矩形中心在 map 系的坐标 |
| `field.yaw_map` | 矩形长边相对 map +x 的夹角(弧度),顺 +x 填 0 |
| `wall_z` | 墙体点高度区间(雷达系 z),滤地面/顶噪 |
| `ground_z_map` | 地面在 map 系的 z,一般 0 |
| `start_pose_map` | 建图开机大致起姿 `[x, y, yaw]`,**只用来打破矩形 90° 对称** |

## 红区 / 蓝区一键切换

`field.yaml` **只按红区填一套**,建蓝区图时加 `--zone blue`,脚本自动把
`center_map.y` / `start_pose.y` / `yaw_map` / `start_pose.yaw` 取反(长宽 / 墙高 / x / ground_z 不变):

```bash
# 红区(默认):
python3 auto_align_walls.py --field my_field.yaml --in red.pcd  --out red_map.pcd
# 蓝区:加一个开关
python3 auto_align_walls.py --field my_field.yaml --in blue.pcd --out blue_map.pcd --zone blue
```

不写 `--zone` 时用 `field.yaml` 里的 `default_zone`。

> 镜像只改"目标 map 系定义",最终 `T_map_lio` 仍是**刚体旋转+平移**,不会翻面(翻面会让扫描左右颠倒)。

## 看懂输出 / 故障排查

| 现象 | 原因 / 处理 |
|---|---|
| `Too few points in wall band` | `wall_z` 区间没覆盖到墙,调高度 |
| 拟合 L/W 偏差 >15% 告警 | 墙没扫全 / 区间裁到了障碍块或地面 / 场地不是闭合矩形 |
| 最优次优分接近告警 | 起姿没打破对称,换个更靠角、朝向更明确的起姿重建,或核对 `start_pose_map` |
| 选错了 90° 方向 | `start_pose_map` 填得离实际起姿太远,重填 |

## 配到重定位包

输出的 map 系 PCD 路径填到
[small_gicp_relocalization_launch.py:52](../../src/location/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py#L52)
的 `prior_pcd_file`,下游导航零改动。

## 局限

- 只适合**矩形**场地。L 形 / 不规则边界用上级目录的锚点法。
- 拟合精度取决于墙扫得全不全;GICP 重定位会在此基础上再收敛,这里只要把图摆到正确的 90° 朝向
  和大致位置即可。
