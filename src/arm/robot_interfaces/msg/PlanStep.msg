# R2 梅林任务的一步指令，自描述：MOVE 段与 PICK/PUSH 段的字段共存
# 不属于当前 type 的字段在生成端置 0，下游按 type 取用即可

uint8 TYPE_MOVE = 0
uint8 TYPE_PICK = 1
uint8 TYPE_PUSH = 2
uint8 TYPE_WAIT = 3    # 原地等待一拍（R1 块尚未让开）：from_id==target_id==当前节点，无坐标动作

uint8 type
int32 target_id        # 1..13
int32 from_id          # MOVE: 起点节点; PICK/PUSH: 机器人当前所在节点

# === 三种 type 共用 ===
geometry_msgs/Pose2D prep_pose    # PublishGoal 直接用 (x, y, theta)
                                  # MOVE: 在 from 块上对齐 target，准备爬/降
                                  # PICK/PUSH: 在 from 块上对齐 target，准备伸臂

# === MOVE 用 ===
float64 abs_dh                    # |高度差| (m)，喂给 ClimbStair / DescendStair

uint8 STAIR_NONE = 0
uint8 STAIR_UP   = 1
uint8 STAIR_DOWN = 2
uint8 stair_dir

float64 turn_deg                  # 进入本段相对上一段朝向的转角(度)：+左转 -右转 0直行 ±180掉头
                                  # 车头初始朝向 1/2/3 侧；仅 MOVE 有意义

# === PICK / PUSH 用 ===
float64 cube_x                    # 物块在 map 下的 x
float64 cube_y                    # 物块在 map 下的 y
float64 block_height              # 所在方块高度 (m)
float64 grasp_yaw                 # 抓取/推取对齐朝向；使车体右侧朝向目标
