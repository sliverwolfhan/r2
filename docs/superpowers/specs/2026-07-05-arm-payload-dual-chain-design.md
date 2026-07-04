# Arm Payload Dual-Chain KDL Model Switching

**Date:** 2026-07-05
**Status:** Draft
**Branch:** main
**Port reference:** `temporary` branch, files `arm_calc/{include,src/arm_calc,src/{arm_ctrl.cpp,test}}`

---

## TL;DR

把 `temporary` 分支中已验证的「双 KDL Chain 预热 + 运行时活动指针切换」实现平移到 `main` 分支，对在做末端 630 g / 350 mm 方块负载时进行精确动力学补偿。`ArmCalc` 构造入参从 1 条链扩展为 2 条，内部由 1×6 solver 变成 2×6 + 6 个活动指针；公开接口全部经活动指针。运行时通过 ROS 2 动态参数 `use_payload` 在 nominal 与 payload 两链之间 O(1) 切换。

---

## 问题陈述

`main` 分支的 `ArmCalc` 当前只持有单条 KDL `Chain`（由 `robot_state_publisher` 的 `robot_description` 解析得到）。控制器在做逆动力学补偿（重力 + 科氏 + 惯量）时，只反映空载情形。当末端装 630 g / 350 mm³ 方块负载后，真实的 `M(q)/C(q, q̇)/g(q)` 都明显变化——特别是 link6 与之前的 link — 导致实际力矩与模型力矩偏差变大、轨迹跟踪变差。

期望：在不用重启节点的情况下，动态在空载与带载两套 KDL 模型之间切换，让动力学补偿实时反映当前负载。

---

## 设计选择（方案对比）

| 方案 | 描述 | 取舍 |
|------|------|------|
| **A. Port temporary（推荐）** | 双链 + 2×6 solver + active-pointer 切换 | 已有可编译实现，零重写风险；12 solver 常驻内存（实际开销很小）；`O(1)` 切换；mutex 保护 |
| B. 单链手算 offset | 保留 ArmCalc 单链接口，在 `arm_ctrl.cpp` 里手算负载附加力矩 `J^T F + ΔM q̈ + ...` | 完全重写正确性难保证；KDL 已有功能被屏蔽 |
| C. 动态质量估计 | 运行时根据实测力矩辨识负载质量，连续插值 | 用户要求「复现 temporary」；本设计不治超出范围 |

**选定：方案 A。**

---

## 段落 1 — 核心架构 (`ArmCalc` 双链结构)

**文件修改：**
- `src/arm/arm_calc/include/arm_calc/arm_calc.hpp`
- `src/arm/arm_calc/src/arm_calc/arm_calc.cpp`

### 接口与字段变更

- 构造函数签名改为 `ArmCalc(const KDL::Chain& nominal_chain, const KDL::Chain& payload_chain)`
- 新增公开：
  - `void SetPayloadMode(bool use_payload)` — 切换活动链（O(1), mutex 保护, 同值 quick-return）
  - `bool payload_mode() const` — 返回当前模式
- 移除 `chain_`, `fk_solver_`, `jacobian_solver_`, `jdot_solver_`, `vel_solver_`, `ik_solver_`, `dynamic_solver_` 等单一命名成员
- 新增：
  - 双链 + use 标志：`chain_nominal_`, `chain_payload_`, `bool use_payload_`
  - 12 个预构建 solver（`fk_nominal_`, `fk_payload_`, `jac_*`, `jdot_*`, `vel_*`, `ik_*`, `dyn_*`）
  - 6 个活动指针：`fk_active_`, `jac_active_`, `jdot_active_`, `vel_active_`, `ik_active_`, `dyn_active_`
- 现有公开接口签名不变（`joint_pos`, `joint_vel`, `joint_torque_*`, `end_pose`, `end_state`, `jacobian`, `signal_arm_calc`, `set/get_joint_pd`）— 内部实现通过 `*active_` 访问

### Solver 组成（每链 6 个）

| solver | KDL 类 | 用途 |
|--------|---------|------|
| FK 位置正解 | `ChainFkSolverPos_recursive` | `end_pose` |
| 运动学 Jacobian | `ChainJntToJacSolver` | 速度映射 / 笛卡尔力求解 |
| Jacobian 时间导数 | `ChainJntToJacDotSolver` | `joint_acc` 中的 q̈ 项 |
| IK 速度 | `ChainIkSolverVel_pinv` | 速度级 IK（vel_solver 原有） |
| IK 位置 | `ChainIkSolverPos_LMA` (ε=1e-6, iter=200, eps_J=1e-10) | 位置级 IK |
| 逆动力学 | `ChainDynParam` (g=(0,0,-9.81)) | `JntToMass` / `JntToCoriolis` / `JntToGravity` |

### 活动指针切换逻辑

```cpp
void ArmCalc::SetPayloadMode(bool use_payload) {
    std::lock_guard<std::mutex> lock(active_mutex_);
    if (use_payload == use_payload_) return;
    if (use_payload) {
        fk_active_ = &fk_payload_;
        jac_active_ = &jac_payload_;
        jdot_active_ = &jdot_payload_;
        vel_active_ = &vel_payload_;
        ik_active_ = &ik_payload_;
        dyn_active_ = &dyn_payload_;
    } else {
        /* 翻回 nominal */
    }
    use_payload_ = use_payload;
    std::cout << "[ArmCalc] KDL chain switched to: "<< (use_payload?"PAYLOAD":"NOMINAL")<<std::endl;
}
```

所有公开接口都先 `lock_guard<mutex>(active_mutex_)` 再访问 `*active_*` — 切换期间阻塞、切换后恢复。

### 缓存复用（仅 1 份）

- `KDL::JntSpaceInertiaMatrix mass_matrix_`
- `KDL::JntArray coriolis_`, `gravity_`, `last_joint_solution_`
- `KDL::Jacobian jacobian_cache_`
- `KDL::JntArrayVel joint_vel_cache_`, `KDL::Twist jdot_qdot_cache_`

这些在 nominal/payload 之间共用（两者关节数/顺序一致，由同源模型保证）。

---

## 段落 2 — 参数接口与运行时切换

**文件修改：**
- `src/arm/arm_calc/src/arm_ctrl.cpp`

### 新 ROS 参数（`declare_parameters()`）

| 参数 | 类型 | 默认值 |
|------|------|--------|
| `use_payload` | `bool` | `false` |
| `payload_urdf_path` | `string` | `ament_index_cpp::get_package_share_directory("arm") + "/model_2/robotic_arm.urdf"` |

### 模型加载（`load_robot_description_and_build_solver()`）

顺序：
1. 复用既有 `fetch_robot_description()` 取 nominal 模型 → `tree` → `tree.getChain(base, tip, arm_chain_)`（无改动）
2. 新增 payload 分支：
   - `payload_urdf_path` 参数 → `ifstream` → `buffer << input.rdbuf()`
   - `kdl_parser::treeFromString(buffer.str(), payload_tree)`
   - `payload_tree.getChain(base_link_, tip_link_, payload_chain)`
3. 构造 `arm_calc_ = std::make_shared<ArmCalc>(arm_chain_, payload_chain);` — 注意从当前 main 的单参数调用变成双参数
4. 构造失败时立即 `throw std::runtime_error(...)` — 与现有一致（fatal，节点退出）

### 运行时切换（`on_parameters_changed()`）

```cpp
} else if (param.get_name() == "use_payload") {
    if (arm_calc_) {
        arm_calc_->SetPayloadMode(param.as_bool());
    }
}
```

返回 `SetParametersResult{.successful = true}`。不需重启节点，rqt / ros2 param set 都可操作。

---

## 段落 3 — 构建、测试、错误处理

**新增文件：**
- `src/arm/arm_calc/test/test_dual_chain.cpp`

### 不做

- 不改 `CMakeLists.txt` / `package.xml`
- 不启用 `ament_add_gtest` 或 `colcon test` 机制
- 不新增脚本 / 文档

### 单元测试 `test_dual_chain.cpp`

C++ 原生 `main` 自包含（c++17）。不依赖 gtest：
- 自由函数 `MakeChain(double link6_mass)` 自建 6-DOF 链：RotZ + 5×RotY，link1..5 `RigidBodyInertia` 固定，link6 `RigidBodyInertia(link6_mass, ...)`
- `ArmCalc calc(MakeChain(0.15), MakeChain(0.78))`
- `assert(calc.payload_mode() == false)`
- 取 `q = 0`, `q̇ = 0`, `q̈ = 0` — 计算 `τ_nominal`
- `SetPayloadMode(true); assert(calc.payload_mode() == true);` — 计算 `τ_payload`
- 断言水平关节（index=1，即 `joint2` 绕 Y）重力矩 `|τ_payload(1)| > |τ_nominal(1)|`
- `SetPayloadMode(false); assert(calc.payload_mode() == false);` — 切回后 τ 与 `τ_nominal` 差 `<1e-12`

自检 pass → stdout 打 `All dual-chain tests passed.\n`、退出码 0；fail → assert abort。

### 模型文件（main 已正确存在）

- `src/arm/arm/model/robotic_arm.urdf` — nominal
- `src/arm/arm/model_2/robotic_arm.urdf` — payload，负载 630 g/350 mm³ 方块放在 link6
- 两份同源模型：相同的 link 名、joint 名、运动学拓扑，仅在 link6 处惯性参数不同。**不做一致性校验**（同源即一致，temporary 分支也如此）。

### 错误处理（与现有一致）

- 文件不可打开 → throw `runtime_error("unable to open payload URDF: " + path)`, 节点退出
- `kdl_parser::treeFromString` 失败 → throw `runtime_error("failed to parse payload URDF: " + path)`
- `getChain(base, tip)` 失败 → throw `runtime_error("failed to build payload KDL chain from " + base + " to " + tip)`
- `SetPayloadMode` 同值 quick-return（不重设，不重复打印）

---

## 风险与已知上限

1. **双份 solver 常驻内存** — 每个 `solver` 很小（只持有 chain 引用 + 内部分配），12 份影响可忽略。
2. **`last_joint_solution_` 作 IK seed** — 切换时被保留；如果跨链切后 IK 初值不在当前链的解空间附近，可能多于平时找到解。当前不主动清空（按段落 1 确认）。
3. **`mass_matrix_` 等缓存共用** — 假设两链关节数相同。同源模型保证；同源模型不兼容时应是 modeling bug 而非运行时 bug。
4. **活动指针是原始指针** — `ArmCalc` 未析构 12 solver 匹配（与 temporary 分支一致，solvers 随 ArmCalc 析构一起释放；active ptr 从不 `delete`）。
5. **切换不受力矩平滑过渡保护** — `SetPayloadMode` 做 abrupt 跳变，力矩在切换点可能有跳变。当前不引入过渡（temporary 也如此）；需要平滑过渡可后续作为一种 `blend_factor ∈ [0,1]` 的扩展。

---

## 下次扩展（不在此次范围）

- 平滑过渡 / 力矩插值避免跳变
- 在线质量估计 / 参数辨识
- 端 payload 惯量估计接口（视觉 + 力矩辨识）
- `last_joint_solution_` 切链时主动清空或提示用户 design IK seed response

---

## 实现指向

这项 port 与 `temporary` 分支等价；实现时应当以 temporary 分支的 diff 作为 `arm_calc/` 源码唯一信源，复制到 main。

改动集：
- `src/arm/arm_calc/include/arm_calc/arm_calc.hpp`
- `src/arm/arm_calc/src/arm_calc/arm_calc.cpp`
- `src/arm/arm_calc/src/arm_ctrl.cpp`
- `src/arm/arm_calc/test/test_dual_chain.cpp`

不改动：
- `CMakeLists.txt`, `package.xml`, `arm/CMakeLists.txt`, `arm/package.xml`
- 任何 launch / yaml / rviz 配置文件
- 任何 .sh 脚本

main 一致性约束：
- 模型 `src/arm/arm/model_2/robotic_arm.urdf` 已经 untracked 存在于 main，不做改动
- nomain arm/model 与 arm_calc 之外的用户代码不感知变化（`ArmCalc` 公开接口签名不变）
