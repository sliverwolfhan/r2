# Arm Payload Dual-Chain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port temporary branch's dual-chain KDL model support into `main` branch so the controller can switch the entire KDL solver stack (FK/Jacobian/IK/Dynamics) between nominal (no payload) and payload (630 g / 350 mm³ cube on link6) at runtime via ROS 2 parameter `use_payload`, allowing real-time dynamics compensation for the loaded arm.

**Architecture:** `ArmCalc()` becomes a double-chain host with two pre-built sets of 6 KDL solvers each (12 solver objects) plus 6 active pointers that all public methods route through. `SetPayloadMode(bool)` is an O(1) pointer swap behind a mutex. `arm_ctrl.cpp` loads the payload URDF from a file at construction time and exposes a `use_payload` ROS parameter whose callback flips the active chain.

**Tech Stack:** ROS 2 Humble / `rclcpp`, `orocos_kdl 1.5`, `kdl_parser`, `Eigen 3.4`. Standards: C++17, colcon build.

## Global Constraints

- Source of truth is the `temporary` branch's working code (`git show temporary:<file>`). Port line-equivalent, do not re-author.
- The payload model file `src/arm/arm/model_2/robotic_arm.urdf` already exists untracked on `main`. Do not modify.
- Model topology is unchanged (6 DOF, same link/joint names/order); only link6 inertia parameters differ. Do not add validation that asserts this.
- Do NOT modify CMakeLists, package.xml, launch scripts, rviz configs, or `.sh` files.
- Keep `ArmCalc` public method signatures unchanged so `JointSpaceMove`, `JCartesianSpaceMove`, `VisualServoMove` consumers compile unchanged.
- Tests: self-contained c++17 main, no gtest framework. Exit code 0 on pass; print diagnostic + `All dual-chain tests passed.` to stdout.
- Plan task granularity: each commit is one small, testable commit. Existing commit message style present in repo: `feat(arm_calc): ...`.

---

## Task 1: Port `arm_calc.hpp` to dual-chain

**Files:**
- Modify: `src/arm/arm_calc/include/arm_calc/arm_calc.hpp`
- Reference: `git show temporary:src/arm/arm_calc/include/arm_calc/arm_calc.hpp`

**Interfaces:**
- Consumes: existing `arm_calc/common_types.hpp`, KDL headers (kept)
- Produces: `ArmCalc(const KDL::Chain& nominal_chain, const KDL::Chain& payload_chain)`; public `SetPayloadMode(bool)`; public `bool payload_mode() const`

- [ ] **Step 1: Replace header with temporary's version**

Run:
```bash
git show temporary:src/arm/arm_calc/include/arm_calc/arm_calc.hpp \
  > src/arm/arm_calc/include/arm_calc/arm_calc.hpp
```

Expected: file written verbatim.

- [ ] **Step 2: Verify the diff against main is only the agreed structural changes (visual)**

Run:
```bash
git diff --stat src/arm/arm_calc/include/arm_calc/arm_calc.hpp
```
Expected: ~47 insertions, minimal deletions; diff includes `kdl/chain.hpp` (kept), keeps `mutex` include, changes constructor signature from `Chain&` to two `Chain&` parameters, adds `SetPayloadMode(bool)`, `payload_mode() const`, 12 solver fields, 6 active pointers, replace singular solver fields.

- [ ] **Step 3: Verify consumer includes still work (semantic sanity)**

Run:
```bash
grep -n "ArmCalc(" src/arm/arm_calc/src/arm_action/*.cpp src/arm/arm_calc/src/arm_ctrl.cpp
```
Expected: this will show `std::make_shared<ArmCalc>(arm_chain)` in arm_ctrl.cpp — that's the next task's job, not this one (the consumer file is modified in Task 3). Confirm no other file calls `ArmCalc(...)` — only `arm_ctrl` does.

- [ ] **Step 4: Commit**

```bash
git add src/arm/arm_calc/include/arm_calc/arm_calc.hpp
git commit -m "refactor(arm_calc): ArmCalc accepts dual KDL chains (nominal + payload)

Adds 6 active pointers + 12 pre-built solvers (FK, Jac, Jdot, IK vel, IK pos,
Dynamics x2). Public interface signatures preserved. Per specs at
docs/superpowers/specs/2026-07-05-arm-payload-dual-chain-design.md."
```

---

## Task 2: Port `arm_calc.cpp` (dual-chain implementation)

**Files:**
- Modify: `src/arm/arm_calc/src/arm_calc/arm_calc.cpp`
- Reference: `git show temporary:src/arm/arm_calc/src/arm_calc/arm_calc.cpp`

**Interfaces:**
- Consumes: `arm_calc.hpp:ArmCalc(nominal_chain, payload_chain)` from Task 1
- Produces: accessible `SetPayloadMode`, `payload_mode`, all original public methods route through `*active_*`

- [ ] **Step 1: Replace implementation file with temporary's version**

```bash
git show temporary:src/arm/arm_calc/src/arm_calc/arm_calc.cpp \
  > src/arm/arm_calc/src/arm_calc/arm_calc.cpp
```

Expected: file written verbatim. The diff should show ~100 insertions, replacing constructor + adding SetPayloadMode + prepending every public method with `lock_guard<mutex>(active_mutex_)` and routing solver calls through `*active_*`.

- [ ] **Step 2: Verify diff stat**

```bash
git diff --stat src/arm/arm_calc/src/arm_calc/arm_calc.cpp
```
Expected: roughly `+100 / -36` (matching the earlier audit).

- [ ] **Step 3: Commit**

```bash
git add src/arm/arm_calc/src/arm_calc/arm_calc.cpp
git commit -m "feat(arm_calc): route all solvers through active-pointer; add SetPayloadMode

All 6 KDL solver categories run through *active_* to enable O(1) payload
switching. Mutex+lock_guard per public method. Per specs at
docs/.../2026-07-05-arm-payload-dual-chain-design.md."
```

---

## Task 3: Port `arm_ctrl.cpp` (load payload chain + declare params)**

**Files:**
- Modify: `src/arm/arm_calc/src/arm_ctrl.cpp`
- Reference: `git diff main temporary -- src/arm/arm_calc/src/arm_ctrl.cpp` — specifically choose only the subset that matches the approved design scope:
  1. Add `use_payload` (default false) and `payload_urdf_path` (default `arm/model_2/robotic_arm.urdf`) in `declare_parameters()`
  2. In `load_robot_description_and_build_solver`: open payload URDF file → `kdl_parser::treeFromString` → `getChain(base, tip, payload_chain)` → construct `std::make_shared<ArmCalc>(arm_chain, payload_chain)` (dual-arg)
  3. Handle `use_payload` in `on_parameters_changed` callback → call `arm_calc_->SetPayloadMode(val)`
- Skip the unrelated torque-print line (`RCLCPP_INFO_THROTTLE ... torque=[...]`) in `publish_joint_target` — not in design scope, leave that file as-is for that method.

**Interfaces:**
- Consumes: `ArmCalc(nominal_chain, payload_chain)` from Tasks 1-2
- Produces: new `use_payload` / `payload_urdf_path` ROS 2 parameters

- [ ] **Step 1: Apply scoped Edits (NOT bulk file replacement)**

Run the following three Edit calls against `src/arm/arm_calc/src/arm_ctrl.cpp`:

Edit A — in `declare_parameters`, after the existing `cartesian_target_quaternion` line, add the two new parameter declarations:
```cpp
    this->declare_parameter<bool>("use_payload", false);
    this->declare_parameter<std::string>("payload_urdf_path",
        ament_index_cpp::get_package_share_directory("arm") + "/model_2/robotic_arm.urdf");
```

Edit B — in `load_robot_description_and_build_solver`, replace the existing single-chain construction block:
```cpp
    if (!tree.getChain(base_link_, tip_link_, arm_chain_)) {
        throw std::runtime_error("failed to build KDL chain from " + base_link_ + " to " + tip_link_);
    }

    arm_calc_ = std::make_shared<ArmCalc>(arm_chain_);
```
with:
```cpp
    if (!tree.getChain(base_link_, tip_link_, arm_chain_)) {
        throw std::runtime_error("failed to build KDL chain from " + base_link_ + " to " + tip_link_);
    }

    // Load payload URDF from local file and build the payload KDL chain.
    const std::string payload_urdf_path = this->get_parameter("payload_urdf_path").as_string();
    KDL::Chain payload_chain;
    {
        std::ifstream input(payload_urdf_path);
        if (!input.is_open()) {
            throw std::runtime_error("unable to open payload URDF: " + payload_urdf_path);
        }
        std::ostringstream buffer;
        buffer << input.rdbuf();
        KDL::Tree payload_tree;
        if (!kdl_parser::treeFromString(buffer.str(), payload_tree)) {
            throw std::runtime_error("failed to parse payload URDF into KDL tree: " + payload_urdf_path);
        }
        if (!payload_tree.getChain(base_link_, tip_link_, payload_chain)) {
            throw std::runtime_error("failed to build payload KDL chain from " + base_link_ + " to " + tip_link_);
        }
    }

    arm_calc_ = std::make_shared<ArmCalc>(arm_chain_, payload_chain);
```

Edit C — in `on_parameters_changed`, add a branch before the `"joint_target"` check:
```cpp
        } else if (param.get_name() == "use_payload") {
            if (arm_calc_) {
                arm_calc_->SetPayloadMode(param.as_bool());
            }
        } else if (param.get_name() == "joint_target") {
```

Expected: all three edits apply; pass through existing error handling patterns.

- [ ] **Step 2: Verify stat + diff**

```bash
git diff --stat src/arm/arm_calc/src/arm_ctrl.cpp
```
Expected: ~+30 / -1 lines added (3 small edits).

- [ ] **Step 3: Commit**

```bash
git add src/arm/arm_calc/src/arm_ctrl.cpp
git commit -m "feat(arm_ctrl): load payload URDF + expose use_payload runtime parameter

Per specs at docs/.../2026-07-05-arm-payload-dual-chain-design.md."
```

---

## Task 4: Add self-contained unit test `test_dual_chain.cpp`

**Files:**
- Create: `src/arm/arm_calc/test/test_dual_chain.cpp`
- Reference: `git show temporary:src/arm/arm_calc/test/test_dual_chain.cpp`

**Interfaces:**
- Produces: file that compiles with `arm_calc` headers, exits 0 on pass, aborts on fail. Signature `int main()`.

- [ ] **Step 1: Create test file**

```bash
git show temporary:src/arm/arm_calc/test/test_dual_chain.cpp \
  > src/arm/arm_calc/test/test_dual_chain.cpp
```

Expected: file written verbatim. 96 lines + includes. Has `MakeChain(double link6_mass)` helper; `int main()` with asserts.

- [ ] **Step 2: Commit**

```bash
git add src/arm/arm_calc/test/test_dual_chain.cpp
git commit -m "test(arm_calc): dual-chain gravity torque self-check

Paths nominal(0.15 kg link6) vs payload(0.78 kg link6); q=0; proves
|tau_payload(joint2)| > |tau_nominal(joint2)|; proves round-trip
(SetPayloadMode(true) → SetPayloadMode(false)) restores tau to 1e-12.
Per specs at docs/.../2026-07-05-arm-payload-dual-chain-design.md."
```

---

## Task 5: Build and verify

**Files:** none (build artifacts only)

**Interfaces:**
- Consumes: Tasks 1-4

- [ ] **Step 1: Build arm_calc package**

```bash
cd /path/to/ws  # workspace root containing src/
colcon build --packages-select arm_calc --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Expected: build succeeds, zero warnings/errors in `arm_calc` (`arm_calc` target links orocos_kdl; `arm_calc_test` target not affected).

- [ ] **Step 2: Run self-check unit test**

```bash
./build/arm_calc/test_dual_chain  # binary path when colcon build succeeds
```
Expected output:
```
tau_nominal = <...> <n1> <...> ...
tau_payload = <...> <p1> <...> ...
All dual-chain tests passed.
```
and exit code 0 (use `echo $?` to confirm). Assert `p1 > n1` in magnitude (joint2 horizontal).

- [ ] **Step 3: Verify the package still launches unchanged**

```bash
ros2 launch <your_launch_file>  # any existing arm launch
```
Expected: node starts up cleanly without parameter errors (do NOT yet flip `use_payload` to true; start tests only ensure no regressions on default path).

- [ ] **Step 4: If tests fail, fix in place and recommit**

Do NOT modify Tasks 1-4 implementation files unless the test reveals a real logic bug. If bugs are found:
- Add a new commit `test(arm_calc): fix ...`  
- Re-run step 2 until tests pass.

---

## Task 6: Runtime smoke test of `use_payload` switch

**Files:** none (runtime-only)

**Interfaces:**
- Consumes: Tasks 1-5 (built artifact + running node)

- [ ] **Step 1: Start arm_calc node with default use_payload=false**

```bash
ros2 launch <arm_launch_file>
```

- [ ] **Step 2: Switch to payload at runtime**

```bash
ros2 param set /arm_ctrl use_payload true
```
Expected: node logs `[ArmCalc] KDL chain switched to: PAYLOAD`.

- [ ] **Step 3: Switch back**

```bash
ros2 param set /arm_ctrl use_payload false
```
Expected: node logs `[ArmCalc] KDL chain switched to: NOMINAL`.

- [ ] **Step 4: No crash, no abort, no exception**

If any assertion fires or stack trace appears, debug via ROS log. Otherwise mark pass.

- [ ] **Step 5: No commit for this task** (runtime-only)

---

## Commit History Summary

1. `refactor(arm_calc): ArmCalc accepts dual KDL chains`
2. `feat(arm_calc): route all solvers through active-pointer; add SetPayloadMode`
3. `feat(arm_ctrl): load payload URDF + expose use_payload runtime parameter`
4. `test(arm_calc): dual-chain gravity torque self-check`

Tests pass → mark plan done. Else iterative fix commit `test(arm_calc): fix ...` and re-run Task 5 step 2.
