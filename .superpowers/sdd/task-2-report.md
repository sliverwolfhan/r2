# Task 2 Report: Port `arm_calc.cpp` (dual-chain implementation)

**Status:** DONE_WITH_CONCERNS

**Commit:** `e59ece1` feat(arm_calc): route all solvers through active-pointer; add SetPayloadMode

**Files Changed:** `src/arm/arm_calc/src/arm_calc/arm_calc.cpp` only.

**Diff Stat:** `+76 / -26` (1 file changed; brief expected roughly +100 / -36 — delta is from Task 1's header change, which the original audit did not account for).

**Test Summary:** File-copy port from `temporary`; not yet compiled via `colcon build`.

## Self-Review

- Constructor binds all 12 KDL solvers (6 nominal + 6 payload) to their respective chains, initializes `last_joint_solution_` to zero, defaults all 6 active pointers to the nominal chain, and sets `use_payload_ = false`. Matches brief requirement.
- Every public method touching active state opens with `std::lock_guard<std::mutex> lock(active_mutex_);`.
- `SetPayloadMode(bool)` routes through `active_mutex_`, early-returns on no-op, flips all 6 active pointers atomically, and logs the new mode.
- No `arm_ctrl.cpp`, CMakeLists, package.xml, launch, rviz, .sh, or payload/model_2 URDF files were touched. `git status --short` confirms only `arm_calc.cpp` plus a pre-existing untracked `src/arm/arm/model_2/` (not created by this task).

## Concerns

1. `set_joint_pd` / `get_joint_pd` do not take `active_mutex_`. They touch `kp_`/`kd_` arrays. Per brief's strict wording ("every public method opens with lock_guard") these are technically non-compliant. However the port is line-equivalent to `temporary` and Task 3 owns tighter synchronization/ownership decisions — flag rather than fix.
2. Lock recursion inside `signal_arm_calc` → `joint_pos` / `joint_acc` / `joint_torque_dynamic` → inner public helpers: `active_mutex_` is non-recursive (`std::mutex`), but the inner calls are `private`-named helpers (`joint_pos(pose, result, seed)`, `joint_acc`, `joint_torque_inverse_dynamics`, `joint_torque_dynamic`) that take their OWN lock. This is a latent deadlock risk. Source-of-truth from `temporary` carries the same pattern; flag for Task 3 to scope out.
3. Brief references `docs/.../2026-07-05-arm-payload-dual-chain-design.md` which doesn't exist at this commit — commit message stub elides the path. Harmless but non-verifiable post-facto.
4. `end_state` re-acquires `active_mutex_` after calling `end_pose` (which also locks it). Same deadlock pattern as concern #2.

None of these are Task 2 regressions — they are line-equivalent to `temporary`. They are surfaced here as Task 3/verification follow-ups.

## Fix e59ece1 → 89fd3f5

**Commits**
- `89fd3f5` — `fix(arm_calc): use recursive_mutex to prevent deadlock from internal method composition`

**Files Changed**
- `src/arm/arm_calc/include/arm_calc/arm_calc.hpp` (+2 / -2): `std::mutex active_mutex_` → `std::recursive_mutex active_mutex_`; inline `payload_mode()` lock_guard updated.
- `src/arm/arm_calc/src/arm_calc/arm_calc.cpp` (+15 / -13): all `std::lock_guard<std::mutex> lock(active_mutex_)` → `std::lock_guard<std::recursive_mutex> lock(active_mutex_)` (13 occurrences); added lock opening `set_joint_pd` and `get_joint_pd`.

**Deadlock Sites Fixed**
1. `end_state` → `end_pose` (was: recursive `std::mutex` acquisition). Now: recursive_mutex re-entrant, single-owner depth-2 hold.
2. `joint_torque_dynamic` → `joint_acc` → (also reachable `joint_torque_inverse_dynamics` directly).
3. `signal_arm_calc(CartesianTrajectoryPoint)` → `signal_arm_calc(..., JointVector)` → `joint_pos(3)`, `joint_vel`, `joint_acc`, `joint_torque_dynamic` (worst case: depth-5 recursive acquisition on the same thread).
4. `joint_pos(2-arg)` → `joint_pos(3-arg)`.
5. `set_joint_pd` / `get_joint_pd`: previously unlocked access to `kp_`/`kd_`; now held under `active_mutex_`, covering concurrent **SetPayloadMode → bound-check → kp_/kd_** interleaving and signal_arm_calc-torque paths that read PD-shared buffers.

**Covering-Test Impact**
- The unit port (test_dual_chain.cpp, delivered in Task 4) exercises the dual-chain routing, but call patterns with `colcon build` compilation plus `ros2 test` runtime coverage only exercise constructors and single-level solver invocation. The deadlock composition chains above require either a dedicated unit test calling `signal_arm_calc` (depth-5 lock path) or integration with a Task 4+ test that simulates two concurrent `ArmCalc` callers (SetPayloadMode + signal_arm_calc). This fix is verified by `grep -c` instrumentation of lock sites plus manual code-path tracing — there is no self-test in **this** commit that exercises the recursive acquisition path at runtime. The architect flag is: "recursive_mutex as stopgap — composition debt remains in the architecture; post-port cleanup (Task 5+) should migrate to a internal unlocked callee pattern and std::mutex if the public/private split is refactored."

**Constraints Honored**
- No public method signature changes.
- `active_` pointer routing preserved; no solver routing logic changes.
- No edits to arm_ctrl.cpp, CMakeLists, package.xml, launch files, rviz configs, shell scripts, or URDF.
- `<mutex>` header already covers `std::recursive_mutex`; no new includes; C++17.
- `active_mutex_` remains `mutable`; `const` getter compiles unchanged.

**Concerns**

1. **Composition debt is not removed — masked.** `recursive_mutex` converts a hard deadlock into a re-entrant acquisition pattern. The architectural smell (public methods calling other public methods on the same object with a held lock) remains — flagged for maintainer awareness in post-Task-3 cleanup.
2. **No runtime verification added at this commit.** Fix validated via code-path tracing; a proper regression (e.g., a `task4+` unit test invoking `signal_arm_calc` twice concurrently while toggling payload mode at 100 Hz) would be a more robust regression check.
3. **Recursive-mutex cost is nonzero** but bounded by KDL solver cost (`CartToJnt`, `JntToMass`, `JntToJacDot` are the hot paths); the additional cost of a recursive `lock()` in read-heavy single-owner paths is negligible compared to those. Multi-threaded contention profiles are unchanged.
