# ClimbStair Node Quick Start Guide

## Overview

ClimbStair is a behavior tree node for controlling robot stair climbing functionality. It communicates with the climber controller via ROS2 topics.

## How It Works

1. **Send Command**: On start, publishes climb height to `/AT_R2/climb_stair`
2. **Wait for Feedback**: Subscribes to `/AT_R2/climber_status` for status updates
3. **State Transitions**:
   - Status = 1 (CLIMBING): Node stays RUNNING
   - Status = 2 (SUCCESS): Node returns SUCCESS
   - Status = 3 (FAILURE): Node returns FAILURE

## Quick Start

### 1. Build the Package

```bash
cd ~/pb25_ws
colcon build --packages-select nav2_bt_publish_goal
source install/setup.bash
```

### 2. Test with Simulator

Terminal 1 - Start test simulator:
```bash
cd ~/pb25_ws
source install/setup.bash
python3 test_climb_stair_node.py success
```

Terminal 2 - Run behavior tree:
```bash
cd ~/pb25_ws
source install/setup.bash
ros2 run nav2_bt_publish_goal simple_bt_runner example_climb_stair.xml
```

### 3. Monitor Topics

```bash
# Watch climb commands
ros2 topic echo /AT_R2/climb_stair

# Watch status updates
ros2 topic echo /AT_R2/climber_status
```

## Common Use Cases

### Case 1: Simple Climb
```xml
<ClimbStair height="0.3" node="{node}"/>
```

### Case 2: Climb with Retry
```xml
<Retry num_attempts="3">
  <ClimbStair height="0.3" node="{node}"/>
</Retry>
```

### Case 3: Navigate then Climb
```xml
<Sequence>
  <PublishGoal x="1.0" y="2.0" yaw="0.0" node="{node}"/>
  <ClimbStair height="0.3" node="{node}"/>
</Sequence>
```

## Troubleshooting

### Problem: Node returns FAILURE immediately

**Cause**: Invalid height parameter

**Solution**: Ensure height > 0

### Problem: Node stays RUNNING forever

**Cause**: No status messages received

**Solution**: 
1. Check if climber controller is running
2. Verify topic names match
3. Check topic connection: `ros2 topic info /AT_R2/climber_status`

### Problem: Climb command not received

**Cause**: Topic not connected

**Solution**:
1. Check publisher: `ros2 topic info /AT_R2/climb_stair`
2. Verify node is initialized correctly
3. Check ROS2 node is passed to blackboard

## FAQ

**Q: Can I change the topic names?**

A: Currently topic names are hardcoded. To change them, modify the source code and recompile.

**Q: What height values are valid?**

A: Any positive number (> 0). Typical values: 0.2 - 0.5 meters.

**Q: Can I use this with Nav2?**

A: Yes! The node integrates seamlessly with Nav2 behavior trees.

**Q: How long does climbing take?**

A: Depends on your climber controller. The test simulator uses 2 seconds per meter.
