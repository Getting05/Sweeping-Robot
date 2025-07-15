# 碰撞检测冷却功能更新说明

## 概述
为扫地机器人评估系统增加了15秒碰撞检测冷却功能，防止同一碰撞事件被重复检测和计数。

## 修改的文件

### 1. sweeping_robot_evaluator.py
- **位置**: `/home/getting/Sweeping-Robot/src/auto_nav/scripts/sweeping_robot_evaluator.py`
- **修改内容**:
  - 添加了碰撞检测冷却相关变量：
    ```python
    self.collision_cooldown = 15.0  # 碰撞检测冷却时间(秒)
    self.last_collision_time = 0.0  # 上次碰撞检测时间
    ```
  - 更新了`detect_collision`方法，增加冷却逻辑
  - 添加了启动日志说明冷却时间

### 2. coverage_monitor.py
- **位置**: `/home/getting/Sweeping-Robot/src/auto_nav/scripts/coverage_monitor.py`
- **修改内容**:
  - 添加了相同的碰撞检测冷却变量
  - 更新了`detect_collision`方法
  - 添加了启动日志说明

## 功能特性

### 碰撞检测冷却机制
1. **冷却时间**: 15秒
2. **触发条件**: 当检测到碰撞/卡住事件后
3. **冷却期行为**: 
   - 不进行碰撞检测逻辑
   - 继续更新位置信息
   - 显示冷却剩余时间（测试模式）

### 碰撞检测逻辑
1. **卡住检测**: 机器人在10秒内移动距离小于0.05米
2. **碰撞计数**: 满足卡住条件时增加碰撞计数
3. **冷却启动**: 检测到碰撞后立即开始15秒冷却
4. **日志输出**: 显示碰撞次数和冷却状态

## 代码示例

### 更新后的detect_collision方法
```python
def detect_collision(self, position, current_time):
    """检测碰撞（通过卡住检测），带15秒冷却功能"""
    # 检查是否在冷却期内
    if current_time - self.last_collision_time < self.collision_cooldown:
        # 在冷却期内，不进行碰撞检测，但继续更新位置信息
        self.last_position_for_collision = position
        return
    
    if self.last_position_for_collision is not None:
        dist = math.sqrt((position[0] - self.last_position_for_collision[0])**2 +
                       (position[1] - self.last_position_for_collision[1])**2)
        
        if dist < self.stuck_threshold:
            if self.stuck_start_time is None:
                self.stuck_start_time = current_time
            elif current_time - self.stuck_start_time > self.stuck_time_threshold:
                self.collision_count += 1
                self.last_collision_time = current_time  # 记录碰撞时间，开始冷却
                rospy.logwarn("检测到碰撞/卡住事件，总计: %d (开始15秒冷却)", self.collision_count)
                self.stuck_start_time = None
        else:
            self.stuck_start_time = None
    
    self.last_position_for_collision = position
```

## 测试
创建了测试脚本 `test_collision_cooldown.py` 来验证冷却功能的正确性。

## 效果
- 避免了同一碰撞事件的重复计数
- 提高了碰撞检测的准确性
- 减少了误报的碰撞事件
- 保持了系统性能评估的可靠性

## 配置参数
- `collision_cooldown`: 15.0秒（可根据需要调整）
- `stuck_threshold`: 0.05米（卡住检测距离阈值）
- `stuck_time_threshold`: 10.0秒（卡住检测时间阈值）

## 日志输出示例
```
[INFO] Collision detection cooldown: 15.0 seconds
[WARN] 检测到碰撞/卡住事件，总计: 1 (开始15秒冷却)
```

## 兼容性
- 向后兼容现有的评估指标
- 不影响其他功能模块
- 保持原有的API接口不变
