# EEF Streaming 抖动调查记录

更新日期：2026-07-29

## 问题现象

- EEF streaming + teleop 时，末端移动速度不均，横向移动尤其明显，机械臂和固定桌面都会振动。
- `eef_streaming.control_rate_hz` 测过 25、50、100、200 Hz，抖动都存在。
- 放大 EEF 和 joint 限幅阈值后仍然抖动。
- 相似路径通过 ROS 2 Action 执行时明显更平滑。
- 独立 `EefStreamingTest` 使用稳定的 50 Hz 正弦 target，也能直接复现问题。

## 已增加的诊断能力

相关提交：

- `76b3f23 Add EEF streaming diagnostics and reduce IK lock scope`
- `1f399a3 Add controller-side EEF streaming timing diagnostics`

测试端和 controller 端日志记录了：

- target 发布、回环和 controller 接收时间；
- `control_tick` 间隔、执行时间、target age；
- IK 耗时、迭代次数和结果；
- joint command、实际 joint feedback；
- 硬件发送循环间隔、串口发送耗时；
- joint-state 读取耗时和并发数。

## 修复尝试和结果

### 1. 缩小 `_cmd_lock` 范围

修改：

- `solve_eef_ik()` 的状态检查不再等待 `_cmd_lock`。
- 500 Hz硬件循环只在锁内复制target和状态，随后释放锁，再执行串口发送。
- streaming最终写入joint target时仍在锁内重新检查状态。

结果：

- IK状态锁等待最大值从约96 ms降为0。
- `control_tick` 执行时间p95从约34.5 ms降为约3.9 ms。
- target写入到首次硬件发送的p95从约15.1 ms降为约2.6 ms。
- 但机械抖动没有改善，control tick仍有约145 ms的空档。

结论：锁竞争是明显的放大因素，但不是抖动主因。这些修改仍应保留。

### 2. 禁止 joint-state timer 自身重入

修改：

- `JointStatePublisher` 使用独立的 `MutuallyExclusiveCallbackGroup`。
- 同一时刻最多运行一次joint-state读取。

结果：

- joint-state并发读取从27次降为0。
- joint-state消息时间戳倒序从5次降为0。
- actual joint曲线的数据顺序更可信。
- control tick实际频率和最大空档没有实质改善，机械臂仍然抖动。

结论：joint-state自身重入是数据异常和额外负载的来源，但不是抖动主因。

### 3. 降低 joint-state 读取频率

将 `joint_state_rate` 从100 Hz降到1 Hz后，每次测试期间只读取约2次joint state。

结果：

- control tick没有改善，某次测试反而从约37 Hz降到约30 Hz。
- 仍出现约279 ms的control gap。
- 17个超过40 ms的control gap中，只有3个与joint-state读取重叠。

结论：主动joint-state读取会占用串口和executor，但其频率不是当前主因。

注意：launch文件会用默认的 `joint_state_rate=100.0` 覆盖YAML。测试1 Hz时必须显式传入：

```bash
ros2 launch rebotarm_bringup bringup.launch.py \
  joint_state_rate:=1.0 \
  use_rviz:=true
```

### 4. 降低底层硬件发送rate

以下测试均使用相同的50 Hz正弦streaming target。`hardware Hz` 是日志测得的实际发送频率，不是配置值。

| 硬件rate | joint-state rate | controller收到target | control tick | 最大control gap | 实际hardware Hz | 最大发送耗时 |
|---:|---:|---:|---:|---:|---:|---:|
| 500 | 100 | 79 | 93 / 37.4 Hz | 145.9 ms | 164.8 Hz | 106.4 ms |
| 500 | 1 | 67 | 75 / 30.2 Hz | 279.1 ms | 173.0 Hz | 140.2 ms |
| 100 | 1 | 82 | 94 / 37.6 Hz | 123.4 ms | 55.1 Hz | 109.5 ms |
| 50 | 1 | 70 | 92 / 36.8 Hz | 126.1 ms | 30.0 Hz | 54.6 ms |

降低rate减少了典型串口负载和部分长尾，但没有消除control tick空档，rate=50时仍然明显抖动。

底层发送循环不是ROS callback，也不属于callback group。它由SDK专用线程顺序执行，同一时刻不会运行多个发送回调。rate过高只能增加串口流量、GIL竞争和feedback争用，不能单独解释streaming与Action的差异。

### 5. 增加 ROS executor 线程数

当前工作区已将：

```python
MultiThreadedExecutor(num_threads=4)
```

改为：

```python
MultiThreadedExecutor(num_threads=8)
```

尚未取得对应真机日志，效果待验证。由于Python GIL和executor没有实时优先级，预计增加线程只能作为诊断，不是可靠的最终方案。

## 已基本排除的方向

### IK多解或换支

- IK使用上一时刻joint command作为warm-start seed。
- 记录到的IK解沿同一分支持续变化，没有±2π跳变或来回换支。
- 正常情况下通常在约0–9次迭代内收敛，误差约小于 `1e-4`。
- joint1和joint5同步变化主要来自当前构型和identity orientation的运动学耦合，不是多解切换。

因此没有证据表明IK多解是抖动主因。

### EEF或joint限幅

- 在使用大阈值的关键对比测试中，EEF和joint限幅均未触发，但抖动仍然存在。
- 后续严重丢帧时偶尔触发joint限幅，是大target跳变的结果，不是最初原因。

## actual joint 数据来源

`JointStatePublisher` 的数据路径是：

```text
JointStatePublisher.publish()
  -> HardwareManager.get_joint_state()
  -> RebotArm.get_state(request_feedback=True)
  -> 从电机请求实际位置、速度和力矩
```

它是真实电机feedback，不是上一时刻command。

EEF streaming开启时会读取一次真实joint position作为初值；运行过程中，IK seed使用上一时刻发送的joint command，不会把持续发布的joint-state feedback重新喂入IK。因此异常joint-state点不会直接改变IK，但读取过程会占用串口和executor。

## 当前最重要的观察

1. 测试程序能稳定以50 Hz发布125个target，但controller经常只收到约67–84个。
2. `control_tick` 名义50 Hz，实际通常只有约30–41 Hz，并反复出现40–280 ms空档。
3. 丢掉中间target后，controller会一次接收更远的新target，造成较大的joint target跳变；joint1/joint5单次曾达到约0.15 rad。
4. 降低joint-state和硬件发送rate后，control gap仍然存在。
5. rate=50时，IK wall time的p95约45.6 ms、最大约109.7 ms；甚至一次0迭代IK也记录到约37 ms。这说明长耗时主要是线程被暂停或调度延迟，不是IK计算量突然增加。

## 为什么 Action 更平滑

Action和streaming使用同一个底层hardware output loop。主要区别在target生成方式：

```text
Action:
预计算完整joint轨迹
  -> 专用_send_loop线程按时间更新q_target
  -> hardware loop发送

EEF streaming:
ROS target callback
  -> ROS timer触发control_tick
  -> 在线IK
  -> 更新q_target
  -> hardware loop发送
```

Action的轨迹更新不依赖共享ROS executor，也不需要每个周期在线求IK。Action平滑而streaming抖动，说明更可疑的是streaming的在线target更新链路，而不是底层发送循环本身。

## 当前结论和下一步

目前最可能的主因是：

> EEF streaming的 `control_tick` 和在线IK运行在共享ROS executor线程中，周期调度不稳定；串口发送、joint-state读取和Python GIL会放大该问题，但都不能单独解释抖动。

下一步顺序：

1. 用相同测试验证8线程executor；如果没有明显改善，不再继续增加线程。
2. 将EEF streaming control loop从ROS timer移到专用周期线程。ROS target callback只保存latest target，专用线程负责限幅、IK和更新joint target，使其结构更接近Action。
3. 如果专用线程后仍存在硬件发送长阻塞，再考虑让单一I/O线程独占串口，并让joint-state publisher只发布缓存feedback。

## 不纳入对比的日志

`/tmp/rebotarm_controller_eef_streaming_20260728_201000_070842454.jsonl`

该轮测试开始streaming时机械臂尚未到达ready pose且仍有明显速度，起点与其他测试不同，因此不用于判断修改效果。
