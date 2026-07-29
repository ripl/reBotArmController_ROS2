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

曾测试将：

```python
MultiThreadedExecutor(num_threads=4)
```

改为：

```python
MultiThreadedExecutor(num_threads=8)
```

后续真机验证显示：4线程改为8线程没有解决抖动。

结论：增加 `MultiThreadedExecutor` 线程数不是有效修复。由于Python GIL和executor没有实时优先级，更多线程可能只增加调度竞争，不能保证50 Hz callback稳定执行。

### 6. 将 EEF control tick 移到专用线程

修改：

- 不再使用ROS timer直接调度 `_control_tick()`。
- EEF streaming enable后启动 `eef_streaming_control` 专用线程。
- 线程使用 `time.monotonic_ns()` 计算下一次tick时间，按 `eef_streaming.control_rate_hz` 周期运行。
- 每次记录 `control_loop_tick`、计划时间、wake lateness、resync等诊断。

结果：

- 专用线程消除了对ROS timer的直接依赖。
- 但在4线程/8线程 `MultiThreadedExecutor` 下，外部topic输入仍然不稳定，机械臂仍抖。
- 因此 `_control_tick()` 本身不再是唯一瓶颈，target输入链路仍有问题。

结论：专用control thread是必要结构改动，但单独不足以修复；必须同时解决controller进程内ROS callback调度不稳。

### 7. 单独测试 hardware send 路径

新增 `HardwareSendRateTest`，绕开ROS topic、IK和EEF streaming，只对SDK/hardware发送路径计时。测试包含两类：

- hold：反复发送当前joint position；
- sine：从ready pose附近开始，对一个或全部关节施加小幅正弦变化。

用户实测结果：

| 请求rate | 实际rate | total send mean | total send max | send>period |
|---:|---:|---:|---:|---:|
| 50 Hz | 49.70 Hz | 1.19 ms | 1.44 ms | 0 |
| 100 Hz | 98.76 Hz | 1.20 ms | 1.62 ms | 0 |
| 200 Hz | 195.53 Hz | 1.20 ms | 5.72 ms | 1 |
| 500 Hz | 481.20 Hz | 1.06 ms | 1.88 ms | 0 |

全部关节正弦运动时结果也稳定：

| 请求rate | 实际rate | total send mean | total send max | send>period |
|---:|---:|---:|---:|---:|
| 50 Hz | 49.69 Hz | 1.20 ms | 1.53 ms | 0 |
| 100 Hz | 98.77 Hz | 1.19 ms | 5.83 ms | 0 |
| 200 Hz | 195.62 Hz | 1.20 ms | 1.50 ms | 0 |
| 500 Hz | 484.25 Hz | 1.05 ms | 1.35 ms | 0 |

结论：

- 串口/hardware send本身能稳定按50 Hz、100 Hz、200 Hz甚至接近500 Hz发送。
- 发送“相同joint position”不是导致抖动的原因；发送变化的joint position也稳定。
- USB 2.0线速不是当前瓶颈。实际发送耗时约1 ms量级，远低于50 Hz的20 ms周期。

### 8. 关闭诊断细节和 target TF

新增参数：

- `eef_streaming.publish_target_tf`
- `eef_streaming.diagnostics_detail`
- `eef_streaming.diagnostics_enabled`

结果：

- 关闭详细诊断和TF后，肉眼观察抖动仍存在。
- 在callback-only测试中，关闭TF有一定改善，但仍远低于50 Hz：

| 场景 | controller收到target | 实际callback rate | 最大callback间隔 |
|---|---:|---:|---:|
| callback-only，TF开 | 55 | 18.48 Hz | 241.93 ms |
| callback-only，TF关 | 67 | 22.74 Hz | 164.78 ms |

结论：TF publish会放大问题，但不是根因。诊断记录也不是根因。

### 9. controller内部生成target

新增 internal target 诊断模式：

- `eef_streaming.internal_target_enabled:=true`
- controller不订阅 `/eef_target_pose`；
- `_control_tick()` 内部直接生成 `x=0.3, y=0.0->0.4, z=0.3` 的sine target。

结果：

- 用户真机观察：运动非常丝滑，不抖。

结论：

- IK、限幅、q_target更新、hardware output loop在该路径下可以平滑工作。
- 抖动不是EEF轨迹本身、IK本身或底层发送本身导致。
- 问题集中到“外部ROS topic target进入controller进程”这条链路。

### 10. 外部publisher + controller只接收target

新增 `EefTargetPublisherTest`：

- 单独测试节点发布 `/rebotarm/eef_target_pose`；
- 不move ready；
- 不调用 `/eef_streaming/enable`；
- 不跑IK；
- 不发硬件控制。

新增 controller callback-only 诊断：

- `eef_streaming.target_callback_diagnostics_enabled:=true`
- controller启动时只记录 `_target_callback()` 收到target的时间；
- 不启用EEF streaming；
- 可选发布TF。

publisher实测：

```text
published targets: 151
actual publish rate: 50.00 Hz
publish interval mean: 20.001 ms
max: 20.155 ms
```

controller callback-only结果：

| 场景 | target_received | 实际callback rate | mean间隔 | p95 | max |
|---|---:|---:|---:|---:|---:|
| TF开 | 55 | 18.48 Hz | 54.10 ms | 208.63 ms | 241.93 ms |
| TF关 | 67 | 22.74 Hz | 43.97 ms | 128.99 ms | 164.78 ms |

结论：

- 外部publisher稳定50 Hz。
- 同样的topic进入 `reBotArmController` 后，subscription callback明显稀疏。
- QoS为 `depth=1 + BEST_EFFORT`，callback调度不上时旧target会被覆盖/丢弃，导致controller看到稀疏target点。

### 11. 纯ROS subscriber隔离

新增 `EefTargetSubscriberTest`：

- 不启动 `reBotArmController`；
- 不连接hardware；
- 只订阅 `/rebotarm/eef_target_pose`；
- 记录callback间隔；
- 可选发布TF。

用户实测：

```text
received targets: 22
actual receive rate: 49.90 Hz
receive interval mean: 20.040 ms
p95: 20.505 ms
max: 20.762 ms
large gaps >25ms: 0
```

虽然只收到22个，是因为subscriber和publisher运行时间只有短暂重叠；重叠期间接收频率稳定50 Hz。

结论：

- ROS topic跨进程publish/subscribe本身没有问题。
- QoS和publisher不是根因。
- 问题在 `reBotArmController` 进程内部结构或executor调度。

### 12. 关闭 joint-state timer

新增参数：

- `joint_state_enabled:=false`

该参数只禁止joint-state timer；`/arm_status` publisher仍保留。

测试结果：

```text
joint_state_enabled: false
publish_target_tf: false

target_received: 58
actual rate: 21.68 Hz
mean interval: 46.13 ms
p95: 184.40 ms
max: 347.55 ms
```

结论：joint-state timer/read hardware路径不是主要原因。

### 13. 关闭 SDK hardware output loop

新增参数：

- `hardware_output_loop_enabled:=false`

该参数让controller连接hardware，但不启动SDK的 `start_control_loop(self._endpos_loop_cb)`。测试时也关闭joint-state timer和TF。

测试结果：

```text
target_received: 105
actual rate: 35.29 Hz
mean interval: 28.34 ms
p95: 66.47 ms
max: 100.00 ms
```

对比关闭前约21.68 Hz，说明SDK hardware output loop会放大调度问题，但关闭后仍未恢复50 Hz。

结论：hardware output loop是影响源之一，但不是唯一根因。

### 14. 完全跳过 hardware connect

新增参数：

- `hardware_connect_enabled:=false`

该模式下controller使用 `_DisconnectedHardware` mock：

- 不实例化真实 `HardwareManager`；
- 不导入/连接SDK；
- 不启动硬件后台线程；
- controller节点、services/actions、EEF subscriber仍创建。

测试结果：

```text
target_received: 78
actual rate: 26.19 Hz
mean interval: 38.18 ms
p95: 131.81 ms
max: 169.42 ms
```

结论：

- 真实SDK/hardware connect不是单独根因。
- 即使没有真实hardware，`reBotArmController` 使用当前executor结构时，target callback仍不稳定。

### 15. 单线程 executor 隔离

新增参数：

- `controller_executor_threads`

实现：

- 默认可配置executor线程数；
- `controller_executor_threads <= 1` 时使用 `SingleThreadedExecutor()`；
- 大于1时使用 `MultiThreadedExecutor(num_threads=...)`。

mock hardware + 单线程executor测试：

```text
controller_executor_threads: 1
hardware_connect_enabled: false
joint_state_enabled: false
hardware_output_loop_enabled: false

target_received: 149
actual rate: 49.98 Hz
mean interval: 20.007 ms
p95: 20.529 ms
max: 20.914 ms
>25ms: 0
```

真实hardware + 单线程executor callback-only测试：

```text
controller_executor_threads: 1
hardware_connect_enabled: true
hardware_output_loop_enabled: true
joint_state_enabled: false
publish_target_tf: false

target_received: 149
actual rate: 49.99 Hz
mean interval: 20.005 ms
p95: 20.193 ms
max: 20.966 ms
>25ms: 0
```

完整EEF streaming + 单线程executor测试：

用户真机观察：运动丝滑，无明显抖动。

日志统计：

```text
target_received: 49.95 Hz, max 24.27 ms, >25ms 0
control_tick:     50.00 Hz, max 20.26 ms, >25ms 0
hardware_output:  49.54 Hz, max 23.28 ms, >25ms 0
```

结论：

- `MultiThreadedExecutor(num_threads=4/8)` 是当前确认的主要调度不稳来源。
- 改为单线程executor后，target callback、control tick和hardware output都稳定在约50 Hz。
- EEF control loop已经移到专用线程，所以controller使用单线程executor不会降低control tick稳定性。

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

## 修复前最重要的观察

1. 测试程序能稳定以50 Hz发布125个target，但controller经常只收到约67–84个。
2. `control_tick` 名义50 Hz，实际通常只有约30–41 Hz，并反复出现40–280 ms空档。
3. 丢掉中间target后，controller会一次接收更远的新target，造成较大的joint target跳变；joint1/joint5单次曾达到约0.15 rad。
4. 降低joint-state和硬件发送rate后，control gap仍然存在。
5. rate=50时，IK wall time的p95约45.6 ms、最大约109.7 ms；甚至一次0迭代IK也记录到约37 ms。这说明长耗时主要是线程被暂停或调度延迟，不是IK计算量突然增加。
6. 后续确认：这些现象的直接原因不是IK计算本身变慢，而是Python线程/callback调度出现大间隔。

## 为什么 Action 更平滑

Action和streaming使用同一个底层hardware output loop。主要区别在target生成方式：

```text
Action:
预计算完整joint轨迹
  -> 专用_send_loop线程按时间更新q_target
  -> hardware loop发送

EEF streaming:
ROS target callback
  -> control loop周期触发control_tick
  -> 在线IK
  -> 更新q_target
  -> hardware loop发送
```

Action的轨迹更新不依赖ROS topic输入链路，也不需要每个周期在线求IK。Action平滑而streaming抖动，说明更可疑的是streaming的在线target更新链路，而不是底层发送循环本身。

## 当前结论

已经确认的主因是：

> `reBotArmController` 使用 `MultiThreadedExecutor(num_threads=4/8)` 时，Python ROS subscription callback调度不稳定，导致 `/eef_target_pose` 在controller进程内只以约18–35 Hz被处理，并出现100–300 ms级别的大间隔。由于QoS为 `depth=1 + BEST_EFFORT`，中间target会被覆盖/丢弃，EEF streaming看到稀疏target点，进而造成 `q_target` 间歇性跳变和机械抖动。

最终有效修复：

1. 将EEF streaming control loop从ROS timer移到专用周期线程，避免control tick依赖ROS executor timer。
2. 将controller默认executor改为单线程：

```python
SingleThreadedExecutor()
```

3. 保留 `controller_executor_threads` 参数用于诊断和回退；当前默认值为1。

为什么单线程更稳定：

- Python有GIL，同一进程内多个Python线程并不能让多个callback真正并行执行Python代码。
- `MultiThreadedExecutor` 的多个worker会增加抢GIL、等待条件变量和callback调度顺序的不确定性。
- EEF streaming需要的是稳定20 ms节拍，而不是更多callback worker。
- 单线程executor按顺序处理ROS callback，反而避免了多worker调度抖动。
- EEF control loop已是专用线程，所以单线程executor不会降低control tick稳定性。

当前推荐运行方式：

```bash
ros2 launch rebotarm_bringup bringup.launch.py \
  joint_state_rate:=1.0 \
  eef_streaming_publish_target_tf:=false
```

如需回退到旧executor行为，可显式传入：

```bash
controller_executor_threads:=4
```

## 不纳入对比的日志

`/tmp/rebotarm_controller_eef_streaming_20260728_201000_070842454.jsonl`

该轮测试开始streaming时机械臂尚未到达ready pose且仍有明显速度，起点与其他测试不同，因此不用于判断修改效果。
