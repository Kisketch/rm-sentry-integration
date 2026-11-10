# 姿态时间戳对齐：哨兵仓接入说明（G-02 后半段）

主仓 rm-autoaim 已合入 `PoseBuffer` + `CameraSource`（提交 af1b80e）。本文是
哨兵仓 `rm-sentry-integration` 的接入：串口姿态流入缓冲、视觉侧按曝光时刻
插值查询。先说清三个在代码里核实过的事实，它们决定了接入方式：

1. **下位机协议没有时间戳。** 17 字节姿态帧只有 yaw/pitch/roll（1e-4 度）、
   color、buff。所谓「姿态时间戳」只能是主机接收时刻减去一个估计的
   采样→接收延迟（`--stamp-delay-ms`，默认 1.5 ms ≈ 115200 波特下 17 字节
   的线缆传输时间）。G-02 预算里对齐误差 ≤0.3 ms 在这条链路上是**估计值
   不是保证值**，真实抖动要上车测（见文末）。
2. **旧 `SerialPort::query` 不是「取最新值」，是欧拉角线性插值**，但它
   有三个问题：查询时刻用的是主循环处理时刻而非曝光时刻；姿态过期时
   钳制到最新样本、再旧也照用且不报错；欧拉插值不走真实旋转路径。
3. **本仓的欧拉约定与主仓不同**：`CameraModel::setGimbalAttitude` 用旧
   约定 `R = Rz(yaw)·Ry(−pitch)·Rx(roll)`，pitch 正值表示向下。接入时
   四元数按此约定构造（`quatFromYawPitchRoll(yaw, −pitch, roll)`），
   查询结果再按同约定拆回欧拉角，`SentryVision::process` 的接口不变。

## 改动

| 文件 | 改动 |
| --- | --- |
| `include/autoaim/pose_buffer.hpp`、`src/pose_buffer.cpp` | 与主仓 af1b80e 相同的文件，四元数环形缓冲 + SLERP |
| `include/io/serial_port.hpp`、`src/io/serial_port.cpp` | 接收线程把每帧姿态转四元数入 `PoseBuffer`（附接收时刻戳）；`query` 改为 SLERP 插值，戳未被样本包围或间隔超过 `max_gap` 时**返回 false**；新增计数器 `receivedFrames()` / `failedQueries()`；构造参数新增 `stamp_delay_s`、`max_gap_s`（有默认值，原调用方式可不改） |
| `apps/sentry_robot.cpp` | 帧戳改为曝光时刻估计（接收时刻 − `--exposure-offset-ms`，视频文件模式不变）；串口存在但查询失败时**整帧跳过**（不用零姿态/旧姿态污染跟踪），保持逻辑照常发不击发指令并计数；新增 `--stamp-delay-ms`、`--max-gap-ms` 参数；结束时打印姿态丢失统计 |
| `tests/serial_port_test.cpp` | PTY 端到端测试：字节流入 → 插值姿态出，覆盖解析比例、roll 的 π 偏移、pitch 符号约定、间隔拒绝与过期拒绝 |
| `CMakeLists.txt` | 核心库加 `src/pose_buffer.cpp`；新增 `serial_port_tests` 目标（Unix） |

## 行为变化（有意为之）

- 启动瞬间、串口断流或姿态帧间隔 >50 ms（`--max-gap-ms`）时，对应图像帧
  会被跳过而不是用错姿态硬算。姿态流应 ≥20 Hz 才能保证包围，常规下位机
  100 Hz 以上没有压力；若上车发现 misses 很多，先查姿态流频率和接线，
  不要先放宽 `max_gap` 把问题盖住。
- `attitude misses` 计数就是验收指标之一：稳态运行时应接近 0。

## 上车还要测的（G-02 第 3、4 步，本次未做）

1. **端到端延迟实测**，把结果写回 `aimer.latency` 并跑回归。姿态侧的
   `stamp_delay` 与相机侧的 `exposure_offset` 都是这一步的标定对象：
   现在给的是估计默认值（1.5 ms / 0 ms），不是测得值。
2. 相机换到海康 MVS 后（主仓 `MvsSource`），曝光戳是硬件戳，
   `--exposure-offset-ms` 应设回 0，本仓的 VideoCapture 路径只作过渡。
