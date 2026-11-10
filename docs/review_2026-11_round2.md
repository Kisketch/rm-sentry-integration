# 第二轮审查（2026-11-10）

范围：合并后的 `rm-sentry-integration` master（PR #1）与 `rm-autoaim` main
（含 `c3ad912` 共用核心移植）。复刻方式同第一轮（OpenCV 兼容层编译全部数学核心，
跑单测与仿真；检测器与实拍回归仍需在你的机器上跑）。

## 1. 合并与移植核对

- **哨兵仓 PR #1**：合并后的 master 与审查分支逐文件比对**完全一致**
  （`.bak` 由你单独删除，结果相同）。三个测试程序全过，仿真与审查时数字一致。
- **rm-autoaim `c3ad912`**：`wrapAngle`、Singer 速度衰减、弹道（10 ms RK4 + Hermite +
  割线）、`Config::validate` 移植正确；`autoaim_tests` 全过，仿真与移植前一致。
  你额外加的真空落差热启动和 `sqrt` 替换 `hypot` 是有效的再优化，本轮已**反向同步**到哨兵仓。
- `docs/core_hardening_port.md` 有一处判断需要更正：PoseBuffer 有界外推并非"本仓无对应物"——
  `PoseBuffer` 本身就在 rm-autoaim 里（`MvsSource` 时间戳链路的另一半）。本轮已同步，默认 0，行为不变。

## 2. 新发现（按影响）

### 2.1 相机—姿态时间差被当成"无害"（两仓，影响上车精度）

`ClockAligner` 的注释与 `docs/timestamp_alignment.md` 都写着：恒定传输延迟折进偏移，
"与 `aimer.latency` 一起被端到端实测吸收，无害"。这对**预测时域**成立，对**姿态查询**不成立：

- `MvsSource`：偏移 = 中位数(主机接收时刻 − 设备曝光起始)，所以对齐后的戳 ≈ 主机接收时刻，
  比真实曝光晚 读出 + 传输 + 驱动延迟（通常数 ms 到十几 ms）。
- `sentry_robot`：帧戳 = 接收时刻 − `--exposure-offset-ms`，默认 0，同样晚。
- 姿态在这个偏晚的戳上查，相当于用"D 毫秒之后的云台姿态"去旋转这一帧的观测。
  云台角速度为 ω 时误差是 ω·D：换板 6 rad/s × 10 ms = 3.4°，5 m 处约 30 cm。
  它不是预测时域，`aimer.latency` 补不掉；而且**恰好在云台扫动、换板时最大**。

修复：新增 `DelayCalibrator`（两仓 `delay_calibration.hpp/.cpp`）。对准**静止**装甲板、
扫动云台约 10 s，对每个候选偏移 d 计算板子的世界坐标散布，散布最小的 d 即所求。

- 哨兵仓：`sentry_robot --calib-delay` 采集并在退出时打印应填的 `--exposure-offset-ms`。
- rm-autoaim：`MvsConfig::stamp_offset_us`，结果填这里。
- 合成数据验证（1 cm 观测噪声）：真值 6 / 12 / 20 ms 均估计为 6.00 / 12.00 / 20.00 ms；
  板子世界坐标散布 4.7–14.5 cm → 1.7 cm（剩下的就是噪声本身）。云台不动时拒绝给出结果。

### 2.2 其他

| 位置 | 问题 | 处理 |
|---|---|---|
| rm-autoaim `aimer.cpp` 中心模式 | 目标中心恰在云台原点时 0/0 | 已加保护（哨兵仓第一轮已有） |
| 哨兵仓 `ballistics.cpp` | 与 rm-autoaim 已分叉（缺热启动） | 已同步：`aim` 30 → 15 µs，`solve` 8.6 → 4.9 µs，仿真结果不变 |
| 哨兵仓无 CI | 合并前没有自动构建/测试 | 新增 `.github/workflows/ci.yml`（Ubuntu 24.04：构建 + ctest 三套测试 + 仿真冒烟），沿用 rm-autoaim 的写法 |
| `sentry.cpp` ROI | 有轨迹且窗口内有检测时，最长 30 帧不做全图扫描：新进入视野的更高优先级目标最多晚约 1.2 s（25 fps）才被发现 | 未改（ROI 默认关闭）；建议上车开 ROI 时把间隔改为可配并实测 |
| `sentry.cpp` | `frame = lround(stamp * fps)` 存进 `int`，以开机时钟计、200 fps 时约 124 天溢出 | 未改，建议换成帧计数器 |

## 3. 验证过但**不建议采用**的想法

**用模型导出的角速度做云台前馈**（替代相邻指令差分）。设想是差分速率里带着换板阶跃和检测噪声，
这正是 `docs/gimbal_ff.md` 里前馈在实拍上变差的原因。仿真 3 种子结果：

| gimbal_ff | 差分速率（现状） | 模型速率 |
|---|---|---|
| 0 | 74.6% | 74.6% |
| 0.5 | 75.5% | 74.0% |
| 1.0 | 75.8% | 73.4% |

模型速率更差：换板时差分速率里的负向阶跃恰好帮云台"甩回去"。结论维持原文档：前馈属于电控联调项。
代码已回退，不入库。

## 4. 后续建议（未实施）

1. **把共用核心抽成一个库**：两仓的 `tracker.cpp` 已相差约 350 行、`aimer.cpp` 约 140 行，
   每次修复都要人工移植（本轮的热启动、PoseBuffer 都是这样漏的）。建议把
   `types / pose / pose_buffer / ballistics / tracker / aimer` 放进 rm-autoaim，
   哨兵仓用 git submodule 或 `FetchContent` 引用，只保留 `target / sentry / io`。
2. **重捕后的估计质量**：`warmup_gate.md` 证明"少开火"是负收益；`tracking_frames_` 在
   TEMP_LOST 一帧后就清零，单帧丢检也会重新计数。真正的方向是 LOST 期间保留轨迹、重见时做一致性校验。
3. **控制帧加 CRC**（协议 v2）。

## 5. 合并前请做

1. 两仓 `ctest` / `autoaim_tests` 在 OpenCV 环境下全过（哨兵仓 CI 会自动跑）。
2. 上车：`sentry_robot --serial ... --calib-delay`，对准静止装甲板扫云台约 10 s，
   把打印的值填入 `--exposure-offset-ms`；MVS 相机则填 `MvsConfig::stamp_offset_us`。
3. 填入后复测：慢速扫云台时静止板的世界坐标应基本不动；命中率不应回退。
