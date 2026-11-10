# 目标身份唯一化与协议 CRC 扩展（2026-11）

与主仓 `rm-autoaim` 同步的两项正确性改动。

## 目标身份（DeviceId）

此前 `TargetManager` 的条目、两个跟踪器的关联都只按装甲数字比对：
同号但不同颜色/板型的板（两队都有 "3"，英雄是大板 "1"）会被关联进
同一条轨迹。现在：

- `autoaim::deviceId(det)`（`types.hpp`）= 颜色 + 数字 + 板型，
  `ArmorDetection` 新增 `color` 字段，检测器在配对时填入；
- `RobotTracker` / `PointTracker` 的 `id_` 与关联条件改按 deviceId；
  `TargetState.label` 仍是纯数字（显示用），`id` 是 deviceId；
- `TargetManager::Entry` 新增 `key`（deviceId，前哨站为 "outpost"）
  做全部关联/复用/同种子分组判断，`label` 只作显示；
- 评测判读与绘制过滤同步改按 deviceId。前哨站跟踪器是单例身份，
  未改。

## 协议 CRC-8（可选扩展，默认关）

17 字节姿态帧的字节 2 此前未使用。扩展约定：发送方在字节 2 放
CRC-8/SMBUS（poly 0x07，覆盖字节 3..16 的载荷）。

- `sentry::packAttitude(frame, with_crc)` 是格式参考实现（供下位机
  与测试用），`parseAttitudeCrc` 是严格解析；旧 `parseAttitude`
  不看字节 2，行为不变；
- `io::SerialPort` 新增 `require_crc`（默认 false）与 `crcFailures()`
  计数；`sentry_robot --require-crc` 打开；
- **启用前提是下位机固件先填字节 2**，否则所有帧都会被拒收——所以
  默认关闭，联调时再开；
- 21 字节控制帧没有空闲字节，加 CRC 必须改帧长，留作协议 v2 决策，
  本次未动。
