# 哨兵视觉整合：旧仓 → 新核的替代设计

本文回答三件事：

1. 旧哨兵代码由哪些部分组成，各自强在哪、弱在哪；
2. 新 `rm-autoaim` 核能替换哪一层，哪些层必须保留；
3. 整合后的代码怎么跑、怎么上车、还缺什么。

对应代码：`/home/lgnatius/Desktop/07_哨兵视觉_整合`。旧代码：
`/home/lgnatius/Desktop/06_参考项目_auto-aim-master`。算法源：
`/home/lgnatius/autoaim-check`（`203dd4c`）。

---

## 1. 旧哨兵代码的分层

| 层 | 文件 | 作用 | 评价 |
|---|---|---|---|
| 检测 | `Armor/src/ArmorDetector.cpp` | 红灯通道阈值 180 → `minAreaRect` → 灯条配对 → `warp 28×28` → `mlp.onnx` 分类 | 能跑，但阈值是单通道绝对阈值；角点用 `minAreaRect` 端点；`LightBlob::top/bottom` 按 y 排序，横置灯条会出错 |
| 位姿 | `Armor/src/ArmorSolver.cpp` | `solvePnP(IPPE)` + `sys_yaw±70°` 步长 1° 重投影搜 yaw | 没有固定 pitch 先验，yaw 搜索以云台 yaw 为中心，不是视线；140 次重投影 |
| 状态估计 | `Armor/src/ArmorTarget.cpp`、`ArmorEKF.cpp` | 10 维 `[xc,vx,yc,vy,z1,z2,r1,r2,yaw,w]`，单/双/前哨站三种策略，熵权匹配，NIS | 无 Singer 阻尼、无各向异性 R、无 dz；匹配逐帧熵权归一化，目标机动时协方差与匹配都不稳 |
| 预测 | `Armor/src/ArmorPredictor.cpp` | 4 板/3 板圆周运动学外推 | 只是把 EKF 状态转成板位置，没有独立预测误差评估 |
| 规划 | `Armor/src/ArmorPlanner.cpp` | 最近板 + ±0.5 s 路径 + 极值点 + 五次多项式 + `max_yaw_acc` | 思想好，但依赖上游 EKF；弹道是 22 m/s 无阻力；`predict_time` 手工 0.075 s |
| 主流程 | `Backbone/src/AutoArmor.cpp`、`main.cpp` | 单线程 `Query→Aim→Send`；`Found/HoldFound 0.5s/Lost`；7 m 门限；ID 锁定 | 没有多目标管理；`aim_ptr` 只有一个；`YamlPath` 写死 `/home/perseverance/...` |
| 驱动 | `io/src/hikrobot.cpp`、`SerialPort.cpp` | 海康 MVS + 165 fps + libusb 自愈；`/dev/ttyACM0 115200`，17 B 收 / 21 B 发，时间戳插值 | 上车必须保留；但视觉核与它们强耦合 |
| 能量机关 | `Buff/` | 圆/扇叶检测 + `mlp_bh4.onnx` + 大小符 EKF + 换扇叶/冷却 | 完整，但独立于装甲链路 |
| 数学基座 | `tools/` | 无阻力弹道、通用 EKF、坐标变换、队列、日志 | 通用 EKF 没被整车 EKF 使用；弹道模型偏旧 |

一句话：**旧仓是“能上车的系统”，强在驱动、串口、主流程、Buff；弱在检测细节、位姿、整车 EKF、弹道与多目标。**

---

## 2. 新核的能力与缺口

新核 `rm-autoaim` 的分层：

| 层 | 文件 | 关键点 |
|---|---|---|
| 检测 | `detector.cpp` | `R−B` 通道差阈值 70；开/闭运算；灯条长宽比/填充率/倾角；配对角度/长度/中心差/间隙比；`reject_contained_lights`；PCA 对称轴角点精修；`mlp.onnx` |
| 位姿 | `pose.cpp` | `solvePnPGeneric(IPPE)` 双解 + pitch 先验选解；固定 pitch=15° 的 1D yaw 搜索（1° 粗搜 + 黄金分割）；`yaw_gain=1.3` 修正辉光致宽 |
| 跟踪 | `tracker.cpp` | 11 维名义 / 13 维代码 `[xc,vx,yc,vy,zc,vz,yaw,w,r1,r2,dz,ax,ay]`；Singer 阻尼；各向异性 R；马氏门限；多板序贯融合；Joseph 更新；NIS 自适应 Q；`LOST/DETECTING/TRACKING/TEMP_LOST` |
| 弹道 | `ballistics.cpp` | 二次阻力 `a=-k|v|v-g`，RK4 1 ms；不动点迭代求 pitch |
| 瞄准 | `aimer.cpp` | `latency+t_fly` 4 轮迭代；选板 facing；`side_angle` 提前换板；3° 滞回；`center_mode` 反小陀螺；`sigma_lat` 开火门 |
| 评测 | `evaluation.cpp`、`autoaim_sim` | 真实视频虚拟打靶 + 预测误差 + 8 场景仿真 + 回归门禁 |

新核**没有**：串口/MCU、相机驱动、ROS 2、Buff、前哨站、多目标、云台速度前馈、相机外参、IMU 时间同步、YAML 之外的部署配置。

---

## 3. 替代边界：换什么、留什么

```
┌──────────────────────────── 旧哨兵 ────────────────────────────┐
│ HikRobot(MVS) → AutoArmor → ArmorDetector → Slover → Aim/EKF   │
│                              → Planner → Transmit → SerialPort  │
│ Buff/ 独立链路                                                   │
└─────────────────────────────────────────────────────────────────┘
                              ↓ 只替换视觉算法层
┌──────────────────────────── 整合后 ────────────────────────────┐
│ HikRobot(MVS) / VideoCapture                                    │
│   → SentryVision：Detector → PoseSolver → TargetManager         │
│        → RobotTracker / OutpostTracker → RobotAimer             │
│   → SerialCommand → packControl → SerialPort                    │
│ Buff/ 保留旧链路，按 armor_or_buff 路由（未并入）                │
└─────────────────────────────────────────────────────────────────┘
```

| 旧层 | 决策 | 原因 |
|---|---|---|
| `ArmorDetector` | **替换**为新 `Detector` | PCA 角点、包含灯条剔除、`R−B` 对过曝更稳；实测同片多检出 0.3–0.5 pp 帧、1.5–2.9% 装甲板 |
| `Slover` | **替换**为 `PoseSolver` | IPPE 双解 + pitch 先验 + 固定 pitch yaw 搜索；计算量从 ~140 次重投影降到 170 次 1° 搜索 + 20 次黄金分割（同量级但更稳），并支持相机外参 |
| `Aim/EKF` | **替换**为 `RobotTracker` | Singer 阻尼、各向异性 R、马氏关联、11 维双半径+dz；旧版无 dz、无 Singer |
| `ArmorPredictor` | **替换**为 `TargetState::armorAt` | 统一给标准 4 板/前哨站 3 板使用 |
| `Planner` | **部分保留思想，默认不接** | 新 `Aimer` 已含 `side_angle`、3° 滞回、`center_mode`、`latency+t_fly` 迭代；旧五次多项式作为可选平滑层，未默认启用，避免和 `center_mode` 冲突 |
| `AutoArmor` | **替换**为 `SentryVision` + `TargetManager` | 多目标、按优先级选目标、目标切换滞回；不再只有一个 `aim_ptr` |
| `SerialPort` | **保留协议，重写传输层** | 17 B/21 B 字节兼容；新 `SerialPort` 有重连、插值、`packControl` |
| `HikRobot` | **保留，未复制** | 依赖 MVS SDK；上车时把 `Query(Mat&, time_point&)` 的结果喂给 `SentryVision::process` 即可 |
| `Buff` | **保留旧链路** | 新核没有能量机关；按主控 `armor_or_buff` 路由，后续再统一 |
| `main.cpp` 状态机 | **保留思路** | 新 `sentry_robot` 有 `HoldFound 0.5s`，但不再持有旧 `AutoArmor` |

### 3.1 新增的哨兵专用能力

1. **相机外参 + 云台姿态**
   `CameraModel` 增加 `R_cam2gimbal`、`t_cam2gimbal`、`setGimbalAttitude(yaw,pitch,roll)`。
   世界系 = 云台系（x 前、y 左、z 上），投影/PNP 都走完整变换：
   `p_world = R_wg * (R_cg * p_cam + t_cg)`。
   这解决了旧核“相机在云台原点、只含 pitch”的假设，能直接吃串口/IMU 的 yaw/pitch/roll。

2. **前哨站 3 板模型**
   `OutpostTracker`：10 维 `[xc,vx,yc,vy,z1,z2,z3,r,yaw,w]`，3 板按 `yaw+2πk/3` 分布，`z` 分别估计；`OutpostTracker::targetState` 给统一 `TargetState`。
   位姿层对 `number=="outpost"` 强制使用大装甲几何与独立俯仰角（`outpost_armor_pitch_deg`）。

3. **多目标管理**
   `TargetManager` 每个目标一个 tracker；观测按预测板位置 + 标签关联；目标切换有 0.5 m 滞回；丢失 1 s 后删除。旧版只有一个 `Aim`，换目标要重新初始化。

4. **统一 `TargetState`**
   瞄准器不再依赖具体 tracker。`RobotAimer::aim(const TargetState&, now)` 对 4 板/3 板/前哨站走同一条选板、弹道、开火逻辑。

5. **串口协议与传输**
   `packControl`/`parseAttitude` 字节兼容旧 21 B/17 B；`io::SerialPort` 有 termios、poll、断线重连、姿态插值。测试覆盖打包/解包。

6. **开火前馈**
   `sentry_robot` 由连续两帧 `AimCommand` 差分出 `yaw_vel`，填入旧协议的 `ui_x` 槽，电控可做速度前馈（旧 `Planner` 也发 `yaw_vel`）。

---

## 4. 同片实测

### 4.1 检测器（只看检测，不含位姿/跟踪）

| 视频 | 旧哨兵检测 | 新检测器 | 变化 |
|---|---|---|---|
| 3.avi (2209 帧) | 2125 帧有目标 (96.2%)，2317 块，4.67 ms/帧 | 2132 帧 (96.5%)，2380 块，6.27 ms/帧 | +7 帧，+63 块，+1.60 ms |
| 4.avi (3416 帧) | 3329 帧 (97.4%)，3827 块，4.72 ms/帧 | 3343 帧 (97.9%)，3885 块，6.34 ms/帧 | +14 帧，+58 块，+1.62 ms |

结论：新检测器在同一批视频上**召回略高**，代价是 PCA 精修 + 分类约 1.6 ms/帧；在 IT13 的 i9-13900HK 上仍远低于 25 fps 预算（6.3 ms ≈ 158 fps 检测吞吐）。

### 4.2 整条链路（虚拟打靶，`autoaim_video`）

| 策略 | 3.avi 命中率 | 3.avi hits/s | 4.avi 命中率 | 4.avi hits/s |
|---|---|---|---|---|
| direct（无预测） | 35.5% | 7.4 | 56.7% | 11.8 |
| point KF（单板匀速） | 29.6% | 5.5 | 43.1% | 3.5 |
| robot EKF（新核） | 67.5% | 10.6 | 68.4% | 10.0 |
| **sentry 整合（多目标+前哨站）** | **68.0%** | **10.3** | **68.5%** | **9.6** |
| oracle（上界） | 99.3% | 22.4 | 93.8% | 22.0 |

说明：

- sentry 整合的命中率比单 tracker 高 0.1–0.5 pp，但 hits/s 低 0.3–0.4，原因是 `TargetManager` 在 `DETECTING/TEMP_LOST` 期间不轻易开火、且 10 m 门限/切换滞回减少了一些边角射击。**命中率优先**的哨兵策略这是合理取舍；要追伤害量可把 `target.require_tracking`、`switch_margin`、`max_distance` 调回。
- 以上是**离线虚拟命中**，不是实弹命中率。`3.avi/4.avi` 只有红色 3 号步兵，前哨站/多目标没有真实视频，只有合成单测。
- 旧链路接入同一 judge 的探索性结果约 9%，但旧 planner 的 pitch/开火约定与 judge 的弹道模型并不完全一致（旧代码硬编码 22 m/s 无阻力、MCU 可能对 pitch 取反），**不能当作旧系统真实水平**；旧系统真实水平应以其原 MCU/弹道约定实测。这里只把它作为“旧链路与离线评测不兼容”的证据，不作为排名。

### 4.3 单元测试

`./build/autoaim_tests`：弹道、PnP 往返、EKF 收敛，全部通过。
`./build/sentry_tests`：

- 相机外参/云台姿态往返；
- 带外参的 IPPE + yaw 搜索（位置 < 0.25 m、yaw < 0.12 rad）；
- 前哨站 3 板 EKF 收敛（ω 误差 < 0.5 rad/s，r 误差 < 0.05 m）；
- 多目标管理器（两个目标、优先级选 4 号）；
- 前哨站 `TargetState` 瞄准（valid、t_fly、armor_k）；
- 串口 17 B 解析 / 21 B 打包。

---

## 5. 目录与接口

```
07_哨兵视觉_整合/
  include/autoaim/        新核头文件（types/config/detector/pose/tracker/target/ballistics/aimer/evaluation）
  include/sentry/         SentryVision、SerialCommand、串口协议
  include/io/             SerialPort
  src/                    实现（含 target.cpp、sentry.cpp、serial_protocol.cpp、io/serial_port.cpp）
  apps/
    sentry_offline.cpp    视频回放 + CSV + overlay
    sentry_robot.cpp      相机/视频 + 串口 + HoldFound
    autoaim_video.cpp     原评测入口（保留）
    detector_bench.cpp    检测器基准
  tests/
    test_main.cpp         原核测试
    test_sentry.cpp       整合测试
  config/sentry.yaml      哨兵配置（外参、前哨站、多目标）
  tools/                  标定、回归、旧检测器/旧链路对照
  docs/
```

关键接口：

```cpp
// 一帧
sentry::SentryVision vision(cfg);
auto out = vision.process(bgr, stamp, gimbal_yaw, gimbal_pitch, gimbal_roll);
// out.observations / out.targets / out.selected / out.command

// 串口
sentry::SerialCommand sc = sentry::toSerial(out.command, comp_yaw, comp_pitch, yaw_vel);
auto frame = sentry::packControl(sc);
serial.send(frame);

// 旧协议回传
sentry::AttitudeFrame a;
serial.query(stamp, a);   // yaw/pitch/roll/color/armor_or_buff
```

---

## 6. 与旧仓的关键差异（回答“缺哪些功能、有哪些提升”）

### 6.1 新核相对旧仓的提升

| 维度 | 旧仓 | 新核/整合 |
|---|---|---|
| 检测 | 单通道阈值 180；`minAreaRect` 端点 | `R−B` 阈值 70；包含灯条剔除；PCA 角点；同片召回 +0.3–0.5 pp |
| 位姿 | IPPE 单解 + ±70° 1° 重投影 | IPPE 双解 + pitch 先验 + 固定 pitch yaw 搜索 + `yaw_gain`；支持外参 |
| 整车 EKF | 10 维；无 dz；无 Singer；熵权匹配 | 11/13 维；双半径+dz；Singer；各向异性 R；马氏门控；Joseph |
| 预测误差 | 无系统评估 | 0.12/0.20/0.32 s 中位误差 5.0/8.3/14.3 px（视频 3） |
| 弹道 | 22 m/s 无阻力 | 25 m/s + 二次阻力 RK4；可配置 |
| 瞄准 | 最近板 + 五次多项式 | facing 选板 + `side_angle` + 3° 滞回 + `center_mode` + `sigma_lat` 开火门 |
| 前哨站 | 有（10 维 3 板） | 有（独立 `OutpostTracker`，统一 `TargetState`） |
| 多目标 | 单 `Aim`，7 m，ID 锁 | `TargetManager` 多 tracker，优先级 + 切换滞回 + 丢失保留 |
| 评测 | 仿真回放 | 真实视频虚拟打靶 + oracle + 预测误差 + 8 场景仿真 + 回归门禁 |
| 测试 | 无 | 原核测试 + 整合测试 |
| 配置 | `text.yaml` 写死路径 | `config/sentry.yaml`，路径编译期兜底 |

### 6.2 新核相对旧仓仍缺的功能

| 缺口 | 影响 | 处理 |
|---|---|---|
| 海康 MVS 驱动 | 不能直接上 IT13 取流 | 保留旧 `io/hikrobot.*`，在 `sentry_robot` 中把 `Query` 结果喂给 `vision.process`；本环境无 MVS SDK，未编译 |
| 能量机关（Buff） | 哨兵打符能力缺失 | 保留旧 `Buff/` 独立链路；按 `armor_or_buff` 路由；未并入新核 |
| ROS 2 | 无法直接接入 RMCS/导航 | 可选包一层；本环境无 ROS 2 |
| 相机内参/外参实测 | 离线 fx=2400、R_cam2gimbal 默认值 | 用 `tools/calibrate_camera.py` + 棋盘格；外参用旧 `text.yaml` 或重新测 |
| 弹速/延迟实测 | 默认 25 m/s、60 ms | 实测后改 `config/sentry.yaml`；否则虚拟命中率不代表实弹 |
| 前哨站真实数据 | 只有合成单测 | 录一段前哨站视频，用 `sentry_offline` 复跑，检查跟踪/命中 |
| 多目标真实数据 | 只有合成单测 | 录多机器人视频，调 `association_gate/switch_margin/priority` |
| 同数字多目标 | `id_` 只用数字，两个 3 号会共用标签 | 后续给 tracker 分配唯一 ID（位置/轨迹哈希），或在 `TargetManager` 用 entry id |
| 云台平滑 | 旧五次多项式未接 | 需要时把旧 `Planner` 作为 `AimCommand` 后处理；当前只发 `yaw_vel` 前馈 |
| 低帧率/侧对漏检 | 单灯条无法配对 | 新核同样存在；YOLO 关键点/单灯条补全可解，未做 |
| 数字模型 | 只有 “3” 验证 | 需补 1/2/4/outpost/guard/base 样本重训 |

---

## 7. 结论

- **检测、位姿、整车 EKF、弹道、瞄准、评测** 这六层用新核替换是明确的净收益；同片检测召回略升，整链命中率 67–68%，且有多目标/前哨站/外参能力。
- **驱动、串口、主流程状态机、Buff** 必须保留旧仓资产；新核不能单独上车。
- 整合后的 `SentryVision` 已经把“视觉算法”和“传输/相机”解耦；上 IT13 时只差三件事：MVS 适配、相机/外参/弹速/延迟实测、前哨站/多目标真实数据回归。
- 当前最值得做的下一步不是继续调 EKF，而是：**实测标定 + 100 fps 以上相机 + 前哨站/多目标数据**。这三件事决定实车上限。
