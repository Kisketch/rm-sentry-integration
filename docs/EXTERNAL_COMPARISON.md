# 外部同类项目对照（GitHub / RM 社区）

对照时间：2026-11-06。目的不是“抄代码”，而是确认新核在同类系统里的位置，
并挑出值得移植的工程点。许可证各自保留；本仓库只吸收思路，未复制代码。

---

## 1. Alliance-Algorithm / rmcs_auto_aim_v2（南京理工，MIT）

仓库：<https://github.com/Alliance-Algorithm/rmcs_auto_aim_v2>

**它是目前工程化程度最高的参考之一**：ROS 2 Component、单进程多线程、
相机/IMU 快照、Foxglove/RTP 可视化、单元测试占一半代码量。

关键实现（读源码得到）：

| 模块 | 做法 | 与本仓库的关系 |
|---|---|---|
| 整车 EKF | 11 维 `[x,y,z,vx,vy,vz,yaw,w,radius_forward,radius_lateral,height_lateral]` | 与我们的 `[xc,vxc,yc,vyc,zc,vzc,yaw,w,r1,r2,dz]` 等价；他们用前向/侧向半径 + 高度差，我们用双半径 + dz |
| 前哨站 | **5 维** `[x,y,z,w,yaw]`，半径/三板高度固定查表，切板检测阈值 50–80° | 我们目前是 10 维（估计 r 和 z1/z2/z3）。**固定几何更稳**，是下一步可替换点 |
| 位姿 | IPPE PnP + yaw 优化：±70°（140°）搜索、1° 步长、三点二次插值；标准板 pitch +15°，前哨站 −15° | 我们的 85° 粗搜 + 黄金分割 + `yaw_gain` 是同一类；他们的前哨站 pitch 与我们的 `outpost_armor_pitch_deg=-15` 一致 |
| 火控 | `attack_window`、`window_redundancy`、`window_hysteresis`、`attack_preaim`、`degraded_angle_speed`、`ff_v/ff_a` 前馈 | 我们有 `side_angle`、`switch_hysteresis`、`center_mode`、`yaw_vel`；缺 `ff_a` 和显式 attack window 状态机 |
| 多目标 | `unordered_map<DeviceId, RobotModel>` + 优先级表 + 超时 | 我们的 `TargetManager` 同类；他们用 `DeviceId` 唯一标识，我们目前用数字标签（同数字多目标会混） |
| 能量机关 | 独立 RuneModel + `rune_energy_fitter` + 虚拟 Rune | 我们保留旧 `Buff/`，未并入 |
| 输入 | 相机帧自带曝光中点 IMU 姿态；`linear_sync_model` 做时间对齐 | 我们已加 `setGimbalAttitude` + `SerialPort::query` 插值；曝光中点时间戳仍需驱动支持 |

**已吸收**：相机外参 + 云台姿态、前哨站独立模型、多目标、串口、`yaw_vel` 前馈。
**待吸收**：前哨站 5 维固定几何、`ff_a`、attack window/preaim、`DeviceId` 唯一 ID。

---

## 2. PurdueRM / auto-aiming（GPL-3.0）

仓库：<https://github.com/PurdueRM/auto-aiming>

- ROS 2 Humble + Jetson Orin；120 Hz 检测目标；
- 功能需求写得很具体：5 m 90% / 3 m 95% / 2 m 95% 检出率、5% 像素损失；
- **search area reduction**：在上一次检测附近缩小搜索区；
- **validity filter**：按距离、XYZ 跳变过滤错误 PnP；
- 弹道用 lookup table / 简单模型；
- GTest 覆盖各模块。

**可吸收**：validity filter（PnP 跳变/距离门）我们已经部分有（`distance 0.2–30 m`、马氏门），
但没有显式的“单帧 PnP 跳变”过滤；search area reduction 在 i9 上收益不大（当前 6.3 ms
全画面），1000 fps 或高分辨率时才值得。

---

## 3. illini-robomaster / irmv_detection（MIT）

仓库：<https://github.com/illini-robomaster/irmv_detection>

- ROS 2 + YOLOv8n（640×640）+ TensorRT FP16；
- Jetson Orin Nano 约 5 ms，RTX 3060 约 4–5 ms；
- **triple buffering + Unified Memory** 做零拷贝，规避 ROS 2 图像话题拷贝；
- GPU 预处理/NMS/CUDA Streams/CUDA Graphs。

**结论**：这是“检测换成 YOLO”的工程路径。我们的传统检测在 1440×1080 上 6.3 ms，
已经够 25–100 fps；上 YOLO 的收益主要是**侧对/远距离召回**，不是速度。
IT13 的 Iris Xe 可用 OpenVINO，但训练数据/标注是前置条件，暂不替换。

---

## 4. 其他血统（已在新核 README 致谢）

| 项目 | 吸收点 |
|---|---|
| chenjunnn/rm_auto_aim（MIT） | 灯条/配对、IPPE、MLP、整车 EKF 状态机 |
| CSU-FYT-Vision/FYT2024_vision（Apache-2.0） | `side_angle` 提前换板、PCA 角点、固定 pitch BA 思路、`d_z` |
| SJTU-RM-CV-2019（MIT） | `LightBlob` 模板、R−B 颜色 |
| TUP-InfantryVision-2022（GPL-3.0） | 自适应 EKF/粒子滤波思路（未复制） |
| robomaster-oss/rmoss_core（Apache-2.0） | 弹道/云台接口思路 |

---

## 5. 辽宁科技大学 COD 哨兵全开源（RM 社区，2026-03）

社区文章：<https://bbs.robomaster.com/article/1882897>

- 覆盖哨兵导航、决策、自瞄、下位机；
- 文章本身未给出可克隆的 GitHub 链接（至少公开页没有）；
- 它的价值是**系统边界**：哨兵不是单一自瞄，导航/决策/自瞄/下位机要分层，
  自瞄只输出“是否控制/是否开火/方向/前馈”，不要直接抢云台。

这与 rmcs 的 `should_control/should_shoot/control_direction/ff_v/ff_a` 接口一致，
也是我们 `SentryVision → AimCommand → 串口` 的设计依据。

---

## 6. 对照结论

1. 我们的**检测→位姿→整车 EKF→弹道→瞄准**与 rmcs/Purdue 同级；差异主要在
   前哨站参数化、火控窗口、工程化（ROS 2/测试/可视化）。
2. 最值得马上移植的三点：
   - 前哨站 **5 维固定几何**（比 10 维更稳，尤其只有单板可见时）；
   - 火控 **attack window + preaim + ff_a**（我们只有 `side_angle` + `yaw_vel`）；
   - **DeviceId 唯一目标 ID**（解决两个同数字机器人）。
3. 不建议现在做的：YOLO（缺数据/标注）、ROS 2（缺队内接口）、search area
   reduction（i9 全画面 6.3 ms 足够）。
4. 我们的优势：**离线评测闭环**（真实视频虚拟打靶 + oracle + 预测误差 + 8 场景仿真 +
   回归门禁）比上述公开仓更完整；这是继续调参的前提，不要为了“功能多”丢掉它。
