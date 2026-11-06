# 哨兵视觉后续任务（按收益排序）

> 目标：把当前“离线 68%”推到实车可用的稳定闭环。不要先调 EKF 参数，
> 先把信息条件和部署链路补齐。

## P0 — 上车前必须完成

1. **海康 MVS 适配**
   - 保留旧 `06_参考项目/io/hikrobot.*`，在新 `sentry_robot` 中用 `CameraSource`
     接口包一层；曝光中点时间戳必须传进来。
   - 验收：连续跑 2 h 不掉线；拔插 USB 后 3 s 内恢复；时间戳抖动 < 2 ms。

2. **相机内参 + 外参实测**
   - `tools/calibrate_camera.py` 标内参；`R_cam2gimbal/t_cam2gimbal` 用 3 m/6 m
     激光点对齐。
   - 验收：3 m/6 m 处静态装甲板重投影误差 < 3 px；yaw 误差 < 1°。

3. **弹速/延迟/云台响应实测**
   - 填 `ballistic.bullet_speed`、`drag_k`、`aimer.latency`、`gimbal_tau`、
     `gimbal_max_rate`。
   - 验收：10 m 固定靶 10 发 ≥ 8 中；移动靶虚拟/实弹差距可解释。

4. **串口联调**
   - 确认 21 B 字段顺序、yaw/pitch 正负、`found/fire` 语义、波特率、CRC（旧协议无 CRC）。
   - 验收：云台角度回传误差 < 0.2°；断线重连不炸；空目标 `fire=0`。

## P1 — 直接提升比赛表现

5. **前哨站 5 维固定几何（参考 rmcs）**
   - 当前 10 维估计 `r/z1/z2/z3`；前哨站尺寸固定，改成 5 维 `[x,y,z,w,yaw]` +
     固定半径/三板高度表，单板可见时更稳。
   - 文件：`include/autoaim/tracker.hpp` 的 `OutpostTracker`、`src/tracker.cpp`。

6. **DeviceId 唯一目标 ID**
   - 当前 `RobotTracker::id_ = armor.number`，两个 3 号会混；给每个 track 分配
     唯一 ID（自增/位置哈希），`TargetManager` 用它做切换滞回。
   - 文件：`TargetManager::Entry`、`TargetState::id`。

7. **火控 attack window / preaim / ff_a（参考 rmcs）**
   - 当前有 `side_angle`、`switch_hysteresis`、`center_mode`、`yaw_vel`；
     缺显式“来板窗口”和角加速度前馈。
   - 验收：4 号视频 8.3 rad/s 段命中率 +3 pp，hits/s 不降。

8. **前哨站/多目标真实视频**
   - 录 3 段：前哨站单板/双板、两个机器人、遮挡切换；用 `sentry_offline` 回归。
   - 验收：前哨站 tracking ratio > 0.9；目标切换不丢锁；误开火 = 0。

## P2 — 感知上限

9. **YOLO 关键点（侧对/远距离）**
   - 当前侧对只露一条灯条会漏检；YOLO11n-pose → ONNX → OpenVINO（Iris Xe）。
   - 需要标注数据；先做 500 帧小集验证召回，再决定是否替换传统检测。
   - 参考：illini `irmv_detection`、`zRzRzRzRzRzRzR/YOLO-of-RoboMaster-Keypoints-Detection-2023`。

10. **100+ fps 相机**
    - 当前 25 fps 是命中率的物理瓶颈；换 100–200 fps 相机后，`sentry_offline`
      和 tracker 都是时间戳驱动，不需要改算法，只需改 FPS/曝光。
    - 验收：同一场景 0.32 s 预测误差中位下降 30%+。

11. **Buff/能量机关**
    - 保留旧 `Buff/` 独立链路；按 `armor_or_buff` 路由；后续把旧 `Buff` 的
      `Small/BigTarget EKF` 接到统一 `TargetState`。

## P3 — 工程化

12. **ROS 2 包装（按队内要求）**
    - 只包 IO：`/camera_frame`、`/imu`、`/referee` 进，`/auto_aim/should_control`、
      `/should_shoot`、`/control_direction`、`/ff_v` 出；核心算法不动。
    - 参考 rmcs 的 Component 边界。

13. **日志/回放/可视化**
    - 当前 CSV 足够离线；上车建议加 ring buffer + 事件触发保存（丢失、开火、切换）。
    - 参考 rmcs 的 Foxglove/RTP，但不要用 ROS 图像话题传 100 fps 大图。

14. **回归门禁**
    - 当前 `tools/regression.py` 锁 3/4 视频；新增前哨站/多目标视频后，把
      tracking ratio、切换次数、误开火加入门禁。

## 不做/暂缓

- 继续扫 `q_yaw/q_r/singer_tau`：README 已证明平台期，收益 < 1 pp。
- 常加速度 CA/IMM：25 fps 下不可观，已实测退步。
- search area reduction：i9 全画面 6.3 ms，收益不大。
- 纯 CPU YOLO：15 ms 可跑 25 fps，但 100 fps 不够；先 OpenVINO/GPU。
