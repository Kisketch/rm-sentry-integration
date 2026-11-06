# IT13 哨兵部署清单（Ubuntu 22.04）

> 目标机：GEEKOM Mini IT13，i9-13900HK（6P+8E，20 线程），Iris Xe 96EU，32 GB DDR4，
> 1 TB PCIe 4.0 NVMe，2.5G 网口，19 V DC。相机：海康 MVS 工业相机（旧仓 `io/hikrobot.*`）。

## 1. 系统与依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev \
                    libyaml-cpp-dev libeigen3-dev libusb-1.0-0-dev \
                    v4l-utils ffmpeg
```

- Ubuntu 22.04 自带 OpenCV 4.5.x 可用；建议 4.8–4.10（与开发机一致）。检查：
  `pkg-config --modversion opencv4`。
- MVS SDK：安装海康 `MVS-xxx.x86_64.deb`，默认 `/opt/MVS`。确认
  `ls /opt/MVS/include/MvCameraControl.h`。
- 串口权限：`sudo usermod -aG dialout $USER`，重新登录；`ls -l /dev/ttyACM0`。
- 关闭 USB 自动挂起（海康掉线常见）：
  ```bash
  sudo sed -i 's/GRUB_CMDLINE_LINUX_DEFAULT="/GRUB_CMDLINE_LINUX_DEFAULT="usbcore.autosuspend=-1 /' /etc/default/grub
  sudo update-grub && sudo reboot
  ```

## 2. 构建

```bash
cd ~/sentry_vision
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

产物：

- `build/sentry_offline`：视频回放 + CSV + overlay，不依赖相机/串口；
- `build/sentry_robot`：相机/视频 + 串口；
- `build/autoaim_video`：原评测（虚拟打靶）；
- `build/sentry_tests`、`build/autoaim_tests`。

## 3. 标定（上车前必须做）

### 3.1 相机内参

```bash
python3 tools/calibrate_camera.py 'calibration/*.png' --cols 9 --rows 6 \
  --square-mm 25 --out config/my_camera.yml
```

把输出的 `fx/fy/cx/cy/dist` 填到 `config/sentry.yaml` 的 `camera` 段。
至少 12 张、覆盖画面四角与中心；检查重投影误差 < 0.5 px。

### 3.2 相机 → 云台外参

- 旧仓 `text.yaml` 的 `Transform.R_camera2gimbal/t_camera2gimbal` 可作为起点。
- 新配置：
  ```yaml
  camera:
     R_cam2gimbal: [0., 0., 1., -1., 0., 0., 0., -1., 0.]
     t_cam2gimbal: [0.10, -0.05, 0.]
  ```
- `R_cam2gimbal` 是**行主序** 3×3，相机光学系（x 右、y 下、z 前）→ 云台系
  （x 前、y 左、z 上）。`t_cam2gimbal` 单位米。
- 测量方法：云台 yaw=0、pitch=0 时，相机光轴与枪管平行；用标定板/激光点
  在 3 m 和 6 m 处对齐，调 `t` 的 y/z；`x` 用相机安装面到云台转轴的距离。

### 3.3 云台姿态与时间戳

- 串口 17 B 帧给 yaw/pitch/roll；`SerialPort::query(stamp, a)` 会按图像时间戳插值。
- **曝光中点时间戳**比“收到帧的时间”更重要；如果相机 SDK 能给曝光中点，直接用它，
  否则用 `steady_clock` 收到帧时间减去 `exposure/2`。
- 检查 `sentry_robot` 里 `attitude.stamp` 与 `stamp` 的偏差；超过 5 ms 要查驱动/线程。

### 3.4 弹速、延迟、云台响应

| 参数 | 默认 | 怎么测 |
|---|---|---|
| `ballistic.bullet_speed` | 25 m/s | 测速仪/高速相机，或打 5 m、10 m 靶反推 |
| `ballistic.drag_k` | 0.019 | 17 mm 弹丸理论值；远距离落点偏差大时再拟合 |
| `aimer.latency` | 0.06 s | 从曝光中点到弹丸出膛；用 LED+示波器或高速相机 |
| `aimer.gimbal_tau` | 0.03 s | 阶跃响应 63% 时间 |
| `aimer.gimbal_max_rate` | 12 rad/s | 云台最大角速度 |
| `aimer.fire_sigma_scale` | 0（关） | 想用协方差门时再开；Q 未校准前不建议 |

## 4. 运行

### 4.1 离线回放（先验证算法）

```bash
./build/sentry_offline ~/video/3.avi --config config/sentry.yaml \
  --out runs/3 --render
python3 - <<'PY'
import csv
rows=list(csv.DictReader(open('runs/3/sentry_track.csv')))
print('locked', sum(1 for r in rows if int(r['selected'])>=0), '/', len(rows))
print('fire', sum(1 for r in rows if int(r['fire'])))
PY
```

### 4.2 实车（视频/相机 + 串口）

```bash
# 用相机（V4L2；MVS 相机需先接 hikrobot 适配）
./build/sentry_robot --camera 0 --serial /dev/ttyACM0 \
  --config config/sentry.yaml --comp-yaw 0 --comp-pitch 0

# 用录好的视频回放，不带串口
./build/sentry_robot --video ~/video/3.avi --serial "" --config config/sentry.yaml
```

MVS 适配骨架（保留旧 `io/hikrobot.*`，只替换取帧循环）：

```cpp
#include "hikrobot.hpp"   // 旧仓 io/include，需要 MVS SDK

io::HikRobot camera(exposure_us, gain, "2bdf:0001");
time_point t;
cv::Mat frame;
while (running) {
    camera.Query(frame, t);                 // 旧驱动：图像 + 时间戳
    const double stamp = toSeconds(t);      // steady_clock -> double
    auto out = vision.process(frame, stamp, yaw, pitch, roll);
    // ... 串口发送 / 可视化
}
```

如果 MVS SDK 能给“曝光中点”时间，优先用它；否则用 `t - exposure/2` 修正。

- `--comp-yaw/--comp-pitch` 是旧仓 `Transmit.compensate_*` 的等价物；先 0，实测偏置后再加。
- 程序内 `HoldFound 0.5 s`：目标短暂丢失时保持上一帧角度，但**不发火**。
- 串口 21 B 帧：`AA BB [yaw][pitch][yaw_vel][ui_x] found fire BB`，
  yaw/pitch/yaw_vel 单位 1e-4 度，`found=1`，`fire=2` 表示开火。
- 若 MCU 的 pitch 正负与视觉相反，在 `toSerial` 里对 pitch 取负，或在电控端处理；
  **不要**同时改两处。

### 4.3 systemd 自启

```ini
# /etc/systemd/system/sentry-vision.service
[Unit]
Description=Sentry Vision
After=network.target

[Service]
Type=simple
User=lgnatius
WorkingDirectory=/home/lgnatius/sentry_vision
ExecStart=/home/lgnatius/sentry_vision/build/sentry_robot \
  --camera 0 --serial /dev/ttyACM0 --config config/sentry.yaml
Restart=always
RestartSec=2
Environment=DISPLAY=:0

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now sentry-vision
journalctl -u sentry-vision -f
```

## 5. 性能预算（IT13，单线程）

| 阶段 | 实测/估算 |
|---|---|
| 检测（1440×1080，含 MLP） | 6.3 ms |
| 位姿（IPPE + yaw 搜索） | 0.5 ms |
| 跟踪 + 瞄准 + 判定 | 0.8 ms |
| 合计 | ~7.6 ms/帧 → ~130 fps 视觉吞吐 |
| 25 fps 占用 | ~19% 单核 |
| 100 fps 占用 | ~76% 单核；建议把检测放 P 核，串口/相机线程放 E 核 |

- 若上 YOLO（Iris Xe / OpenVINO），先量化到 640×640；CPU 15 ms 可接受，但 100 fps 需 GPU/NPU。
- 2.5G 网口可接工业相机/图传；USB4 40 Gbps 可接高速相机，但注意供电与线缆。

## 6. 上车前检查表

- [ ] `ctest` 两个测试全过；
- [ ] `sentry_offline` 在 3.avi/4.avi 上 locked > 95%，fire 帧数合理；
- [ ] 相机内参重投影误差 < 0.5 px；
- [ ] 云台 yaw=0/pitch=0 时，3 m 与 6 m 激光点与枪管重合；
- [ ] 串口回传 yaw/pitch/roll 与云台实际角度误差 < 0.2°；
- [ ] 弹速/延迟实测并写入 yaml；
- [ ] 断相机、断串口、拔电重启三种故障下程序不崩、能重连；
- [ ] 空弹/无目标时 `fire` 恒为 0；
- [ ] 前哨站/多目标至少各录一段视频回归（当前只有合成单测）。
