# 实测记录（2026-11-06）

环境：Ubuntu 26.04.1，g++ 15.2.0，OpenCV 4.10.0，8 核；视频
`/home/lgnatius/video/3.avi`（2209 帧）、`4.avi`（3416 帧），1440×1080@25fps，
红色 3 号步兵。所有数字来自本机真实运行，可用下面的命令复现。

## 1. 构建与测试

```bash
cd 07_哨兵视觉_整合
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
ctest --test-dir build --output-on-failure
```

```
1/2 Test #1: autoaim_tests ... Passed
2/2 Test #2: sentry_tests ... Passed
100% tests passed, 0 tests failed out of 2
```

`sentry_tests` 覆盖：相机外参/云台姿态往返、带外参 PnP、前哨站 3 板 EKF、
多目标优先级、前哨站 TargetState 瞄准、串口 17 B/21 B。

## 2. 检测器对比（同视频、同帧）

旧哨兵检测器（`06_参考项目.../ArmorDetector.cpp`，`SENTRY` 宏）：

```bash
./build/legacy_detector_bench /home/lgnatius/video/3.avi runs/legacy_3.csv
./build/legacy_detector_bench /home/lgnatius/video/4.avi runs/legacy_4.csv
```

```
3: frames=2209 detected_frames=2125 armors=2317 ms_per_frame=4.67
4: frames=3416 detected_frames=3329 armors=3827 ms_per_frame=4.72
```

新检测器：

```bash
./build/detector_bench /home/lgnatius/video/3.avi runs/new_3.csv --config config/sentry.yaml
./build/detector_bench /home/lgnatius/video/4.avi runs/new_4.csv --config config/sentry.yaml
```

```
3: frames=2209 detected_frames=2132 armors=2380 ms_per_frame=6.27
4: frames=3416 detected_frames=3343 armors=3885 ms_per_frame=6.34
```

| 视频 | 旧有目标帧 | 新有目标帧 | 旧装甲板 | 新装甲板 | 旧 ms | 新 ms |
|---|---|---|---|---|---|---|
| 3.avi | 2125 (96.2%) | **2132 (96.5%)** | 2317 | **2380** | 4.67 | 6.27 |
| 4.avi | 3329 (97.4%) | **3343 (97.9%)** | 3827 | **3885** | 4.72 | 6.34 |

结论：新检测器召回略高，代价约 1.6 ms/帧（PCA + 分类）。6.3 ms 在 IT13 上仍
≈158 fps 检测吞吐。

## 3. 整链虚拟打靶

```bash
./build/autoaim_video /home/lgnatius/video/3.avi --config config/sentry.yaml --out runs/eval_sentry
./build/autoaim_video /home/lgnatius/video/4.avi --config config/sentry.yaml --out runs/eval_sentry
```

3.avi：

| 策略 | shots | verifiable | hits | hit rate | hits/s |
|---|---:|---:|---:|---:|---:|
| direct | 1943 | 1856 | 658 | 35.5% | 7.4 |
| point KF | 1743 | 1653 | 489 | 29.6% | 5.5 |
| robot EKF | 1441 | 1389 | 938 | 67.5% | 10.6 |
| sentry（多目标+前哨站） | 1387 | 1336 | 908 | **68.0%** | **10.3** |
| oracle | 2010 | 1990 | 1977 | 99.3% | 22.4 |

4.avi：

| 策略 | shots | verifiable | hits | hit rate | hits/s |
|---|---:|---:|---:|---:|---:|
| direct | 2927 | 2840 | 1611 | 56.7% | 11.8 |
| point KF | 1138 | 1096 | 472 | 43.1% | 3.5 |
| robot EKF | 2039 | 1994 | 1363 | 68.4% | 10.0 |
| sentry（多目标+前哨站） | 1961 | 1918 | 1314 | **68.5%** | **9.6** |
| oracle | 3229 | 3198 | 3001 | 93.8% | 22.0 |

跟踪比例 0.933 / 0.928；有装甲板帧比例 0.965 / 0.979。
sentry 策略命中率略高、hits/s 略低，原因是 `TargetManager` 在 DETECTING 阶段
不轻易开火 + 目标切换滞回；这是“命中率优先”的默认取舍。

## 4. 哨兵离线 CSV

```bash
./build/sentry_offline /home/lgnatius/video/3.avi --config config/sentry.yaml --out runs/sentry3
./build/sentry_offline /home/lgnatius/video/4.avi --config config/sentry.yaml --out runs/sentry4
```

| 视频 | 帧数 | 有检测帧 | 锁定帧 | 开火帧 | 最大目标数 |
|---|---:|---:|---:|---:|---:|
| 3.avi | 2209 | 2132 | 2209 | 1397 | 1 |
| 4.avi | 3416 | 3343 | 3416 | 1990 | 1 |

（`locked` 包含 DETECTING 的“跟但不打”；`fire` 只统计 `cmd.fire`。）

## 5. 旧链路接入同一 judge 的探索

```bash
./build/legacy_pipeline /home/lgnatius/video/3.avi runs/eval_sentry/3_detections.txt runs/legacy_pipeline_3.csv config/sentry.yaml
```

结果约 `shots=1625 verifiable=1538 hits=140 hit_rate=9.1%`。

**不要把这个数字当作旧系统真实水平**：旧 `Planner` 用 22 m/s 无阻力、MCU 的 pitch
符号约定未知、`predict_time=0.075` 是手调；接入新 judge 时这些约定没有完全对齐。
它只能说明“旧链路与离线评测不兼容”，不能说明旧系统实车打不准。旧系统的真实
水平应在其原 MCU/弹道约定下实测。

## 6. 结论

- 替换检测/位姿/EKF/弹道/瞄准后，整链 67–68% 命中、10 hits/s，和基线一致；
- 多目标/前哨站适配没有破坏单目标视频的表现；
- 检测器召回略升，耗时仍在预算内；
- 下一步的瓶颈是**实测标定 + 高帧率相机 + 前哨站/多目标真实数据**，不是继续
  微调 EKF。
