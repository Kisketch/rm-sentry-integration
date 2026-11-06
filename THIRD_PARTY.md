# Third-party notices

This tree is an integration of the author's own work and ideas from open-source
RoboMaster projects. No third-party source file was copied except where noted.

| Component | Origin | License | Use here |
|---|---|---|---|
| `include/autoaim`, `src/*` (except sentry/io/target) | Kisketch/rm-autoaim (`203dd4c`) | MIT | algorithm core |
| `model/mlp.onnx`, `model/label.txt` | assessment template / rm_auto_aim lineage | see origin | digit classifier |
| `tools/legacy_include/Base.h` | CSU-FYT-Vision style sentry reference (`06_参考项目_auto-aim-master`) | PSL v2 | comparison harness only; not linked into the shipped vision core |
| `tools/legacy_pipeline.cpp`, `tools/legacy_detector_bench.cpp` | original comparison harness | MIT | offline comparison; links the old tree only when explicitly built |
| Ideas: `side_angle`, PCA corner refinement, fixed-pitch yaw search | chenjunnn/rm_auto_aim (MIT), CSU-FYT-Vision/FYT2024_vision (Apache-2.0) | see origin | re-implemented, not copied |
| Ideas: outpost 5-state, attack window, feedforward | Alliance-Algorithm/rmcs_auto_aim_v2 (MIT) | MIT | documented as next step, not copied |
| Ideas: YOLO/TensorRT/triple buffering | illini-robomaster/irmv_detection (MIT) | MIT | documented as future work, not copied |

The old sentry tree (`06_参考项目_auto-aim-master`) is **not vendored** into the
shipped core. The two legacy tools are optional and build only when the old tree
is present on disk (`tools/build_legacy_bench.sh`).
