# Step 37 视频与图表索引

本轮新增 **21 个完整 demo**：N=5、N=6 的 IID 各 7 个，以及 N=10 的全 pitch-down reference 7 个。每个视频覆盖约 18.85 s 的完整评估段，20 fps。

所有条件保持 leg yaw 固定、body yaw 半幅 19°，沿用本轮增强后的 moderate PD。原始模型 XML 未修改。

## 一、视频与各 phase offset 的 SD 图

波数标签以 N=10 为基准：\(\Delta\phi=-2\pi N_{\mathrm{wave}}^{(10)}/9\)。改变 N 时锁定的是相邻关节 phase offset，不是机器人上实际包含的完整波数。

| 参考波数 | 锁定 phase offset | N=5 IID | N=6 IID | N=10 全 pitch down | SD vs N |
|---:|---:|---|---|---|---|
| 1.0 | -40° | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N5/N5_Nwave10_1p0_iid_contact_opportunity_seed029.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N6/N6_Nwave10_1p0_iid_contact_opportunity_seed041.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/demos/N10_Nwave10_1p0_all_pitch_down.mp4) | [单独图](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_SD_vs_segment_Nwave10_1p0.png) |
| 1.5 | -60° | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N5/N5_Nwave10_1p5_iid_contact_opportunity_seed056.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N6/N6_Nwave10_1p5_iid_contact_opportunity_seed064.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/demos/N10_Nwave10_1p5_all_pitch_down.mp4) | [单独图](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_SD_vs_segment_Nwave10_1p5.png) |
| 2.0 | -80° | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N5/N5_Nwave10_2p0_iid_contact_opportunity_seed053.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N6/N6_Nwave10_2p0_iid_contact_opportunity_seed103.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/demos/N10_Nwave10_2p0_all_pitch_down.mp4) | [单独图](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_SD_vs_segment_Nwave10_2p0.png) |
| 2.5 | -100° | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N5/N5_Nwave10_2p5_iid_contact_opportunity_seed042.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N6/N6_Nwave10_2p5_iid_contact_opportunity_seed056.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/demos/N10_Nwave10_2p5_all_pitch_down.mp4) | [单独图](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_SD_vs_segment_Nwave10_2p5.png) |
| 3.0 | -120° | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N5/N5_Nwave10_3p0_iid_contact_opportunity_seed067.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N6/N6_Nwave10_3p0_iid_contact_opportunity_seed000.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/demos/N10_Nwave10_3p0_all_pitch_down.mp4) | [单独图](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_SD_vs_segment_Nwave10_3p0.png) |
| 3.5 | -140° | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N5/N5_Nwave10_3p5_iid_contact_opportunity_seed055.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N6/N6_Nwave10_3p5_iid_contact_opportunity_seed015.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/demos/N10_Nwave10_3p5_all_pitch_down.mp4) | [单独图](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_SD_vs_segment_Nwave10_3p5.png) |
| 4.0 | -160° | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N5/N5_Nwave10_4p0_iid_contact_opportunity_seed047.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/demos/N6/N6_Nwave10_4p0_iid_contact_opportunity_seed063.mp4) | [播放](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/demos/N10_Nwave10_4p0_all_pitch_down.mp4) | [单独图](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_SD_vs_segment_Nwave10_4p0.png) |

绿色表示 **pitch-down 指令**，红色表示 **pitch-up 指令**，不代表直接开启/关闭地面接触。视频底部把下压指令数量与实际接触数量分开显示。

全 pitch-down reference 中，所有腿在稳定阶段后持续保持 +5° 下压目标，不再随机或交替抬腿；实际触地仍由动力学决定。每个 phase offset 是一个确定性 reference trial，不是统计均值。

## 二、统计图与原始数据

- [所有 phase offset：SD vs N 总览](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_distribution_SD_vs_segment_all_phase_offsets.png)
- [各 phase offset 的 SD 趋势斜率及 95% 区间](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/figures/iid_phase_offset_SD_trend.png)
- [SD 数值及各条件 bootstrap 区间](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/sd_by_phase_and_segment.csv)
- [SD 趋势数值](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/sd_dispersion_trends.csv)
- [原始 5,376 条 IID trials](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/iid_trials.csv)
- [全 pitch-down：位移 vs phase offset](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/figures/all_pitch_down_displacement_vs_phase_offset.png)
- [全 pitch-down：实际接触比例](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/figures/all_pitch_down_actual_contact_vs_phase_offset.png)
- [N=10：IID 与全 pitch-down 描述性对照](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/figures/N10_IID_vs_all_pitch_down_displacement.png)
- [全 pitch-down 原始结果](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/all_pitch_down_reference.csv)
- [完整解读与结论边界](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/FINDINGS.md)

SD 为样本标准差（ddof=1）。7 个 phase offset 的拟合 SD 斜率均为负，各自 95% bootstrap 区间均不跨零；不是逐点单调结论，区间未做多重比较调整。原先主指标 IQR 的结论不变。

## 三、重要结果与回放边界

全 pitch-down 也从参考波数 1 的 +0.840 m 降到参考波数 4 的 -0.151 m。因此，负位移不需要 IID 才能出现；此前把反向位移直接归因于 IID 破坏接触相位耦合的解释过强。具体动力学/数值机制仍需定位。

原始 IID 扫描没有记录 MuJoCo 版本。本轮视频及全下压 reference 使用 MuJoCo 3.3.7。小尺寸 IID demo 选择原始数据中最接近该条件中位数的 seed 后重新模拟，并非保存轨迹的精确回放：最大位移差为 0.0330 m；N=5、参考波数 2.5 的接近零样本从 +0.0101 m 变为 -0.0172 m。所有差异均保留在 manifest 中，没有替换原始 trial 数据。

因此，IID/reference 叠图只作描述性比较，不能把两条曲线之差当作已隔离出的 IID 因果效应。

- [小尺寸视频 provenance 与全部位移差异](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/small_N_demo_manifest.csv)
- [小尺寸视频验证](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/small_N_demo_validation.json)
- [全下压 reference 验证](C:/Users/wxy22/Documents/Centipede_robot_MUJOCO/exp_data/reliability/step37_locked_phase_segment_iid_sweep/all_pitch_down_reference/validation.json)

