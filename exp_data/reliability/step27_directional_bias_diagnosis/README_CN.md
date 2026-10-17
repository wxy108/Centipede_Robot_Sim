# Centipede Robot 方向性位移病因归档

这个目录是 Steps 21--26 的统一归档，不复制原始大型 trajectory/video，
而是通过带 SHA-256 的 manifest 固定引用证据。

## 最终结论

原来观察到的“segment 越多，正向平均位移越大”由两个机制叠加：

1. 单端 floating root、动力学树方向和接触约束排序造成的非物理偏差；
2. 真实 STL、质量/惯量分布和首尾边界不对称在摩擦接触下形成的整流。

center-root 消除了 92.0--99.2% 的一阶 root bias。原模型中显著的
segment-number slope 在 center-root 和 head/tail average 中都不再显著。

严格 center-rooted symmetric dynamic null 的各个 \(N\) 均值置信区间
都包含零，整体 slope 为

\[
0.044\;[-1.148,1.232]\ \mathrm{mm/segment}.
\]

真实 center-root robot 仍保留约 32.75 mm 的整体平均位移，并且相对
dynamic null 的差异为

\[
31.21\;[25.73,36.70]\ \mathrm{mm}.
\]

因此，剩余位移不应再解释为 IID randomness 自己产生的偏置；当前最强
假设是真实前后形态极性和有限链边界经由切向摩擦、接触集合切换产生的
物理模型整流。仍需保留最多 7.31 mm 的已测 solver/root residual 作为
极端确定性条件下的数值不确定度。

## 文件索引

- `REPORT.md`：完整英文技术报告；
- `output/pdf/centipede_directional_bias_diagnosis_report.pdf`：正式 PDF；
- `figures/`：跨阶段核心图；
- `tables/causal_evidence_matrix.csv`：每个候选病因及其排查结论；
- `tables/model_stage_trends.csv`：不同修复阶段的 slope；
- `tables/robot_null_comparison.csv`：真实 robot 与 dynamic null 的逐 \(N\) 比较；
- `tables/source_artifact_manifest.csv`：原始证据文件校验和；
- `analysis_summary.json`：可机器读取的 bootstrap 统计。

## 后续使用规则

后续 noise/redundancy 实验使用 center-root physical model；正式结果同时
保留少量 matched head/center/tail audit，并报告：

\[
D_{\mathrm{debias}}=\frac{D_H+D_T}{2},\qquad
B_{\mathrm{root}}=\frac{D_H-D_T}{2},\qquad
E_C=D_C-D_{\mathrm{debias}}.
\]

