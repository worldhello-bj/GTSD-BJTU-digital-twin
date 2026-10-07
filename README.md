# GTSD BJTU · B8 冻结备份

这是两车机械物理工程原型及 Blender 源模型回放的 B8 冻结版本。22 组工况、273 项检查通过，状态为 PASS_WITH_MODEL_LIMITATIONS；包含 77 个广义机械速度坐标与 26 个气体质量状态。尚未实测标定，不代表整机数字孪生完成或安全认证。

## 完整下载

四个独立 ZIP 位于 [artifacts/B8](artifacts/B8)，全部下载并解压到同一工作目录。A 为 Blender 模型与回放，B 为物理源码与验收，C 为全部工况数值数据，D 为其余完整动态轨迹。这是 Git 版本化归档，不是 GitHub Release。

- [A：Blender 模型与回放](artifacts/B8/GTSD_BJTU_B8_A_Blender模型与回放.zip)
- [B：物理源码与验收](artifacts/B8/GTSD_BJTU_B8_B_物理源码与验收.zip)
- [C：全部工况数值数据](artifacts/B8/GTSD_BJTU_B8_C_全部工况数值数据.zip)
- [D：其余完整动态轨迹](artifacts/B8/GTSD_BJTU_B8_D_其余完整动态轨迹.zip)
- [总览 PNG](media/B8/b8_overview.png) · [求解器回放 MP4](media/B8/service_brake_solver_replay.mp4)

## 可读源码及验收索引

- [物理复现说明](source/vehicle-physics/b8/README.md)
- [最终验收报告](source/vehicle-physics/b8/FINAL_ACCEPTANCE.md)
- [273 项检查的机器可读结果](source/vehicle-physics/b8/results/verification.json)
- [结果解释](source/vehicle-physics/b8/RESULTS_ZH.txt)
- [模型覆盖范围](source/vehicle-physics/b8/COVERAGE.md)
- [气路边界](source/vehicle-physics/b8/BRAKE_CIRCUIT_SCOPE.md)
- [Blender 回放说明](source/GTSD_BJTU_B8/README_回放适配说明.md)
- [文件校验清单](BACKUP_MANIFEST.json)

source/ 仅展开源码、参数和主要报告，全部数值与动态轨迹保留在四包内，不重复展开大数据。运行方法以 B 包及其 README 为准。没有配置自动执行的 GitHub Actions。

## 必须保留的限制

所有实机质量、刚度、摩擦与气路参数未标定。软线/软管、阀芯、压缩机内部、风扇及把手内部动作尚未完整物理化；门闩、传动、接触和结构仍含理想化与等效模型。历史原生接触收敛失败及域外过载失败保留在验收材料中。具体边界见上述最终验收报告。

本仓库保留 B8 冻结交付，不包含 B9，不包含缓存、凭据或外部整本参考资料。原始四包的字节、SHA-256 和 Git blob SHA 见校验清单。
