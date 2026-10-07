# GTSD BJTU · B8 冻结备份与 B9/B10 物理增量

这是两车机械物理工程原型及 Blender 源模型回放的 B8 冻结版本。22 组工况、273 项检查通过，状态为 PASS_WITH_MODEL_LIMITATIONS；包含 77 个广义机械速度坐标与 26 个气体质量状态。尚未实测标定，不代表整机数字孪生完成或安全认证。

## B10 压缩机与风机增量

在 B9 阀芯和冻结 B8 整车上接入有限质量气缸、力驱动压缩机、两台电压驱动风机及两个热容。活塞运动和供气由压力反力与电机转矩共同决定，断电保留惯性，压力开关只输出电压命令。新增内部结构采用明确推定的固定试验台等效模型，参数未标定，安装质量和反力尚未传入车体。

- [B10 源码与复现](source/vehicle-physics/b10/README.md) · [数值结果](source/vehicle-physics/b10/RESULTS.md) · [266 项验收检查](source/vehicle-physics/b10/results/verification.json)
- [完整 B10 增量包：源码、数据与 Blender 工程](artifacts/B10/GTSD_BJTU_B10_auxiliary_dynamics.zip) · [包校验](artifacts/B10/PACKAGE_VERIFICATION.json) · [解包运行检查](artifacts/B10/UNPACKED_SMOKE.json)
- [辅机响应图](source/vehicle-physics/b10/results/b10_auxiliary_response.png) · [Blender 工程重读检查](source/vehicle-physics/b10/replay/reports/b10_readback.json) · [辅机试验台预览](source/vehicle-physics/b10/replay/renders/b10_auxiliary_bench.png)

三组完整整车工况、六组 120 秒辅机工况、16 项组件测试和 36 项回归测试通过。保留 B8 原阈值，半步长停车距离变化约 0.0059%。Blender 的独立辅机场景展示已求解的启动轨迹，显示减速 33.3 倍。软线/软管与设备真实内部结构仍待进一步推进。

## B9 物理化增量（历史检查点）

供气/排气端口增加力驱动阀芯：线圈力、质量、弹簧、阻尼、软止挡和遮盖量决定实际开口，再驱动原有限质量气路。19 项模块检查和 35 项控制/旧 B8 回归检查通过；五组完整 B9 工况检验正常制动、半步长、阀芯迟滞、供气线圈失效及正常/快速排气。B8 原 22 组保留为历史验收；本次没有重跑全部 22 组。

- [B9 源码与复现](source/vehicle-physics/b9/README.md) · [数值结果](source/vehicle-physics/b9/RESULTS.md) · [验收 JSON](source/vehicle-physics/b9/results/verification.json)
- [完整 B9 增量包：源码、数据与 Blender 工程](artifacts/B9/GTSD_BJTU_B9_valve_dynamics.zip) · [包校验](artifacts/B9/PACKAGE_VERIFICATION.json)
- [阀芯—压力—车速对比图](source/vehicle-physics/b9/results/b9_valve_response.png) · [Blender 保存重读检查](source/vehicle-physics/b9/replay/reports/b9_readback.json)

新增 51 个独立压力平衡计量端口代理、102 个阀芯状态；这是数值拓扑，不是实物阀数量。阀芯参数未标定，固定实验台支承反力未传入车体。B9 检查点时软线/软管、压缩机内部、风扇叶轮尚未物理化；后续辅机等效模型见 B10。B8 冻结源码、原包和历史失败均保留。

## 一键恢复完整四包

A/B/D 以无损二进制分片保存（每包 2 片），C 为单个完整 ZIP。连接器请求体上限使较大 ZIP 不能直接上传；没有删减或重新压缩原包。下载整个仓库（Code → Download ZIP）并解压，或 git clone 后，在仓库目录运行：

```sh
python3 restore_b8.py
```

脚本只在本地核对文件并重组，不联网、不安装依赖、不运行仿真。它验证所有存储文件与原包 SHA-256、大小及 ZIP CRC。完成后原始四包位于 artifacts/B8/。本次已在干净目录实际恢复并确认与原ZIP字节一致。

## 下载索引

四个独立 ZIP 的归档位于 [artifacts/B8](artifacts/B8)，先按上面步骤恢复，再将四包解压到同一工作目录。A 为 Blender 模型与回放，B 为物理源码与验收，C 为全部工况数值数据，D 为其余完整动态轨迹。这是 Git 版本化归档，不是 GitHub Release。

- [A：Blender 模型与回放：第1片](artifacts/B8/GTSD_BJTU_B8_A_Blender模型与回放.zip.part001) · [第2片](artifacts/B8/GTSD_BJTU_B8_A_Blender模型与回放.zip.part002)
- [B：物理源码与验收：第1片](artifacts/B8/GTSD_BJTU_B8_B_物理源码与验收.zip.part001) · [第2片](artifacts/B8/GTSD_BJTU_B8_B_物理源码与验收.zip.part002)
- [C：全部工况数值数据](artifacts/B8/GTSD_BJTU_B8_C_全部工况数值数据.zip)
- [D：其余完整动态轨迹：第1片](artifacts/B8/GTSD_BJTU_B8_D_其余完整动态轨迹.zip.part001) · [第2片](artifacts/B8/GTSD_BJTU_B8_D_其余完整动态轨迹.zip.part002)
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

所有实机质量、刚度、摩擦与气路参数未标定。B9 阀芯及 B10 压缩机、风机为数值等效模型，不能作为真实内部结构的证据；软线/软管、把手内部动作尚未完整物理化。门闩、传动、接触和结构仍含理想化。历史原生接触收敛失败及域外过载失败保留在验收材料中。具体边界见各版本验收报告。

本仓库保留 B8 冻结交付，并单独保存 B9、B10 增量。按交付包清单与启发式扫描排除缓存、凭据及外部整本参考资料。原始四包的字节、SHA-256 和 Git blob SHA 见校验清单。文本 LF 规则用于保留冻结文件的原始字节校验。
