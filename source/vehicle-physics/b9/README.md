# B9 阀芯动力学增量

本版接续 B8 的两车物理模型，将供气和排气端口从“指令直接给开度”改为“线圈力驱动阀芯，实际位移决定开口面积”。B8 冻结源码、273 项旧验收和四个原包保留。B9 是未标定的工程原型。

## 实现范围

- 每个等效计量端口有阀芯质量、回位弹簧、黏性阻尼、有限行程软止挡及遮盖量。输入是 0–1 线圈力比例，没有指定阀芯轨迹或瞬时开度。
- 分段二次势能采用离散梯度积分；阀芯动能、弹簧/止挡储能、线圈功和阻尼耗散单独审计。
- 实际阀芯行程经遮盖量映射为有效孔口面积，再接入 B8 的有限质量、可压缩气路。泄漏仍为故障孔口；压缩机仍为 B8 集总源。
- 两车系统保留 77 个广义机械速度坐标与 26 个气体质量状态；51 个独立等效计量端口新增 102 个阀芯位置/速度状态。这是计算拓扑，不能当作实物有 51 个阀。
- 阀芯断电后通过原质量和弹簧回闭；线圈失效仅撤去驱动力。高阻尼故障通过阻尼改变响应，不强行冻结或改写状态。
- `simulate.py` 和 `live.py` 对 B8 作独立模块导入及气路工厂注入；没有修改 B8 文件，没有改写整车 qpos/qvel。

## 参数与边界

名义阀芯质量 0.01 kg、刚度 1800 N/m、阻尼 12 N·s/m、行程 2 mm、遮盖量 0.2 mm、最大线圈力 4.5 N、止挡刚度 100000 N/m。高阻尼工况使用 80 N·s/m。全部为 `CALIBRATION_ASSUMPTION`，没有目标设备阀芯尺寸/型号的实测依据。

本次采用压力平衡 2/2 端口代理：不平衡有效面积和气体扫掠容积为零，不计算流致力、密封摩擦、磁路电流和真实多口阀的机械联锁。阀芯由固定实验台支承，质量不重复计入原 265.904 kg 车辆；其支承反力没有耦合进车体动量。气路仍为直接作用制动执行缸支路，没有自动失压施闸声明。

源模型没有可信的活动阀芯内部网格，本次不会把旧 ADD 骨骼当成新阀芯。Blender 回放显示 B9 求出的整车姿态，阀芯数值在 CSV 和文本数据块中。软线/软管、压缩机曲柄/活塞、风扇叶轮等仍是后续缺项。

## 复现

Python 3.11+，在仓库或增量包的 `source/vehicle-physics` 目录运行：

```powershell
python -m pip install -r b9/requirements.txt
python b9/test_valve_dynamics.py
python b9/test_integration.py
python b9/simulate.py --case all
python b9/verify.py
python b9/report.py
```

五组完整工况为正常制动、0.025 ms 半步、16 个制动供气线圈失效、高阻尼制动阀、受电弓正常排气。每组先受力积分沉降 6 s，再观察 10 s；主步 0.05 ms、CSV 100 Hz、回放 30 Hz。五工况保留原 B8 0.15 J 机械账本、0.05 J 气机械耦合、0.05 N·s 动量与 2.5% 停车距离收敛阈值。

本次仅验证这五组 B9 工况；B8 原 22 组不是本次重新运行的 B9 验收，未声明低黏着或新增阀模型的全工况认证。结果覆盖范围和数值见 [验收结果](RESULTS.md)、[机器可读验收](results/verification.json)、[因果对比图](results/b9_valve_response.png)。

交互使用相同气路：

```powershell
python b9/live.py
python b9/live.py --headless --action brake_apply --duration-s 0.2
```

按键沿用 B8：W 动力、S 惰行、B 施闸、N 松闸、R 升弓、E 正常排气、Q 快排、O 开门、K 关门、P 暂停。程序 API `vehicle.set_valve_coil_failure(port, True)` 设置线圈失效。验证了无窗口控制与接线；图形交互不属于本次验收。

## 输出与身份

每工况有机械 CSV、阀芯 CSV、XML、回放 JSON、指标 JSON 和身份 JSON。阀芯 CSV 的 `network_time_s` 包含前 6 s 沉降；机械 CSV 的 `time_s` 从受控观察开始计时。身份 JSON 绑定 B8/B9 实际力律源码、数值库版本与输出字节哈希。XML 本身没有阀芯/外部接触律，不能单独用来复现混合物理。

运行覆盖同名 B9 输出；修改参数前备份。`verify.py` 会检查输出/源码哈希，并核对 B8 冻结身份。根目录 `.gitattributes` 固定文本 LF，防止 Windows 换行转换破坏 B8 校验。

Blender 工程和完整数值结果包含在增量 ZIP 中。打开 `source/vehicle-physics/b9/replay/assets/GTSD_BJTU_B9_source_dynamics.blend`，选择 `B9_MECHANICAL_REPLAY` 或 `B9_PANTOGRAPH_CLOSEUP`，时间轴 30 fps。回放适配器的坐标命名沿用 B8，文件、场景标题和求解身份标记为 B9。

重新生成源网格回放时，增量包已包含所需的只读原模型和适配器。在完整仓库中，先按 `restore_b8.py` 恢复原包，将 A 包解压到仓库 `local-replay-base/`，再使用 Blender 5.2：

```powershell
blender --background --factory-startup --disable-autoexec --python-exit-code 1 --python source/vehicle-physics/b9/build_blender_replay.py
blender --background --factory-startup --disable-autoexec --python-exit-code 1 --python source/vehicle-physics/b9/validate_blender_replay.py
```

本次 Blender 保存重读检查遍历全部回放帧；它检验显示矩阵和身份，不增加实机物理精度声明。
