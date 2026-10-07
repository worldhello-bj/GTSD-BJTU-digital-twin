# B10 压缩机与风机等效物理模块

B10 保留 B8 的 77 个机械广义速度坐标及 B9 的 51 个平衡计量口代理，新增一个有限质量气缸、一个压缩机机构坐标、两个风机坐标和两个热状态。整车气体质量状态由 26 增至 27。执行器输入是电压；压力、速度、活塞位移、供气量和温升由积分得到。

**新增内部结构和参数全部未标定。原 Blender 只有泵壳、泵头、风机面板/轮毂/格栅，没有可用于恢复内部机构的构件。B10 采用明确推定的单作用 Scotch-yoke 等效泵，并非认定真实设备采用该结构。** 新增部件在独立固定试验台边界内，尚未把安装反力和质量加入整车。回放保留真实来源的外壳；等效内部运动在单独标识的试验台展示。

## 物理链路

- 电机：准静态永磁直流等效，`V = R I + K ω`，`τ = K I`，SI 中 `Kt = Ke`；记录电源功、铜耗和轴功。忽略电枢电感。断电故障是开路，保留动量；连接驱动且零电压时可以电阻制动。
- 压缩机：`x = r(1−cos θ)`，`M(θ) = J + m r² sin² θ`；活塞质量对转动惯量的贡献随角度变化。气缸体积始终正值，气体质量守恒，压力通过精确等温 `∫(p−pₐ)dV` 功反馈到机构。
- 进气/排气：理想无质量压差开启单向阀，有限可压缩孔口流；不使用定额供气源。尚无阀片惯量、座碰撞、泄漏或实物阀片结构。
- 控制：储气罐压力开关有独立启停阈值及滞环，输出电压命令，不能指定转速或气压。
- 风机：有限惯量，轴承阻力及 `τ_air = c |ω|ω`；轴功传给空气的功率为 `c |ω|³`。流量代理正比转速。两个热容积分电机损耗及柜内 80 W 等效热负载，风机转速影响散热系数。
- 热容是诊断等效模型，气体仍沿用 B8 的恒温外界热浴。温升不反馈改变气体温度，风量/换热系数没有风道或实测曲线支撑。

## 积分与功的符号

压缩机采用对称离散梯度法积分一个角度及其共轭动量。周期相位保持在一周内，累计转角单独用于遥测，避免长时间运行时三角函数参数舍入影响迭代。固定气体质量时，Hamiltonian 的压力势能增量为 `−W_gas`；机械守恒式为 `ΔK = W_shaft + W_gas − Q_friction`。压缩时 `W_gas < 0`，轴必须向气体供功。孔口流与机械几何步交错，改变的是气体质量与动力学状态，不是动画轨迹。风机使用隐式中点，热容使用 Crank–Nicolson。

B8 整车 CSV 的 `gas_useful_work_J` 包含全部气室做功。B10 加入泵后，保留此原始列与原始 `coupling_work_discrepancy_J`，另外提供 `pump_gas_work_J`、`vehicle_gas_useful_work_J` 与 `vehicle_coupling_work_discrepancy_J`。整车耦合验收使用最后一列，扣除实际记录的泵气缸几何功，而非放宽 B8 的 0.05 J 阈值。

## 复现

依赖与 B9 一致：Python 3.12、MuJoCo 3.3.7、NumPy 2.2.6、Matplotlib 3.10.8。

```powershell
python source/vehicle-physics/b10/test_integration.py
python source/vehicle-physics/b10/bench.py --case all
python source/vehicle-physics/b10/simulate.py --case all
python source/vehicle-physics/b10/verify.py
python source/vehicle-physics/b10/report.py
python source/vehicle-physics/b10/live.py --headless --settle-s 0 --duration .02
```

完整验收包括 6 个 120 s 辅机试验台工况，3 个整车工况（6 s 自然沉降 + 10 s 观察），16 项组件测试及 36 项控制/冻结 B8 回归。验收文件绑定实际运行源文件和结果 SHA-256。B8/B9 历史验收仍按原版本保留，不把它们计作 B10 全部工况复测。

交互窗口沿用 B8 按键；新增控制有 Python API：`set_auxiliary_voltage(name, fraction)` 与 `set_auxiliary_power_failure(name, failed)`，`name` 为 `compressor`、`compressor_fan` 或 `bay_fan`。压缩机手动电压 API 会切换到手动模式。检查阀只能由压差开启，不能由操作员直接设定开度。

## Blender 与交付包

完整交付为 `artifacts/B10/GTSD_BJTU_B10_auxiliary_dynamics.zip`，包含本次全部数值轨迹、源码、原始源模型与已保存 Blender 工程；`PACKAGE_VERIFICATION.json` 逐项记录大小、SHA-256 和 ZIP CRC，`UNPACKED_SMOKE.json` 记录新目录中实际运行的验收与交互启动检查。B8 的原始四包和 B9 的历史增量包保留。

打开 `source/vehicle-physics/b10/replay/assets/GTSD_BJTU_B10_source_dynamics.blend`：`B10_MECHANICAL_REPLAY` 为整车，`B10_PANTOGRAPH_CLOSEUP` 为受电弓，`B10_INFERRED_AUXILIARY_BENCH` 为推定辅机试验台。后者展示 0.5 秒已求解启动轨迹，以 33.3 倍慢速显示；没有速度表达式驱动。工程重读检查覆盖 301 帧整车姿态、84 个源门件、501 帧辅机轨迹与 7 个活动节点。

本次从独立执行快照计算，运行起止核对力学源码哈希。复现时可先执行 `python source/vehicle-physics/b10/freeze_execution.py --version b10-local`，再从该快照运行工况；已存在且源码不同的快照拒绝覆盖。首次创建快照包含当时全部源码，后续新增非力学交付脚本不改变已完成工况的身份。

## 依据与推定边界

- [Atlas Copco：往复式压缩机](https://www.atlascopco.com/en-eg/compressors/reciprocating-piston) 支持变容积吸气、压缩、排气的一般原理。
- [EFRC：Compressor valves](https://www.recip.org/tutorial-compressor-valves/) 支持吸排气阀由压差驱动的一般拓扑；B10 将阀片简化为无质量单向座。
- [MathWorks：DC Motor](https://www.mathworks.com/help/simscape-electrical/ref/dcmotor.html) 支持电枢电压、电流、转矩、反电势及功率关系。
- [美国能源部：Fan system sourcebook](https://www.energy.gov/sites/default/files/2014/05/f16/fan_sourcebook.pdf) 支持定几何下风量与速度、功率与速度的相似律。B10 的阻力和换热系数仍是未标定输入。

这些资料仅用于选择等效物理关系，**均未提供本项目真实压缩机、风机的型号、内部结构或参数**。Scotch-yoke、缸径、行程、惯量、电机常数、孔口面积、压力开关、风量系数及热参数均属推定。要提升为设备数字孪生，需要实物机构图、电机/泵/风机曲线及动态压力、转速、温度数据。
