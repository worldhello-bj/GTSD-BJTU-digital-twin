# B8 两车机械物理原型

本版保持源项目两短车/双转向架、A单电机、Z形受电弓，并将主要运动交给受力求解。含6扇源可动门、77个广义速度坐标、26个气体质量状态。

**最终状态 PASS_WITH_MODEL_LIMITATIONS：22组完整工况，273项检查通过。所有实机质量、刚度、摩擦与气路参数仍未标定；源两车拓扑本身未与竣工设备确认。此版本不代表所有零件高保真孪生或运行安全认证。**

原生接触峰值失败与800N域外失败保留。当前连续轮轨力律三步长已通过原20%峰值判据；没有删去失败、替换测量量或放宽0.15J能量账本阈值。详细数据见 RESULTS_ZH.txt、FINAL_ACCEPTANCE.md 和 results/verification.json。

## 模型与输入

- 车体/转向架/轴箱三维悬挂、弹性车钩、轮轴转动、八个气压支柱、16有质量闸片由力驱动；车轮与行进没有纯滚动运动绑定。
- 源Z弓由气缸缩回拉力、回位弹簧和闭链约束运动，实际接触刚性接触线。
- 六门由铰链转矩开启、弹簧/阻尼关闭；理想门闩只在角度和速度捕获窗内启用，没有强制改位置。
- 集总气路解有限质量、可压缩阀流量和随真实机构变化的体积；压力经几何雅可比施力。阀开度是输入，阀芯没有质量/运动状态。
- 本制动气路为直接作用执行缸支路，泄漏是缸下游支路向大气泄漏，不是铁路列车制动主管断管。没有分配阀或自动失压施闸声明，见 BRAKE_CIRCUIT_SCOPE.md。
- 断动力时输入转矩归零；电机电气部分为有界机械功率输入，未强制绑定为从弓网取电。实际电路效率/电流不在本机械验收内。

## 混合物理求解

MuJoCo3.3.7负责刚体、关节、闭链、原生闸片和弓网接触；compliant_contact.py负责实际轮体最近点的Hunt–Crossley法向力与连续化Coulomb摩擦。外部力经真实雅可比施加，保留轮速相关摩擦及反作用力矩。

主物理步长0.00005s，半步0.000025s；低黏着另有0.0001s粗步。implicitfast/Newton，默认100迭代与1e−10容差；solver_tight在加载后覆写200与1e−12。CSV100Hz，回放30Hz，与物理步长分开。

机械能账本包含动能、重力、弹性副、显式轮轨弹性储能；与电机、气动力、门转矩、阻尼、外力、原生约束和接触耗散比较。等温气体另记热浴、开口焓流、p·dV功及火用耗散。小残差仅说明离散账本闭合。

XML不能独自代表本混合物理模型。results/physics_run_identity.json绑定力律、主仿真、气路、门、弓源码和每个XML/CSV/指标/回放哈希；Blender必须读取匹配对。仿真脚本无位置/速度覆盖，只有render_full.py允许把记录姿态写入显示副本。

## 复现

Python3.11及以上，从解压后的vehicle-physics目录执行：

    python -m pip install -r requirements.txt
    python b8/test_pneumatics.py
    python b8/test_pantograph.py
    python b8/test_access_doors.py
    python b8/test_compliant_contact.py
    python b8/test_live_controls.py
    python b8/run_suite.py --workers 4
    python b8/verify_full.py
    python b8/write_report.py
    python b8/plot_full.py
    python b8/render_full.py --case service_brake

单工况：python b8/simulate_full.py --case service_brake

交互重新求解：python b8/live_viewer_full.py。图形桌面必需；macOS可用MuJoCo官方mjpython入口。控制键见 LIVE_CONTROLS.md。当前云端仅验收无窗口引擎、键映射、显示隔离；未声称GUI实际交互已测试。

脚本优先使用项目vendor目录（若存在），交付不含第三方二进制依赖，使用requirements.txt安装官方包。所有脚本只在本地仿真，不接入硬件。完整22组在本云端并行约24分钟，速度依机器而变；不会放大dt来伪装实时。

运行会覆盖results中同名输出；修改参数前先复制原结果。B7原48交付文件未变。上层build_model.py是B8需要的原始构建依赖，已随包保留。

## 判据及边界

停车要求候选时刻之后直到10秒观察窗末，A车体|速度|<0.01m/s，且至少0.5s持续。第一次过零不算停稳。单接触峰值逐积分步检查；CSV合力/采样摩擦功是100Hz量，两者不可混用。

六门刚体/理想门闩已实现；软线/软管、阀芯、压缩机内部、风扇、四个仅显隐的上盖及把手内部机构未物理化。理想齿轮、等效轴箱弹性副、代理轮轨几何、刚性车体/接触线、等温气体与未标定参数均见 COVERAGE.md。源固定螺钉/外壳可并入刚性组件，不按对象数量冒充独立机械自由度。

旧800N高位载荷导致倾覆/脱轨和过大能量误差，留作历史域外失败；不是当前HC77已验收的工况。地面是视觉背景，不解释脱轨后地面冲击。接触峰值数值收敛与真实接触刚度标定是两回事。

## 分包及媒体

A包为Blender源网格与严格回放适配；B及后续物理分包为源码、验证和完整工况数据，均是独立ZIP，解压到同一目录合并。每包有相同MANIFEST.json，精确内容以PACKAGE_VERIFICATION.json和清单为准。不是需专用工具的分卷压缩。

主视频单独交付一次，不在物理ZIP中重复收录；可由当前XML/JSON与render_full.py重建。诊断目录保留原失败与数值改进依据，正式22工况与历史/探索数据分开。
