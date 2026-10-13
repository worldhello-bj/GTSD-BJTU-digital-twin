# GTSD 来源整机与虚拟研究控制台

## 传输中草稿：远端资源尚未齐备

当前公开分支仍在补齐资源，尚不能视为可独立运行的完整交付。完整本地冻结快照的验收结果不等于远端 checkout 已可运行；具体传输进度、缺失资源和阻塞项请读取 [TRANSFER_STATUS.json](TRANSFER_STATUS.json)。下面保留的是资源齐备后的运行设计、启动命令与研究边界。

本目录保存面向完整交付的公共源码与生成模型快照（2026-10-13）。完整快照包含以下内容，远端是否已齐备以传输状态为准。包含完整最新装配视图、strict82 来源轨道/近侧弹簧座两个物理变体、独立受电弓 H1 候选、44 个来源审阅状态，以及完整 1,932 网格原生 Blender 整机。只监听回环地址，不接真实硬件。

## 资源齐备后的启动方式

Python 3.12。运行时版本固定为 MuJoCo 3.3.7、NumPy 2.2.6。Node.js 22 或更新版本仅供测试（本次用 24.19.0 复验）；Blender 不是控制台运行依赖。

```sh
cd source/cad-source-console
python3.12 -m venv .venv
# Linux / macOS
. .venv/bin/activate
# Windows PowerShell 改用：.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python doctor.py
python launch.py --port 8778
```

在运行服务的同一台电脑打开 http://127.0.0.1:8778/assembly 。Ctrl-C 关闭。双击 HTML 不会启动服务。若端口被占用，改为 `--port 8779`。

近侧弹簧座是显式的未标定研究模式，单独启动：

```sh
python launch.py --port 8778 --cad-seat-research
```

`launch.py` 和 `doctor.py` 都会自动从仓库里的二进制分块恢复所需资源，无需手工拼接、Git LFS、外部下载或旧工作盘。`prepare_assets.py` 只用 Python 标准库，逐块、整段、解压后分别检查 SHA-256；资源已存在但被修改时拒绝覆盖。首次恢复会写生成资源，之后检查不修改它们。doctor 构造并精确恢复零步状态，不积分仿真、不创建硬件连接。

```sh
python prepare_assets.py              # 单独自动恢复
python prepare_assets.py --check-only # 全只读验证，缺资源即报错
```

仓库内使用普通二进制分块，现存分块最大 4 MiB；本次增量交付的新分块每块不超过 300,000 字节。生成资源约 175 MB，分块约 82 MB（十进制），另有普通源码、合同和实渲图。57 个恢复资源中包括 9 张生成 PNG；运行 `doctor.py` 或 `launch.py` 时会恢复到原路径，无需手工拼接。解压后按 `.gitignore` 忽略生成的大文件和这些 PNG；不要提交恢复副本。

## 页面

- `/`：完整设备控制台，共用 strict82 会话
- `/assembly`：真实装配源网格、逐件来源/绑定、运行/暂停/单步及来源替换开关
- `/pantograph-candidate`：独立 H1 拓扑试验，默认禁用，不沿用整车状态
- `/source-library`：44 个互斥来源、解释、拓扑清理或冲突对照状态
- `/source-audit`：2,738 个 Model 顶层对象索引
- `/source-dimensions`、`/source-drive`、`/source-fixture`：轴承包络、40 件驱动对照、变压器配套来源

## 实际包含的原生整机与渲染

运行 `python prepare_assets.py` 后直接用 Blender 打开：

`integration/source_assembly_native/GTSD_Source_Assembly_Revision.blend`

SHA-256：`08b61aa81c1a3b51c54488b413aaae5072013c51ca6afcd32c20168b7bb9eac2`。

保存的原生回读报告：1,932 网格、1,656,604 三角形，每个 Float32 顶点与索引序列相等。完整长来源名保存在对象自定义属性 `source_name`；原生 ID 显示名可能截断。

以下是此同一修订的实际 Blender 渲染，不是概念效果图：

![完整静态来源整机](integration/source_assembly_native/whole_static_revision.png)
![轨道轴线顶视](integration/source_assembly_native/track_axis_top.png)

本快照提供运行与审阅所需的编译后生成资产。原始 DWG/DXF、课件 PDF、照片、私有笔记、完整审计包和原始逐步轨迹不随仓库发布。因此不提供需要这些私有输入的重新抽取/重建脚本。受电弓候选构造器是运行依赖，所需输入已随包提供。冻结基线历史文档/构建脚本保留原字节，仅作来源记录，不代表那些历史重建工作流已在本公共包提供全部输入。

## 验收

```sh
python doctor.py
node --test integration/tests/test_*.js
python run_acceptance.py
```

`run_acceptance.py` 自动恢复资源，在 `acceptance-output/` 写本次证据并运行：6 项恢复安全测试、doctor、41 个 Node 测试、来源页面只读 HTTP、两个整车模式各三次原生单步及完整 checkpoint 恢复/续算、独立候选联合路由。它不重写冻结基线或历史证据。测试应在无其它本目录控制台运行的环境执行。核心正常启动不依赖 Node。

公共源码自带 `PUBLIC_MANIFEST.json`；冻结 `baseline_console/GTSD_Assembly_Console/BUNDLE_MANIFEST.json` 的 289 项文件恢复后逐字节不变，原有求解器身份不重新签署。历史 `/workspace/shared/...` 常量是代码来源，portable 加载器在运行时使用当前目录，不能为“清理路径”而改写这些冻结源文件。公共包 fresh-copy 复验结果见 `PUBLIC_ACCEPTANCE.json`。浏览器 WebGL/鼠标交互不由 HTTP/Node 通过推出。

## 必须保留的研究边界

- 新几何不等于新碰撞。326 个静态显示替换件不自动成为新的质量、接触或物理关节；可视通孔不是求解器接触孔。
- 默认来源轨道变体仅更新两轨代理的范围/顶面宽度/中心；40 mm 厚度与整车其余碰撞、质量仍是未标定继承参数。
- 近侧座模式只更新两组近侧弹簧端点和 restlength，保持初始轴向预载；Y 安装、刚接、弹簧率与质量未实测，远侧不照搬近侧投影。
- 受电弓 H1 是 5 关节、2 闭环的独立候选；每体 1 kg、零重力、无接触仅作拓扑试验，关键连接仍有图纸歧义。
- 1,932 是保守默认显示网格数；开启 66 个零厚来源参照后是 1,998 个网格/参照，不等于独立实物数。来源分母/分类不等于 CAD 全部建模或整机 1:1 完成。
- 短步恢复验收与已保存的 20 ms 数值研究不证明长期收敛、载荷准确、实机安全或实时性能。原始长程结论不自动继承到新几何。

发布范围和扫描结果见 `PUBLIC_PRIVACY_REPORT.json`；资产和原生身份见 `ASSET_MANIFEST.json`、`integration/source_assembly_native/NATIVE_MANIFEST.json`。
