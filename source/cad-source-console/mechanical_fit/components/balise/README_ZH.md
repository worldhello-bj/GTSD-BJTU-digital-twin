# 应答器枕间安装与扣头外置配合候选

这是可选、可回退的工程安装候选，不是实装认定。来源轨道、原生轨枕轴和 CAD 扣件均未移动或改形。两套应答器各六件、两组源位椅/推定扣头各三件一起替换，18 删 18 增；没有新增实体数量、刚体、质量或碰撞。

## 可直接集成

- `balise_fit_profile.json`：精确 `remove_names` 与 `add_meshes`。顶点为世界 XYZ 米单位平铺数组，三角索引为平铺数组，`group=environment`。
- `balise_installation_contract.json`：来源组、目标 bay、刚体矩阵、尺寸、轨侧净距及扣头就座参数。
- `fit_config.json`：两套组件的 bay、正交 yaw、横向中心、枕侧/轨侧/扣头间隙参数。默认关闭，保留照片推定标签。
- `current_main_effective_scene_validation.json`：实际主包五场景的完整有效资源复核，先 base GLB+17 extras，再按场景应用 323→326 来源替换，最后加本候选。四个场景 1936 件，plan 场景 1937 件。18 个被替换名全部存在；无旧/新重复。
- `balise_fit_validation.json`：原三对内部重叠见证、XYZ 材料弦、候选后实际三角复核及刚体不变式。
- `balise_fit_before_after.png`：由真实网格生成的平面前后对照，未嵌入原照片。

独立 profile 必须原子应用，不单独移动黄盖、壳体、螺钉或恢复其中一个扣头。`additional_mutually_exclusive_names` 防止同时放入旧待决源位变体；它们在当前 main 中本就不在有效集合，不属于额外隐藏对象。

## 真实照片和原生 CAD 复查

实际查看了 `应答器.jpg` 与图片 15、17、18。近照中的灰壳位于相邻枕间，底部下降到台面附近；黄盖高于轨枕，存在盖缘外伸。照片不支持把它直接抬到枕顶。四张照片没有唯一识别模型编号 02/03、原生 station、90°方向或毫米尺寸的标尺。

重新扫描全部 205,442 原生 records 的原始 tags，包括 1,099 个 block 定义和 32 个 DIMENSION；检索应答器/应答/balise/transponder/eurobalise/rfid/信标。同时检查 270 条已抽出的注释文本、实例库存和登记表来源。没有找到能唯一绑定这两套对象的文字证据。轨区三个未标注矩形 4EDFE/4EDFF/4EE00 也重新核过，不能凭外形认领为这两套应答器。这不是“全 CAD 不存在未命名图样”的结论。

`balise_source_reassessment.json` 保存检索范围、照片观察、Git blob/内容 hash 和来源 commit；没有原照片字节。小型 `balise_source_seed.json` 保存源位纯平移合同、25 条有关轨边/枕轴记录及原三对材料见证，足够从已恢复的公开主包重建候选，无需大 SQLite 或私有目录。

## 候选参数和几何结果

1. 应答器 02：选源 station 11/12 间 bay，中心移至 X=-7.195666523 m、Y=0.148508672 m；相对原显示中心平移约 (-84.33315, +148.50867, 0) mm，并整体绕 Z 转 90°。
2. 应答器 03：选源 station 16/17 间 bay，中心移至 X=-6.195666523 m、Y=0.098822087 m；中心变化约 (-14.33346, +98.82209, 0) mm，整体绕 Z 转 90°。
3. 壳体仍为约 122×116×60 mm，没有缩放。90°方向使 116 mm 短边进入约 120 mm 源枕间净距，壳体每侧约 2.00004 mm。零度方向在这个 bay 会被参数门槛拒绝。盖缘高于枕顶，可外伸而不把二维投影重叠误判为穿料。
4. 原 Z 顶点逐点不变，壳体仍位于继承台面高度。整个壳/盖/四螺钉同一刚体矩阵。整个组件对两侧源轨三角形的保守横向间隙：02 约 7.90803 mm，03 约 57.59461 mm。
5. 恢复两组源位椅后，主包复核额外发现两个正侧旧扣头进入轨底。现把四个旧推定扣头按实际有限轨底三角形边界外移：12_2 最大 1.731833 mm，17_2 最大 1.561377 mm。形状不变，轨侧参数间隙为 0.5 mm，椅边保守余量最小 0.768168 mm。每个扣头所有不同 XY 顶点均通过真实椅材料中截面的内部测试，不仅用 AABB 宣称受承托。

0.5/1/5 mm 这些门槛是公开可改的工程检查参数，不是规范值、制造公差或已测尺寸。扣头外置安装并未证明螺杆孔位、夹紧接触或预紧。

原三对材料弦 XYZ（mm）：
- 支椅17_2／应答器03：7.055979、30.352972、6.284902
- 负扣头12_2／应答器02：10.000229、0.965773、16.000003
- 正扣头12_2／应答器02：0.162790、1.685644、16.000003

这些是内部点沿 XYZ 射线到双方真实三角面的共有材料长度，不是 AABB 深度、最小分离平移或穿透体积。候选后这三对在 AABB 阶段已几何分开。

最终有效主包五个场景中，本候选的全部改变件对全场检查：0 新增正材料重叠，0 未核清 after 对。剩余 16 个正 AABB 疑点经过实际三角面六向边测试、共面 SAT、闭合检查及顶点包含检查，均分离。两处椅顶/轨底的 float32 共面序列化差约 2.38 nm，使用 10 nm 数值接触容差，未用毫米级容差掩盖干涉。此结论只覆盖本候选改变件；不宣称整车所有旧代理从此无穿料。

## 可公开复现（仅 NumPy）

先按主项目说明恢复 baseline GLB、scene_extras 和 source_replacements 等资产。下列 `PROJECT_ROOT` 指包含 `baseline_console` 与 `integration` 的 `source/cad-source-console`，不需要原照片或 census 数据库。

```sh
python build_balise_candidate.py --project-root PROJECT_ROOT
python verify_main_effective_scene.py --project-root PROJECT_ROOT
python test_balise_candidate.py --project-root PROJECT_ROOT
```

不同参数请复制 `fit_config.json`，指定 `--config CONFIG --output-dir OUTPUT`。输出目录只写生成物，不改项目资源。横向位置超出有限源轨走廊、未转向而壳体宽于枕间净距、扣头投影离开原椅材料、非零抬升或缩放企图都会拒绝。若照片/实测证明另一安装，应通过明确参数及来源更新重新评估，不改轨/枕躲避冲突。

如需重新生成示意图，额外安装 matplotlib，再运行 `render_balise_fit.py --project-root PROJECT_ROOT`。该图不是验收门槛，验证不依赖 matplotlib。

## 父程序集成后交叉检测接口

`balise_geometry.py` 均为只读纯计算接口：

```python
scenes, identity = load_effective_scenes(project_root)
for scene_name, before in scenes.items():
    after, new_to_old = apply_profiles(before, profiles)
    report = validate_changed_meshes(before, after, new_to_old)
```

`profiles` 可同时含本候选与其他机械 profile，真正检查不同 profile 的新件彼此干涉。每个新增网格要有 `metadata.replaces_mesh_name`，与实际删除旧名对应。一个旧件拆成两个新件时，同源新件之间不会错误地用“旧件与自身重叠”免除检查。结果同时报告继承的旧重叠；不能只看 `passes_no_new_interference_gate` 就宣称绝对无干涉。

也可调用 `verify_main_effective_scene.py --project-root PROJECT_ROOT --profile A.json --profile B.json --output combined.json`。它会拒绝缺组、重复选择或一个旧对象被多个 profile 替换。

## 尚未确认

- 应答器 02/03 的确切物理身份、bay、方向、型号尺寸和制造公差
- 安装支架、底部固定、孔位、接触和预紧
- 线缆/检修空间、电磁安装要求和车辆动态限界
- 轨枕/椅截面及整个试验台的真实结构载荷路径

照片和 CAD 各自能确定的内容没有互相冒充。候选可用于显示、比较和进一步实测核验，不能自动接入碰撞、质量或真实控制硬件。
