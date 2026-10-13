# GTSD 组合机械候选独立干涉审计

## 结论

组合候选的六个夹具目标配合已消除正体积穿料；轨支承与应答器的变更件，在当前完整保留场景及互相叠加中，没有发现新增正体积干涉。但这不等于全模型无干涉，也不证明安装、连接、制造公差或承力路线成立。

本次直接读取默认场景 20_FRONT_ROUTE_NOMINAL_FIT 的世界坐标三角网格：1932 个原网格移除 37、加入 33，组合后共 1928 个，其中 1895 个不变。变更件对所有保留件及所有变更件互相，共检查 63,063 个唯一 pair；原 37 个前身对完整原场景另检查 70,781 个唯一 pair。先完整 AABB，再对所有正 AABB 候选进行三角相交、共面接触、包含检查。

- 34 个组合后超过数值容差的正 AABB 候选：27 对三角实体分离；2 对确认真实正体积交叠；5 对进入接触/数值边界复核。
- 324 对跨 profile 的变更件配对，均无正体积 AABB 交叠。变更件内部仅两个正 AABB 候选，都是夹具销/B535 两对，三角检查均分离。
- 另单列41对AABB边界候选。其中导出双精度坐标有6处支承/轨型Z方向约0.000002384186 mm（2.384 nm）的薄层，低于主容差；转换为Blender float32后六处厚度均为0。它们不属于34个超容差窄相候选，完整具名记录保留，且不把AABB边界等同于真实接触。
- 33 个新增网格全部闭合、边二次关联、面向一致、无退化三角形。
- 对导出双精度坐标和 Blender float32 坐标各独立重跑，34 对分类一致。此次没有启动 Blender，也没有改变仿真或源模型。
- 未发现新增的正体积交叠证据。仍有零间隙、继承交叠和未定连接，不能无条件通过严格全机净空验收。

## 原“7组17网格7穿料对”的口径

这七对属于历史 source-position candidate 的冲突视图。独立读取七份候选三角资源，七对均复现正体积内部证据。它们在当前1932网格默认旧位场景中，七对本来都是 AABB 分离。因此不能描述为“当前默认场景的七处穿料现在全部清零”。

1. DETAIL_BENCH_branch_chair_03_1 / DETAIL_BENCH_crossing_support_plate：合并为 FIT_TRACK_station03_common_support。
2. DETAIL_BENCH_branch_chair_17_2 / ENV_balise_body_03：组合后分离，X投影间隔约3.50017 mm，Y约65.01710 mm。
3. DETAIL_BENCH_branch_chair_23_2 / ENV_rail_chair.046：合并为 FIT_TRACK_station23_common_support。
4. DETAIL_BENCH_branch_fastener_01_0_-1 / DETAIL_BENCH_guard_diverging：组合后Y投影间隔约65.99551 mm。
5. DETAIL_BENCH_branch_fastener_12_2_-1 / ENV_balise_body_02：组合后X投影间隔约36.99989 mm，Y约104.92628 mm。
6. DETAIL_BENCH_branch_fastener_12_2_1 / ENV_balise_body_02：组合后X投影间隔约36.99989 mm，Y约19.46264 mm。
7. DETAIL_BENCH_crossing_support_plate / ENV_rail_chair.007：合并为 FIT_TRACK_station03_common_support。

第1、3、7对变成单一设计支承件，不能宣称“两件仍独立且实现净空”。合件制造、连接方式及负载路径仍待工程确认。上述是投影间隔，不是全表面最短距离。

## 六个夹具目标

- 518DF 与 518E0 的4079杆，各自对环及下夹块：四对原先有独立内部证据，组合后AABB已严格分离，正体积交集为0。
- 50DF4 与51409 的销/B535：原交集分别约9.24817369、9.24610305 mm³；组合后两对均为0，实际三角端面最小间距分别约0.49904009、0.49892493 mm。

对非均匀源零件，没有无条件沿用“整个零件均匀拉伸”假设。补充截面校验若不能确认端面等价，保留较保守的内部点/AABB证据，不把未经复证的完整体积数字当作独立结果。

## 仍存在的真实交叠

- ENV_contact_wire / FIT_518DF_4079_DEPTH_UNRESOLVED_ROUND5_CANDIDATE：38.18368006 mm³。
- ENV_contact_wire / FIT_518E0_4079_DEPTH_UNRESOLVED_ROUND5_CANDIDATE：38.18342368 mm³。

两对前后体积完全相同，且有独立绕数、距表面距离和三轴内部弦证据。4079仅移动深度，无法据此确认下端是焊接、夹紧、销接还是源模型简化；不得作为已验证连接或无条件通过项。

作为对已知继承问题的额外点查，ENV_messenger_wire 对两侧 CAD_SOURCE_518DF/518E0_C4D1_4AEC_jaw_lower_square_fit_candidate 各保留约0.000106906083 mm³的微交叠。主6件夹具profile没有移动整根承力索；这里未把可选承力索变体混入结果。这两对不在本次“变更件对全场景”的穷举范围内，明确另列，不代表全部未变件已检查。

## 五对接触/数值边界复核

1. CAD_50DF4_F643_profile_0 / FIT_50DF4_F643_NOMINAL_PIN5_8A2C_CANDIDATE。
2. CAD_51409_F643_profile_0 / FIT_51409_F643_NOMINAL_PIN5_8A2C_CANDIDATE。
   两对均为名义零间隙销/槽接口。三角网格存在接触，完整穿料见证未出现；补充截面法在声明的数值公差下得到0体积。不能据此保证生产装配公差。
3. CAD_50DF4_F643_profile_3 / FIT_50DF4_B535_OUTLINE_8A2C_HOLE25_VERSION_CANDIDATE。
   前后相同的浮点薄交叠：Y厚度0.000014901161 mm，体积0.0002384337565 mm³。该数值不能被隐藏为绝对0，也不宜等同于实体制造碰撞。
4. FIT_50DF4_B535_OUTLINE_8A2C_HOLE25_VERSION_CANDIDATE / REPAIR_518DD_88AC_1_source-plan6.5。
5. FIT_51409_B535_OUTLINE_8A2C_HOLE25_VERSION_CANDIDATE / REPAIR_518DE_88AC_1_source-plan6.5_FRONT_ROUTE_REFLECTED_CANDIDATE。
   两对条板/舌槽板的实际截面积分前后均为0，保留名义接触，无正净空证据。

五对在当前默认原场景中已有对应的接触/边界问题，float32重跑没有新增分类。三角碰触次数受三角剖分改变影响，不是干涉恶化量化指标。

## 方法及边界

- 主筛选数值容差：1e-8 m，即0.00001 mm。AABB排除只是几何排除；正AABB不等于穿料。
- 窄相为三角AABB、六条边/面相交、共面2D分离轴、双向顶点包含和闭合性检查。无法严格排除的接触保守记录。
- 正体积样点另用立体角绕数以及真正点到三角形的距离独立确认。报告中的内切球体积仅是正交集的保守下界，AABB体积只是上界，两者都不是完整交集体积估算。
- 截面补充使用真实三角边/面截线与GEOS多边形；截面同一性检查采用0.00001 mm Hausdorff容差及周长缩放面积界。截线坐标仅在1e-9 mm处规范化，零体积仍需区别于零间隙。
- 穷举范围不包括1895个未变网格互相的1,794,565对。没有全机自交、源轨型拓扑、运动扫掠、载荷、电气或制造验收。
- 4079原先与其他杆/管的相交在移动后消失，只能证明几何分离，不证明原连接被合理替代。需要连接定位和固定方式证据。

## 文件

- combined_interference_audit.json：导出坐标的完整逐pair前后证据、身份哈希及范围。
- native_float32/combined_interference_audit.json：Blender量化坐标的独立重复结果。
- numerical_boundaries_export.json、numerical_boundaries_float32.json：41对数值边界具名记录及低于主容差的薄层。
- after_pairs.csv、before_pairs.csv：便于筛选的具名前后结果索引。
- interface_refinement.json：原七对三态对照、六目标、五复核与继承交叠的截面补证。
- audit_combined.py、triangle_classifier.py：只读主审计及已审阅的三角检查器。
- refine_interfaces.py、cap_geometry.py、geos_kernel.py：只读截面补证脚本。
- test_audit.py：闭合、接触、包含、绕数及距离的解析回归测试。

脚本仅需Python、NumPy及已安装的GEOS C库。主脚本参数为 --source、--combined、--profiles、--replacements、--output；可加 --float32。截面脚本参数为 --source、--combined、--audit、--source-library、--output。使用实际交付的相对文件位置填入参数即可，不需要用户本机、原始照片或新仿真。

## 从公开主仓库一键复跑

推荐唯一入口：python reproduce.py --project-root <cad-source-console公开仓库根目录> --output <新的报告目录>。

入口调用仓库内 mechanical_fit/export_scene.js 和 profiles.json，自动在临时目录重建source/combined输入，顺序执行完整双精度检查、float32重复检查、接口精化与7项解析回归，最后自动清理临时网格。交付包不保存source全量几何，不要求保留任何临时native_input目录；不写主仓库、不启动Blender或仿真。需要Python、NumPy、Node.js及系统GEOS C库。可选 --blend <已交付blend文件> 仅增加文件哈希，不打开Blender。
