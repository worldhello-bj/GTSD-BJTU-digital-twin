# 从公开 fresh checkout 复现

主 profile 已冻结。公开复现只需要当前控制台仓库正常恢复后的资产，以及 `PORTABLE_DELIVERY.json` 列出的本目录小文件；不需要私人工作路径、原照片、DWG/DXF 或 159 MB census 数据库。

1. 在 `source/cad-source-console` 执行原项目 `python prepare_assets.py`，确保原 catalog、GLB 与 `integration/data/source_replacements.json` 已恢复。
2. 重建环境安装 `requirements-rebuild.txt`。这是独立离线几何重建依赖，不要求控制台运行环境升级 NumPy。
3. 执行：

```sh
python portable_rebuild.py --project-root /path/to/source/cad-source-console --output /tmp/gtsd-fit-rebuilt
python test_portable_rebuild.py --project-root /path/to/source/cad-source-console
```

`--project-root` 必填，不回落到私有路径。脚本只从该根读已授权公开资产，只向 `--output` 写文件，不启动 Blender 或 MuJoCo，不触达硬件。

`rebuild_inputs.json` 保存 13 个相关原对象的 CAD 源位置变换、少量相邻源轴切向、完整组关系、显式设计参数和无顶点的 metadata 模板。几何从冻结 GLB 真正重新计算：连续共用支承、原尺寸扣头平移、外侧新增安装材料都由算法生成，不从已有 candidate 文件读顶点冒称重建。

本次 fresh-root 重建 2 个 profile，18 个网格的所有顶点和三角索引与冻结交付逐项完全相同。8 项公开小回归全部通过，其中包括实际主场景全量复核：1,936→1,932 显示网格，7 对非平凡 AABB 全有精确分离证据。保留最初 18 项研发检查报告，两组测试范围不同，不能将计数当新增实物数。

本重建器绑定当前 `source_replacements.json` 哈希；输入版本变化会拒绝，避免把未复核新场景默默当已验收。两个方案仍需显式选择，不默认提升为实装或物理模型。

`rail_body_review/` 是不修改主 profile 的独立后续审阅。其测量脚本同样使用 `--project-root`，只量化旧轨本体交叠，不擅自生成/安装轨道修形。
