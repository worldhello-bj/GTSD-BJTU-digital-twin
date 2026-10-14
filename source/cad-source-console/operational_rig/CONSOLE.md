# 装配控制台操作

## 启动

从 `source/cad-source-console` 运行：

```sh
python -m pip install -r requirements.txt
python launch.py --operational-rig --port 8176
```

在运行服务的同一电脑打开 http://127.0.0.1:8176/operational-rig 。资源从仓库分块自动校验恢复，运行不需要 Blender。

## 完整运动与控制

1. 点击“开始”，加载默认装配并运行完整预定运动。
2. “暂停”停在完整积分步边界；再次“开始”继续剩余时程。
3. “单步”前进一步；“运行”执行所填步数。接口也支持 10、50 步批次。
4. 暂停后保存检查点，可在同一会话恢复，或导出为 JSON 留档。
5. “重启”回到本会话初始状态。参数页修改质量、刚度、摩擦或驱动设置后，重新开始会建立新的模型会话。
6. 发生失联时刷新状态；页面不会自动重发运动命令。

“轮轨、夹线、销、整机”切换镜头。模型按实际位移显示，数值卡同时显示微小位移。程序到达末端后可保存结果或重启重放。

“运行”每次最多 40,000 原生步，单会话最多 100,000 步或 5 秒模拟时间。页面显示实际模拟时间；墙钟时间取决于本机求解速度。

## 参数

页面的“参数”入口集中列出默认值、单位和来源。保存草案不会立即改动物理模型；下一次“开始”采用新参数建立新身份。

端接间隙属于构造参数。修改 `photo_turnout/operational_revision/parameters.json` 并用同目录构建与验证命令重新生成几何；动力学会读取新网格并产生新身份。质量、刚度、摩擦、销直径和索线基准等运行参数可直接在网页修改。

## 本地验证

```sh
python -m unittest discover -s operational_rig/tests -p 'test_*.py' -v
npm ci --prefix operational_rig
npm test --prefix operational_rig
python operational_rig/validate_protocol.py --output /tmp/gtsd-operational-check
python operational_rig/verify_http_native.py --complete-demo --output /tmp/gtsd-operational-http
```

接口有 generation、session、revision 和 request ID 检查，重复点击不会重复积分。数值异常会恢复到整条命令开始前的原生状态及能量账；用“重启”或已有检查点继续。

HTTP 验证会真实发出完整演示命令、等待原生时程完成，再导出、前进一步并恢复终点检查点。它与网页 DOM 回归分别验证传输和操作逻辑。仓库 CI 还从分块恢复全部资源，重跑原生完整运动及能量/接触门控。
