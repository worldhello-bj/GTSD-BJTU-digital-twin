# strict82 可移动软件短验收

本目录是独立验收与最小依赖提取，不覆盖任何已冻结交付目录。

## 运行

将整个本目录复制到任意可读路径，使用已安装 Python 3.12、numpy 2.2.6、MuJoCo 3.3.7 的解释器。从本目录或任意其他工作目录运行：

```sh
python /新路径/portable_console.py verify --audit
python /新路径/portable_console.py doctor --audit --port 0
python /新路径/portable_console.py serve --audit --port 8767
```

只绑定 127.0.0.1。serve 创建模型后初始暂停。端口0由系统选择，读终端打印的实际地址。Ctrl-C关闭服务。不自动导入检查点。

短命令入口始终显式选择 strict82；被冻结的默认75配置文件仍是 `portable/GTSD_Phase6/console_strict82/console_config.portable.json`，没有静默换variant。库不在本目录内，若doctor显示版本不符，应选择符合版本的既有解释器，本验收没有安装或打包新软件。

## 目录和字节身份

- `portable/GTSD_Phase5_Internal_Coupling/`：实际运行需要的旧Phase5源码与资产
- `portable/GTSD_Phase6/console_strict82/`、console_fan82、console_phase7、console_phase6：原冻结控制台文件
- `portable/GTSD_Phase6/strict82/`：严格数值入口及原SOURCE_FREEZE
- `portable/GTSD_Phase6/trusted_rollback/`：原trusted_rollback.py
- `portable/GTSD_Phase6/auxiliary/`、cad_body、cad_doors、geometry、truth：运行所需固定依赖与只读证据
- `portable/PORTABLE_MANIFEST.json`：逐文件SHA-256、字节数和原来源。223个原文件共32,517,957字节，无symlink；只复制不改写
- `tools/materialize_portable.py`：本次来源提取记录；这是生产者重建工具，消费端运行不依赖原路径
- `tests/check_portable_http.py`：独立完整短验脚本，不修改冻结源

原严格bootstrap.py含云绝对路径，保留原字节并纳入固定hash，但未执行它。运行使用原strict82_server.py已有的独立path/admission shim。数值源码、原生MRO、checkpoint身份与显示身份边界不变。

## 复验

```sh
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  timeout 260s python tests/check_portable_http.py
node portable/GTSD_Phase6/console_strict82/tests/check_dom.js
```

完整短验最多8次单步原生积分，脚本内部墙钟上限240秒；普通checkpoint候选构建/验证远慢于这几步积分。它使用真实回环HTTP服务器，记录HTTP状态与命令事件。用于失联测试的外层显示对象在一次成功原生step后等待5.15秒，以避免五秒租约内大量积分；原控制器clock、默认5秒租约和原生数值方法都未改。该测试等待不进入任何生产launcher。

复验会在evidence覆盖同名测试产物；保留本轮证据时请先复制整目录到新的验收路径。不得从这些短验推断实时性、长时稳定性、半步长收敛或硬件标定。

## 审计范围

完整短验从/tmp工作目录运行，导入前安装Python audit hook，拦截open、os.listdir与os.scandir。拒绝读取本验收目录与现成Python环境之外的/workspace/shared路径；系统Python/numpy/MuJoCo是正常库，允许读取。对原冻结文件进行了运行前后SHA复核。启动依赖只读发现独立记录在discovery_startup.json，它不是可移动验收。

审计是Python访问证据，不是整个操作系统的native syscall隔离证明；解释器初始化发生在hook安装前，期间没有导入项目模块。MuJoCo native库内部syscall不由Python audit完整枚举。不能称为完全OS沙箱验证。

## 明确未验

- 真实浏览器/GPU操作或画面
- 新整机装配的动力学/碰撞或几何真值升级
- 旧命令全集、所有非默认参数、半步长UI身份
- 长物理工况、长期稳定性、全状态收敛、硬件标定
- 默认75的再次物理推进

首次基础Python调用因缺MuJoCo且numpy版本不同被doctor正确拒绝，随后选择现成冻结环境。运行中曾遇执行器transport断开；早期中断尝试没有完整逐步日志，不能断言其精确累计积分次数。本目录的通过范围只以具有完整native_calls、HTTP与最终result的记录为准，不能把成功轮的步数当作所有中断尝试的总步数。

## 本轮已完成的结果

完整成功轮 exit=0，160.70秒，5次实际原生积分调用（上限8）：

- 新路径冷启动steps=0且paused；run/pause/step真实HTTP生效
- 重复同request_id只执行一次；同ID改参数、过期session与非法命令拒绝
- 14个HTTP负例；对应请求与状态见evidence/http_transcript.json
- 重新签署checksum的坏原生identity、两种历史速度篡改均拒绝；当前检查点和谱系不被失败恢复污染
- 完整checkpoint导出、精确恢复、恢复后再推进与原分支完整状态精确相等
- 上一条的“精确”采用完整JSON解码结构==，无数值容差；未另记录canonical字节比较，不称逐字节验收
- 默认5秒失联租约实测触发暂停，该测试段只1次积分；外层限步等待已在上文披露
- 观测端点不续租、不改变checkpoint
- 212个移位原文件运行前后SHA不变，原来源文件也复核未变；旧项目目录读取尝试0
- 8项Node渲染/client逻辑通过，是真实执行Node测试，不是浏览器DOM/GPU验收

实际运行使用现成`/workspace/shared/gtsd-bjtu-digital-twin/.venv-console/bin/python`，从`/tmp`运行独立tests/check_portable_http.py。数字身份保持 `9bd3b5a7f26a40c44f1b77f5c71b9ec53488e682170ddfda651a939aa96323db`，几何身份保持 `2b6314a577bc67c5781268d797c57be6674ef16d42eed9f47731238cfc299fed`。

汇总见ACCEPTANCE_SUMMARY.json；完整证据包括portable_http_result.json、http_transcript.json、command_results.json、native_calls.jsonl、audit_reads.json、relocated_doctor.json、launcher_doctor.json、dom_result.json和两个checkpoint。零步HTTP geometry/binding/state额外保存用于只读显示映射；本轮没有单独保存步后body_poses，不能把完整检查点当作该显示帧文件。

可移动目录另补齐已由原candidate_review.py列出的独立候选审阅PNG/JSON资产，均原字节SHA固定。此静态补齐不启动模型，没有扩大前述212文件HTTP成功轮的测试声明；新增资产的HTTP/浏览器路由未重验，记录见evidence/candidate_asset_completion.json。
