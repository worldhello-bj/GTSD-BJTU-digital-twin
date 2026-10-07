"""Build fact-only B9 summary and exported numerical comparison figure."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
OUT = ROOT/"results"


def main():
    v = json.loads((OUT/"verification.json").read_text(encoding="utf-8"))
    if not v["passed"]:
        raise RuntimeError("acceptance must pass before publishing a summary")
    names = ("service_brake", "half_dt", "slow_brake_valves", "brake_coil_failure", "normal_exhaust")
    metrics = {n: json.loads((OUT/(n+"_metrics.json")).read_text(encoding="utf-8")) for n in names}
    arrays = {n: np.genfromtxt(OUT/(n+".csv"), names=True, delimiter=",") for n in names}
    traces = {n: np.genfromtxt(OUT/(n+"_spools.csv"), names=True, delimiter=",") for n in names}
    colors = {"service_brake":"#147d92", "half_dt":"#294963", "slow_brake_valves":"#d07829", "brake_coil_failure":"#bc3947", "normal_exhaust":"#777b93"}
    labels = {"service_brake":"Nominal", "half_dt":"Half step", "slow_brake_valves":"High damping", "brake_coil_failure":"Supply coil failure", "normal_exhaust":"Normal exhaust"}
    plt.rcParams.update({"font.family":"DejaVu Sans", "font.size":10, "axes.spines.top":False, "axes.spines.right":False, "axes.titleweight":"bold", "figure.facecolor":"#fafaf7", "axes.facecolor":"#fafaf7"})
    fig, axes = plt.subplots(2, 2, figsize=(11,7), layout="constrained")
    for n in ("service_brake", "slow_brake_valves", "brake_coil_failure"):
        a, tr = arrays[n], traces[n]
        port = next(k.removesuffix("_opening") for k in tr.dtype.names if "_pad_supply_opening" in k)
        axes[0,0].plot(tr["network_time_s"]-6, tr[port+"_opening"], label=labels[n], color=colors[n], lw=2)
        axes[0,1].plot(a["time_s"], (a["brake_pressure_Pa"]-101325)/1000, label=labels[n], color=colors[n], lw=2)
        axes[1,0].plot(a["time_s"], a["speed_mps"], label=labels[n], color=colors[n], lw=2)
    axes[0,0].set(xlim=(3.98,4.18), ylim=(-.03,1.03), title="Force-driven supply metering", xlabel="Controlled time (s)", ylabel="Actual opening fraction")
    axes[0,1].set(xlim=(3.98,5.0), title="Pressure responds to actual area", xlabel="Controlled time (s)", ylabel="Brake gauge pressure (kPa)")
    axes[1,0].set(xlim=(0,10), title="Coil failure does not stop the vehicle", xlabel="Controlled time (s)", ylabel="A vehicle speed (m/s)")
    for n in ("service_brake", "half_dt"):
        a = arrays[n]
        axes[1,1].plot(a["time_s"], a["mechanical_balance_residual_J"], color=colors[n], label=labels[n], lw=1.6)
    axes[1,1].axhline(.15, color="#bc3947", ls="--", lw=1, label="Unchanged B8 limit")
    axes[1,1].axhline(-.15, color="#bc3947", ls="--", lw=1)
    axes[1,1].set(xlim=(0,10), ylim=(-.16,.16), title="Mechanical energy ledger", xlabel="Controlled time (s)", ylabel="Balance residual (J)")
    for ax in axes.flat:
        ax.grid(alpha=.15); ax.legend(frameon=False, fontsize=8)
    fig.suptitle("GTSD BJTU / B9 dynamic metering ports", fontsize=18, weight="bold")
    fig.supxlabel("Uncalibrated balanced-port prototype / 77 vehicle coordinates + 26 gas masses + 102 spool states", fontsize=9)
    fig.savefig(OUT/"b9_valve_response.png", dpi=180)
    fig.savefig(OUT/"b9_valve_response.pdf")
    plt.close(fig)
    m,h = metrics["service_brake"], metrics["half_dt"]
    table = ["| 工况 | 停车距离 m | 制动后停稳 s | 末速 m/s | 机械账本峰残差 J |", "|---|---:|---:|---:|---:|"]
    for n in names:
        q = metrics[n]; a = arrays[n]
        distance = "未停稳" if q["stop_distance_m"] is None else f'{q["stop_distance_m"]:.6f}'
        delay = "—" if q["stopped_after_brake_s"] is None else f'{q["stopped_after_brake_s"]:.3f}'
        table.append(f'| {labels[n]} | {distance} | {delay} | {q["final_speed_mps"]:.6f} | {max(abs(a["mechanical_balance_residual_J"])):.6f} |')
    text = f'''# B9 本次验收结果

状态 `{v["status"]}`。19 项阀芯模块检查、35 项控制/旧 B8 回归检查通过；五组完整 B9 工况的 {len(v["checks"])} 项验收检查通过。本次没有重跑 B8 原 22 组全部工况。

{chr(10).join(table)}

主步长 0.05 ms，半步长 0.025 ms。正常制动停车距离相邻差 {100*abs(h["stop_distance_m"]/m["stop_distance_m"]-1):.6f}%，原 2.5% 阈值保留。停车要求随后至 10 s 窗末 |A 车体速度| < 0.01 m/s，持续至少 0.5 s。

五工况空气质量最大残差 {max(q["max_air_mass_residual_kg"] for q in metrics.values()):.3e} kg；阀芯能量账本采样峰残差 {max(q["max_sampled_spool_balance_residual_J"] for q in metrics.values()):.3e} J。机械账本使用原 0.15 J 阈值，气机械功差使用原 0.05 J 阈值，动量使用原 0.05 N·s 阈值；全部零 MuJoCo 告警。

正常制动和阀芯迟滞的停车差、失效后仍运行、正常排气与快排差别均来自求解结果。没有指定位姿、规定轮滚动关系或强制停车。完整数字与哈希见 results/verification.json 和各工况 identity JSON。

新增的 51 个压力平衡端口代理有 102 个位置/速度状态，计量孔口跟随实际阀芯，不等于 51 个实物阀。名义参数、外部固定支承及剩余覆盖限制见 README.md。新增阀芯参数、气路和源设备均未实测标定；软线/软管、压缩机内部和风扇仍待物理化。

![阀芯、压力、车速和守恒对比](results/b9_valve_response.png)
'''
    (ROOT/"RESULTS.md").write_text(text, encoding="utf-8")
    print(text)

if __name__ == "__main__":
    main()
