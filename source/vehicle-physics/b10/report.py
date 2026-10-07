"""Publish measured numerical outputs and standalone comparison figure."""
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from auxiliaries import ROOT
from verify import read_csv


def main():
    v = json.loads((ROOT/"results/verification.json").read_text(encoding="utf-8"))
    if not v["passed"]:
        raise RuntimeError("physics acceptance must pass")
    names = ("charge", "half_dt", "power_loss", "outlet_blocked", "fan_power_loss", "pressure_switch")
    bench = {n:json.loads((ROOT/"bench_results"/(n+"_metrics.json")).read_text(encoding="utf-8")) for n in names}
    data = {n:read_csv(ROOT/"bench_results"/(n+".csv")) for n in names}
    train = {n:json.loads((ROOT/"results"/(n+"_metrics.json")).read_text(encoding="utf-8")) for n in ("service_brake", "half_dt", "compressor_power_failure")}
    plt.rcParams.update({"font.family":"DejaVu Sans", "font.size":10, "axes.spines.top":False,
                         "axes.spines.right":False, "figure.facecolor":"#fafaf7", "axes.facecolor":"#fafaf7"})
    fig, axes = plt.subplots(2,3,figsize=(14,8), layout="constrained")
    colors = dict(charge="#147f93", half_dt="#617388", power_loss="#c54c4a", outlet_blocked="#bc8a28", fan_power_loss="#c54c4a", pressure_switch="#147f93")
    for n in ("charge", "half_dt", "power_loss", "outlet_blocked"):
        a = data[n]
        axes[0,0].plot(a["network_time_s"], (a["reservoir_pressure_Pa"]-101325)/1000, label=n.replace("_"," "), color=colors[n], lw=1.5)
        axes[0,1].plot(a["network_time_s"], a["delivered_air_kg"]*1000, label=n.replace("_"," "), color=colors[n], lw=1.5)
    a = data["pressure_switch"]
    axes[0,2].plot(a["network_time_s"], a["reservoir_pressure_Pa"]/1000, color=colors["charge"], label="0.2 L bench")
    axes[0,2].axhline(601.325, color="#c54c4a", ls="--", label="Switch off threshold")
    for n in ("charge", "fan_power_loss"):
        a = data[n]
        axes[1,0].plot(a["network_time_s"], a["bay_fan_omega_rad_s"], color=colors[n], label=n.replace("_", " "))
        axes[1,1].plot(a["network_time_s"], a["cabinet_temperature_K"]-293.15, color=colors[n], label=n.replace("_", " "))
    for n in ("service_brake", "compressor_power_failure"):
        a = read_csv(ROOT/"results"/(n+".csv"))
        axes[1,2].plot(a["time_s"], a["speed_mps"], label=n.replace("_", " "), color=colors["charge"] if n == "service_brake" else colors["power_loss"])
    specs = [("Pressure from piston cycles", "Absolute bench time (s)", "Reservoir gauge pressure (kPa)"),
             ("Finite delivered air", "Absolute bench time (s)", "Delivered mass (g)"),
             ("Pressure switch is a torque input", "Absolute bench time (s)", "Reservoir absolute pressure (kPa)"),
             ("Fan coasts after power loss at 10 s", "Absolute bench time (s)", "Fan speed (rad/s)"),
             ("Cooling responds to actual speed", "Absolute bench time (s)", "Cabinet rise above ambient (K)"),
             ("Full coupled vehicle", "Controlled vehicle time (s)", "Vehicle speed (m/s)")]
    for ax, (title, xlabel, ylabel) in zip(axes.flat, specs):
        ax.set(title=title, xlabel=xlabel, ylabel=ylabel); ax.grid(alpha=.15); ax.legend(frameon=False, fontsize=8)
    fig.suptitle("GTSD BJTU / B10 force-driven auxiliary equivalents", fontsize=18, weight="bold")
    fig.supxlabel("Inferred fixed bench / uncalibrated / isothermal gas / DC motors / diagnostic thermal model", fontsize=9)
    fig.savefig(ROOT/"results/b10_auxiliary_response.png", dpi=180); fig.savefig(ROOT/"results/b10_auxiliary_response.pdf"); plt.close(fig)
    table = ["| 120 s 试验台工况 | 储气罐绝压 kPa | 排入储气罐 g | 柜内温升 K |", "|---|---:|---:|---:|"]
    for n in names:
        r = bench[n]["final"]
        table.append(f'| {n} | {r["reservoir_pressure_Pa"]/1000:.3f} | {r["delivered_air_kg"]*1000:.6f} | {r["cabinet_temperature_K"]-293.15:.3f} |')
    normal, half = train["service_brake"], train["half_dt"]
    trtable = ["| 6+10 s 整车工况 | 停车距离 m | 末速 m/s | 储气罐末绝压 kPa |", "|---|---:|---:|---:|"]
    for n, r in train.items():
        dist = "未停稳" if r["stop_distance_m"] is None else f'{r["stop_distance_m"]:.6f}'
        trtable.append(f'| {n} | {dist} | {r["final_speed_mps"]:.6f} | {r["final"]["reservoir_pressure_Pa"]/1000:.3f} |')
    text = f'''# B10 数值验收

`{v["status"]}`：16 项组件测试、36 项控制及未修改 B8 回归通过；3 个完整整车工况和 6 个 120 s 辅机工况的 {len(v["checks"])} 项验收通过。B8/B9 历史验收保持原版本记录。

{chr(10).join(table)}

{chr(10).join(trtable)}

整车主步长 0.05 ms、半步长 0.025 ms；停车距离相邻差 {100*abs(half["stop_distance_m"]/normal["stop_distance_m"]-1):.6f}%，沿用 B8 2.5% 阈值。辅机试验台步长 0.2/0.1 ms，120 s 排气质量相邻差 {100*abs(bench["half_dt"]["final"]["delivered_air_kg"]/bench["charge"]["final"]["delivered_air_kg"]-1):.6f}%。三组整车零 MuJoCo 告警。

泵机械能采样峰残差 {max(max(abs(a["pump_mechanical_balance_residual_J"])) for a in data.values()):.3e} J；试验台质量末残差最大 {max(abs(m["gas_audit"]["mass_balance_residual_kg"]) for m in bench.values()):.3e} kg。电能、气体内能/火用、风机轴功和热容守恒分别核验，详细阈值和哈希见 verification.json。

供气和散热故障改变流量、气压和温升，不修改转子动量、气体质量或温度。理想检查阀、恒温气体、忽略电感、固定安装边界与未标定系数的限制见 README.md。Blender 中额外试验台使用 0.5 s、1 kHz 已求解启动轨迹，33.3 倍慢放；源模型泵壳与格栅不被当作活动机构。

![压缩机供气、风机和整车响应](results/b10_auxiliary_response.png)
'''
    (ROOT/"RESULTS.md").write_text(text, encoding="utf-8"); print(text)


if __name__ == "__main__":
    main()
