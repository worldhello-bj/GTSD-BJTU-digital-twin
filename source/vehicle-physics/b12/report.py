"""Plots and numeric results for accepted cover workbench, with explicit scope."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from verify import read_csv

ROOT=Path(__file__).resolve().parent
acceptance=json.loads((ROOT/'results/verification.json').read_text(encoding='utf-8'))
if not acceptance['passed']:raise RuntimeError('cover acceptance required')
names=('fastened','drop','drop_half_dt','handle_remove','handle_half_dt')
metrics={n:json.loads((ROOT/'results'/(n+'_metrics.json')).read_text(encoding='utf-8')) for n in names}
arrays={n:read_csv(ROOT/'results'/(n+'.csv')) for n in names}
fig,axes=plt.subplots(2,2,figsize=(11,8),layout='constrained')
for name in names:
    a=arrays[name]
    axes[0,0].plot(a['time_s'],a['cover_0_z_m'],label=name)
    axes[0,1].plot(a['time_s'],a['floor_normal_force_N'],label=name)
    axes[1,0].plot(a['time_s'],a['mechanical_balance_residual_J'],label=name)
    axes[1,1].plot(a['time_s'],a['cover_0_angular_speed_rad_s'],label=name)
for ax,(title,unit) in zip(axes.flat,[('First cover center height','m'),('Floor reaction, 200 Hz display','N'),('Mechanical work balance','J'),('First cover angular speed','rad/s')]):
    ax.set(title=title,xlabel='Time (s)',ylabel=unit); ax.grid(alpha=.2); ax.legend(fontsize=7)
fig.suptitle('B12 inferred fixed maintenance bench\nSource shapes; four unmeasured 0.25 kg proxies; no vehicle installation claim',fontsize=12)
for suffix in ('png','pdf'):fig.savefig(ROOT/'results'/('b12_cover_response.'+suffix),dpi=160)
plt.close(fig)
lines=['# B12 可拆检修盖独立试验台验收','',f"**{acceptance['status']}**：5 个完整 2 秒工况、5 项组件测试、{len(acceptance['checks'])} 项检查通过。",'',
       '| 工况 | 步长 µs | 最大机械残差 J | 最大动量残差 N·s | 每步检查的地面总反力峰 N | 最大穿入 mm |',
       '|---|---:|---:|---:|---:|---:|']
for name in names:
    m=metrics[name]; a=arrays[name]
    lines.append(f"| {name} | {1e6*m['dt_s']:.0f} | {m['maximum_mechanical_residual_J']:.9g} | {max(a['momentum_balance_residual_Ns']):.6g} | {m['peak_floor_normal_force_N']:.6f} | {1000*m['maximum_contact_penetration_m']:.6f} |")
checks={r['name']:r for r in acceptance['checks']}
lines+=['',f"落下工况半步长反力峰变化 {100*checks['halfstep_peak_drop']['detail']['relative_change']:.6f}%，偏心力提起/落下工况变化 {100*checks['halfstep_peak_handle_remove']['detail']['relative_change']:.6f}%；预设限值 5%。释放四个固定约束时，位置和速度变化均为零，随后保留自由惯性。",'',
        '图中接触力为 200 Hz 显示；表中峰值在每个积分步检查，避免把稀疏显示曲线当冲击峰。', '',
        '两块源 B5 盖板在启用隐藏集合并重评估骨骼后仍位于世界原点；其真实装配位置未确认。因此本版采用明确推定的四工位独立试验台，保留源形状，不接入整车质量与反力。','',
        '四件均赋予未实测的 0.25 kg 代理质量，以源包围盒作均匀惯量及接触；散热鳍片源网格只用于显示。固定夹具是可停用的数值六轴约束，没有源铰链、螺钉拆卸或手部模型。','',
        '接触为未标定软代理，容许穿入为最薄代理厚度的一半；上述千牛峰值只属于该代理，不能用于预测真实盖板/地板冲击载荷。2 ms 夹具试验未满足固定误差阈值，历史失败保留；夹具收紧到 0.5 ms 后仍按原阈值验证。']
(ROOT/'RESULTS.md').write_text('\n'.join(lines)+'\n',encoding='utf-8',newline='\n')
print('Wrote accepted B12 numeric plots and report')
