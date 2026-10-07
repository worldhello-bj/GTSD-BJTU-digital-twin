"""Plot accepted B11 dynamics and document exact convergence/attachment values."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from verify import read_csv

ROOT=Path(__file__).resolve().parent


def main():
    verification=json.loads((ROOT/'results/verification.json').read_text(encoding='utf-8'))
    if not verification['passed']:
        raise RuntimeError('full B11 acceptance required for result report')
    names=('service_brake','half_dt','spatial_refinement','tether_release','doors_open_close','doors_half_dt')
    metrics={n:json.loads((ROOT/'results'/(n+'_metrics.json')).read_text(encoding='utf-8')) for n in names}
    data={n:read_csv(ROOT/'results'/(n+'.csv')) for n in names}
    fig,axes=plt.subplots(3,2,figsize=(12,11),layout='constrained')
    colors=dict(service_brake='#126782',half_dt='#f18f01',spatial_refinement='#48954b',tether_release='#9e2a2b',doors_open_close='#126782',doors_half_dt='#f18f01')
    for n in names[:4]:
        a=data[n]
        axes[0,0].plot(a['time_s'],a['speed_mps'],label=n,color=colors[n])
        force=sum(a['cable_supply_'+str(i)+'_attachment_force_N'] for i in range(3))
        axes[0,1].plot(a['time_s'],force,label=n,color=colors[n])
        gap=np.maximum.reduce([a['cable_supply_'+str(i)+'_attachment_error_m'] for i in range(3)])
        if n=='tether_release':
            gap=np.where(a['time_s']<2,gap,np.nan)
        axes[1,0].plot(a['time_s'],1000*gap,label=n,color=colors[n])
        axes[2,0].plot(a['time_s'],a['mechanical_balance_residual_J'],label=n,color=colors[n])
    for n in names[4:]:
        a=data[n]
        axes[1,1].plot(a['time_s'],np.degrees(a['door_cabinet_PWR_angle_rad']),label=n,color=colors[n])
        axes[2,1].plot(a['time_s'],a['cable_bond_PWR_attachment_force_N'],label=n,color=colors[n])
    labels=[('Vehicle speed','m/s'),('Sum of supply attachment force magnitudes','N'),
            ('Maximum active supply attachment gap','mm'),('Power cabinet hinge angle','degrees'),
            ('Mechanical work balance residual','J'),('Power cabinet bonding attachment force','N')]
    for ax,(title,unit) in zip(axes.flat,labels):
        ax.set(title=title,xlabel='Observation time (s)',ylabel=unit); ax.grid(alpha=.2); ax.legend(fontsize=7)
    fig.suptitle('B11 source-anchored cable equivalents\nUncalibrated material/contact parameters; six settled force-driven cases',fontsize=13)
    for suffix in ('png','pdf'):
        fig.savefig(ROOT/'results'/('b11_cable_response.'+suffix),dpi=160)
    plt.close(fig)
    checks={r['name']:r for r in verification['checks']}
    lines=['# B11 线缆等效模型验收结果','',f"状态：**{verification['status']}**。六组完整工况与 {len(checks)} 项检查通过；每组自然沉降 6 s，再观察 10 s。",'',
           '| 工况 | 广义速度坐标 | 停车距离 m | 最终速度 m/s | 最大机械残差 J | 最大动量残差 N·s |',
           '|---|---:|---:|---:|---:|---:|']
    for n in names:
        m,a=metrics[n],data[n]; distance='—' if m['stop_distance_m'] is None else f"{m['stop_distance_m']:.9f}"
        lines.append(f"| {n} | {m['dofs']} | {distance} | {m['final_speed_mps']:.8g} | {max(abs(a['mechanical_balance_residual_J'])):.8g} | {max(abs(a['momentum_balance_residual_Ns'])):.8g} |")
    lines += ['', '## 接触几何范围', '',
              '| 工况 | 回放姿态数 | 采样最大穿入 mm | 采样最大穿入/半径 | 观察逐步最大穿入 mm | 观察逐步最大穿入/半径 |',
              '|---|---:|---:|---:|---:|---:|']
    for n in names:
        geometry = verification['replay_contact_geometry'][n]
        observed=metrics[n]['cable_contact_geometry_all_steps']['observation']
        peak_depth=f"{1000*observed['maximum_penetration_m']:.6g}"
        peak_ratio=f"{observed['maximum_penetration_radius_ratio']:.6g}"
        lines.append(f"| {n} | {geometry['recorded_poses']} | {1000*geometry['maximum_sampled_cable_penetration_m']:.6g} | {geometry['maximum_sampled_penetration_radius_ratio']:.6g} | {peak_depth} | {peak_ratio} |")
    lines += ['', '观察阶段每个原生积分位置均检查全部线缆接触，记录最大穿入/最小接触线缆半径；位置计数和完整时间范围同时核对。使用实际 XML 对每个约 30 Hz 回放姿态独立重算几何，逐步峰值须覆盖采样峰值。表中最大深度与最大半径比可能来自不同接触。端点间隙另采用 100 Hz CSV 采样。软接触穿入属于未标定的数值代理，不能解释为实物压缩量；本次没有证明接触力峰值或连续体解收敛。', '',
              '源参考姿态的中间服务线段存在初始重叠，最深约 8.24 mm。自然沉降的全部积分位置也记录接触范围；它是数值预处理，没有声称验证了这一重叠初态下的实机启动。完整观察从自然沉降后的求解状态开始，未覆盖或强制设定机械轨迹。旧版正常制动的端点/接触越界及两组被替代的未完成细化单独保存为拒收历史。']
    lines += ['',f"基准模型有 {metrics['service_brake']['cable_configuration']['total_links']} 个有限质量链节、五条物理路线，替换源模型八个曲线对象。线缆总质量 {metrics['service_brake']['cable_configuration']['total_cable_mass_kg']:.9f} kg，单独列入系统，原整车质量 265.904 kg 保留。",'',
              f"时间步长减半后停车距离相对变化 {100*checks['halfstep_stop']['detail']['relative_change']:.6g}%，空间链节上限减半后变化 {100*checks['spatial_stop']['detail']['relative_change']:.6g}%。各自限值为冻结 B8 的 2.5% 与新增空间细化的 5%；数值收敛不代表实物准确度。",'',
              '接头误差按各路线半径的十分之一检查。内部球铰间隙、单链节长度和固定端漂移逐回放帧核对；释放供电端只停用约束，释放后的连接反力归零。', '',
              '车体动量账本包含全部原生根平动约束反力，包括线缆牵引与受电弓接触。系统机械账本包含链节惯性、被动关节和原生接触的实际功；不把固定线缆支承归入整车质量。','',
              '密度、EI、阻尼及摩擦未标定；刚性链节没有轴向拉伸，球铰弯扭为各向同性代理。未求解电气连续性、软管压强—截面耦合、真实连接器失效或实物阀/泵/锁内部结构。B10 辅机继续属于独立固定试验台。']
    (ROOT/'RESULTS.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('Wrote accepted B11 plots and RESULTS.md')


if __name__=='__main__':
    main()
