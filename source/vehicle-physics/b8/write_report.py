"""Rebuild fact-only final handover and source/input identity from checked outputs."""
from pathlib import Path
import json,csv,hashlib
import numpy as np
from simulate_full import CASES
ROOT=Path(__file__).resolve().parent;OUT=ROOT/'results'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 v=json.loads((OUT/'verification.json').read_text());assert v['passed'],v['status']
 M={n:json.loads((OUT/f'{n}_metrics.json').read_text()) for n in CASES}
 A={n:np.genfromtxt(OUT/f'{n}.csv',names=True,delimiter=',') for n in CASES}
 s=M['service_brake'];f=M['half_dt'];lc=M['low_adhesion_coarse_dt'];lm=M['low_adhesion'];lf=M['low_adhesion_half_dt'];fa=M['flange_disturbance'];fb=M['flange_half_dt'];da=M['doors_open_close'];db=M['doors_half_dt']
 dd=next(x['detail']['max_angle_difference_rad'] for x in v['checks'] if x['name']=='doors_halfstep_trajectory')
 mx=lambda n,k:float(max(abs(A[n][k])))
 pct=lambda a,b:100*abs(b/a-1)
 text=f'''B8 源配置两车机械物理原型：最终验收

状态：{v['status']}。{len(M)}组完整工况，{len(v['checks'])}项检查全部通过。
这是未实测标定的工程原型。主系统受力驱动与“所有零件高保真孪生”不是同一结论。历史失败保留，未改写原20%轮缘峰值和0.15J机械账本判据。

本轮实际完成
1. 两独立车体/弹性车钩、双转向架三维悬挂、四轮轴、八个空气弹簧；A单电机转子/理想2:1传动，B无动力。共77个广义速度坐标，闭链和理想约束会减少独立自由度。
2. 2L有限储气、16制动执行腔、8空气弹簧、1受电弓缩回腔，共26个气体质量状态。可压缩阀流量、临界流、止回、排气、快排、泄漏及实际容积功。阀开度是控制输入，尚无阀芯动力学。
3. 源单臂Z弓/两平行四边形闭链、被动同步约束、后置缩回缸及单边弓网接触；未替换成菱形弓。
4. 源4车门+2柜门有独立质量/惯量/铰链、开门力矩、关门弹簧/阻尼与软止挡；捕获角度/速度满足条件才挂理想门闩，无强行改位姿。84个源门部件与求解器刚体绑定。
5. 轮轨采用连续Hunt–Crossley法向力和正则化Coulomb摩擦，盘片/弓网仍用MuJoCo原生接触。实际接触雅可比和轮轴转速产生力矩；没有纯滚动轨迹约束、峰值滤波或保底轮重。

源一致性和质量预算
源项目两短车各一转向架保留，但源两车本身来自项目假设，未证实竣工设备实际编组。源轮径0.28m、轮缘外径0.298m、轮距0.478m、Z杆长0.16m等仅是来源几何，不冒充实测。
车上质量{s['total_mass_kg']:.3f}kg；两扇世界固定柜门另计12kg，不计入轮重。A车4门5kg从原60kg中分出，残余车体55kg；闭态质量、重心和完整惯量重构通过独立测试。
B7原48交付文件保持原哈希。基准和集成检修共同XML：{s['model_xml_sha256']}。
XML不独自定义混合求解模型，results/physics_run_identity.json另绑定Python力律/仿真/气路/门/弓源码、参数、CSV和回放哈希。

基准及因果性
- 4s制动阀开启时速度{s['speed_at_brake_start_mps']:.6f}m/s；持续停车距离{s['stop_distance_m']:.6f}m，延迟{s['stopped_after_brake_s']:.3f}s。
- 停车要求之后直至10s观察窗末|A车体速度|<0.01m/s，至少持续0.5s；悬挂回弹第一次过零不算停车。
- 关动力位移{M['power_off']['travel_m']:.8f}m；惰行末速{M['coast']['final_speed_mps']:.6f}m/s。
- 去掉闸片摩擦末速{M['brake_mu_zero']['final_speed_mps']:.6f}m/s；服务储气无初压末速{M['no_supply_pressure']['final_speed_mps']:.6f}m/s，弓缸无升弓行程/接触。
- 加60kg负载后制动开始速度{M['loaded_brake']['speed_at_brake_start_mps']:.6f}m/s；制动缸下游支路泄漏末速{M['brake_cylinder_branch_leak']['final_speed_mps']:.6f}m/s。
- 松闸排气再给转矩末速{M['brake_release']['final_speed_mps']:.6f}m/s。单空气弹簧泄漏产生最大侧滚{np.rad2deg(M['air_spring_leak']['max_roll_rad']):.4f}°。
- 弓头外力扰动可分离并恢复；快排面积增大带来真实更快降压/折叠，不改变动画时长。
- 制动是直接作用执行缸供气模型，不是减压自动施闸的铁路列车制动主管。没有证据支持已实现分配阀/断管紧急制动，详见BRAKE_CIRCUIT_SCOPE.md。

同一77坐标模型数值收敛
主步长0.05ms，半步0.025ms；低黏着另外保留0.1ms粗步对照。
- 基准停车距离差{pct(s['stop_distance_m'],f['stop_distance_m']):.6f}%；弓网单点峰差{pct(s['peak_collector_single_contact_N'],f['peak_collector_single_contact_N']):.4f}%。
- 低黏着轮缘单点法向峰：0.1/0.05/0.025ms分别{lc['peak_flange_single_contact_N']:.6f}/{lm['peak_flange_single_contact_N']:.6f}/{lf['peak_flange_single_contact_N']:.6f}N；相邻变化{pct(lc['peak_flange_single_contact_N'],lm['peak_flange_single_contact_N']):.4f}%和{pct(lm['peak_flange_single_contact_N'],lf['peak_flange_single_contact_N']):.4f}%，原20%判据通过。
- 轴箱侧载轮缘峰{fa['peak_flange_single_contact_N']:.4f}→{fb['peak_flange_single_contact_N']:.4f}N，差{pct(fa['peak_flange_single_contact_N'],fb['peak_flange_single_contact_N']):.4f}%；至少7个踏面接触，分别{fa['wheel_partial_contact_steps']}/{fb['wheel_partial_contact_steps']}步部分卸载，未人为禁用卸载。
- 六门最大开角：车门约1.3114rad，柜门约1.9216rad；0.05/0.025ms全轨迹最大差{dd:.6f}rad。最终六门全部被动回闭并挂闩。
- Newton迭代/容差收紧到200/1e−12，基准末速差{abs(M['solver_tight']['final_speed_mps']-s['final_speed_mps']):.3e}m/s。全部工况零MuJoCo告警。

守恒与被动性
- 全22组空气质量最大残差{max(m['max_air_mass_residual_kg'] for m in M.values()):.3e}kg。
- 基准机械能+显式接触弹性储能账本峰残差{mx('service_brake','mechanical_balance_residual_J'):.6f}J；半步{mx('half_dt','mechanical_balance_residual_J'):.6f}J。
- 六门峰残差{mx('doors_open_close','mechanical_balance_residual_J'):.6f}J；半步{mx('doors_half_dt','mechanical_balance_residual_J'):.6f}J。
- 轴箱侧载峰残差{mx('flange_disturbance','mechanical_balance_residual_J'):.6f}J；半步{mx('flange_half_dt','mechanical_balance_residual_J'):.6f}J。
- 全组最大机械账本残差{max(mx(n,'mechanical_balance_residual_J') for n in CASES):.6f}J，全部低于未改动的0.15J线。
- 基准、六门、轴箱侧载外力冲量—整车动量峰差分别{mx('service_brake','momentum_balance_residual_Ns'):.6f}/{mx('doors_open_close','momentum_balance_residual_Ns'):.6f}/{mx('flange_disturbance','momentum_balance_residual_Ns'):.6f}N·s，半步继续降低。低黏着粗/主步数据保存于新增动量观察器之前，不对这两份补造动量账本；细步数据有独立动量检查。
- 基准初始总垂向轮轨力{A['service_brake']['rail_force_world_z_N'][0]:.4f}N，对应列车重力{s['total_mass_kg']*9.81:.4f}N。固定柜门质量没有错误加到轮重。
- 基准轮轨摩擦累计功{A['service_brake']['contact_friction_work_J'][-1]:.6f}J，低黏着约{A['low_adhesion']['contact_friction_work_J'][-1]:.3f}J，均耗散；法向阻尼累计功同样非正。低黏着大损耗来自真实轮速与低车速的差，不是删除电机功。
- 气体按等温热浴记内能、开口焓流、p·dV功和换热；小残差验证离散账本，不是绝热守恒或实机能量精度认证。

历史失败与当前适用域
原生接触241→444N（84.1%）及加门后的157→269N（约70.8%）峰值失败均保留。10ms原生候选不能作为77坐标终态。旧800N高车体载荷倾覆/脱轨且能量误差过大，属于旧模型的适用域外失败，未在当前HC77上宣称通过。连续法当前0.1ms基准残差约0.192J也保留，改用0.05ms后通过，没有放宽原门槛。
连续力律2e6N/m、40s/m、0.005m/s是有文献形式依据的等效柔性/耗散/正则化假设。新旧峰值比较同一A前右轮缘单接触量，但力律已改变，15.5N不是对241N的测量校正；必须经过实物加载/接触测试才能解释绝对峰值。低黏着原生弓网最大软穿入{lm['max_contact_penetration_m']*1000:.3f}mm，不能解读为真实钢材变形。

未物理化和理想化项
软线/软管无质量/张力/弯曲/地面接触；阀芯/手阀/调压旋钮无独立受力运动；压缩机曲柄/活塞及风扇叶轮无机械模型；四个仅显隐检修盖固定；把手/锁扣内部动作由理想门闩替代。齿轮为理想约束、轴箱转臂为等效弹性副、轮轨型面为代理几何、车体和接触线刚性。没有完整悬链线、齿隙/齿面接触、材料应力或真实设备参数标定。
详细逐类覆盖见COVERAGE.md。控制器无窗口测试通过，实际GUI未在云端无显示环境交互验收。没有控制任何硬件，不提供运行安全认证。
'''
 dd=next(x['detail']['max_angle_difference_rad'] for x in v['checks'] if x['name']=='doors_halfstep_trajectory')
 (ROOT/'RESULTS_ZH.txt').write_text(text)
 (OUT/'metrics.json').write_text(json.dumps([M[n] for n in CASES],indent=2))
 with (OUT/'case_summary.csv').open('w') as fh:
  keys=['case','dt_s','dofs','total_mass_kg','speed_at_brake_start_mps','final_speed_mps','stop_distance_m','stopped_after_brake_s','max_lateral_m','max_roll_rad','max_contact_penetration_m','minimum_wheel_treads_in_contact','wheel_partial_contact_steps','peak_flange_single_contact_N','peak_collector_single_contact_N','max_air_mass_residual_kg']
  w=csv.DictWriter(fh,fieldnames=keys);w.writeheader();w.writerows({k:m[k] for k in keys} for m in M.values())
 sources=['full_model.py','simulate_full.py','compliant_contact.py','pneumatics.py','pantograph.py','access_doors.py','live_viewer_full.py','parameters.json']
 identity={'schema':'B8 hybrid-physics source/input identity v1','scope':'Frozen source snapshot and exact output association; not an external execution signature. XML alone omits explicit Python force law.','source_files':{n:sha(ROOT/n) for n in sources},'b7_builder_sha256':sha(ROOT.parent/'build_model.py'),'solver':'MuJoCo 3.3.7','physics':'MuJoCo rigid mechanics/native pad and panto contacts; Python continuous Hunt-Crossley/Coulomb wheel rail; finite isothermal gas; torque-driven access doors','cases':{n:{'config':m['config'],'parameters':m['parameters'],'source_run_id':m.get('source_run_id',n),'momentum_audit_enabled':m.get('momentum_audit_enabled',False),'files':{n+suffix:sha(OUT/(n+suffix)) for suffix in ['.xml','.csv','_metrics.json','_replay.json']}} for n,m in M.items()},'reused_runs_note':'Five canonical cases reused byte-identical completed HC77 forward solves under equivalent parameters. Low-adhesion 0.1/0.05ms preceded added non-force momentum observer; other reused runs contain it. No trajectories synthesized.'}
 (OUT/'physics_run_identity.json').write_text(json.dumps(identity,indent=2))
 print(v['status'],len(M),'cases',len(v['checks']),'checks; reports and identity written')
if __name__=='__main__':main()
