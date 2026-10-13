"""Detached, named, identity-bound observations. Never writes native state."""
import copy,hashlib,json
from pathlib import Path


def normalize_command_registry(snapshot):
    """Export only: source enum/values becomes controller string/enum."""
    value=copy.deepcopy(snapshot)
    if value.get('variant')!='cad_doors_four_fans':raise ValueError('82 presentation requires explicit variant')
    key=value['command_registry']['four_door_fan']['args']['key']
    if set(key)!={'type','values'} or key['type']!='enum' or not isinstance(key['values'],list) or len(key['values'])!=len(set(key['values'])) or any(type(x)is not str for x in key['values']):raise ValueError('Unexpected frozen four-fan key schema')
    value['command_registry']['four_door_fan']['args']['key']={'type':'string','enum':list(key['values'])}
    value.setdefault('display_interface',{}).update(fan82_view_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),command_schema_normalization='four_door_fan.args.key: enum/values -> string/enum; exact values preserved; raw source command validates again')
    return value


def named_endpoint(plant):
    import mujoco,numpy as np
    m,d=plant.model,plant.data;coordinates={};world={}
    original=plant.auxiliary_spec['coordinates'];extra=plant.four_fan_spec['added_mounts']
    names={n:n for n in original}
    names.update({v['joint']:v['rotor'] for v in extra.values()})
    for joint,body in names.items():
        jid=m.joint(joint).id;kind=int(m.jnt_type[jid]);qa=int(m.jnt_qposadr[jid]);va=int(m.jnt_dofadr[jid])
        if kind not in (int(mujoco.mjtJoint.mjJNT_HINGE),int(mujoco.mjtJoint.mjJNT_SLIDE)):raise ValueError('Auxiliary coordinate is not scalar hinge/slide')
        rotary=kind==int(mujoco.mjtJoint.mjJNT_HINGE)
        coordinates[joint]={'joint':joint,'body':body,'kind':'rotation' if rotary else 'translation','position':float(d.qpos[qa]),'velocity':float(d.qvel[va]),'position_unit':'rad' if rotary else 'm','velocity_unit':'rad/s' if rotary else 'm/s','resolved_qpos_address':qa,'resolved_dof_address':va,'address_usage':'resolved from current joint name, not retained array index'}
        if rotary:
            bid=m.body(body).id;parent=int(m.body_parentid[bid]);jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);js=np.zeros_like(jp)
            mujoco.mj_jacBody(m,d,jp,jr,bid);mujoco.mj_jacBody(m,d,jp,js,parent)
            wr=jr@d.qvel;ws=js@d.qvel;axis=d.xaxis[jid].copy();residual=float(axis@(wr-ws)-d.qvel[va])
            if abs(residual)>1e-9:raise ValueError('World/relative angular speed residual')
            world[joint]={'rotor_body':body,'stator_body':m.body(parent).name,'axis_world_xyz':axis.tolist(),'angular_velocity_rotor_world_rad_s':wr.tolist(),'angular_velocity_stator_world_rad_s':ws.tolist(),'angular_velocity_air_reference_world_rad_s':[0.,0.,0.],'relative_native_omega_rad_s':float(d.qvel[va]),'world_minus_stator_axis_residual_rad_s':residual}
    return {'schema':'gtsd-fan82-named-endpoint-v1','sample_time_s':float(d.time),'coordinates':coordinates,'world':world,'inventory':{'rotating_axes':sum(v['kind']=='rotation' for v in coordinates.values()),'sliding_axes':sum(v['kind']=='translation' for v in coordinates.values()),'native_added_coordinates':len(coordinates),'native_nv':int(m.nv),'native_nq':int(m.nq)},'sampling':'instantaneous endpoint native q/velocity/Jacobian; separate from accepted interval work','hardware_calibrated':False}


def fan_records(snapshot,spec):
    """Source paths retained on every value; commands and accepted history distinct."""
    from lineage_registry import get,valid_value
    defs={};selected='end_neg_y_neg'
    fields=[]
    def add(key,label,unit,path,shape=None,interval=False):
        found,value=get(snapshot,path);okay=found and valid_value(value,unit,shape) and (not interval or snapshot.get('steps',0)>0)
        fields.append({'id':f'fan82.{key}.{label}','fan':key,'field':label,'unit':unit,'shape':shape,'value':copy.deepcopy(value) if okay else None,'available':okay,'availability':'IDEAL_SOLVER_NOT_HARDWARE' if okay else ('NO_ACCEPTED_INTERVAL' if interval and not snapshot.get('steps',0) else 'MISSING_OR_INVALID_NAMED_PATH'),'source_path':path,'requires_accepted_interval':interval,'sample_time_s':snapshot.get('time_s')})
    for key in [selected,*spec['added_mounts']]:
        old=key==selected;native=['telemetry','auxiliary_mount','native'] if old else ['telemetry','four_door_fans','native',key]
        body='bay_fan_rotor' if old else spec['added_mounts'][key]['rotor'];stator='bay_fan_stator' if old else spec['added_mounts'][key]['stator']
        defs[key]={'rotor':body,'stator':stator,'mesh_name':'CAD_fan_shroud_'+key,'source_kind':'original_native_bay_fan' if old else 'additional_same_spec_research_fan'}
        for field,unit in [('angle_rad','rad'),('omega_rad_s','rad/s')]:add(key,field,unit,native+[field,'bay_fan_rotor'] if old else native+[field])
        motor=['telemetry','pneumatic','fans','bay_fan'] if old else ['telemetry','four_door_fans','motors',key]
        for field,unit in [('current_A','A'),('electrical_work_J','J'),('copper_heat_J','J'),('shaft_work_J','J'),('power_failed','boolean')]:add(key,'accepted_motor_'+field,unit,motor+[field],interval=True)
        add(key,'voltage_fraction','1',['telemetry','pneumatic','commands','fan_voltage_fractions','bay_fan'] if old else ['telemetry','four_door_fans','controls',key,'voltage_fraction'])
        add(key,'commanded_power_failed','boolean',['telemetry','pneumatic','commands','fan_power_failed','bay_fan'] if old else ['telemetry','four_door_fans','controls',key,'power_failed'])
        mount=['telemetry','auxiliary_mount','last_sensor','bay_fan'] if old else ['telemetry','four_door_fans','last_sensor',key]
        for field,unit,shape in [('force_world_N','N',3),('torque_world_Nm','N m',3),('force_on_parent_world_N','N',3),('torque_on_parent_world_Nm','N m',3),('site_position_world_m','m',3),('force_evaluation_start_s','s',None),('solver_interval_end_s','s',None)]:add(key,field,unit,mount+[field],shape,True)
        for field in ('angular_velocity_rotor_world_rad_s','angular_velocity_stator_world_rad_s'):
            add(key,field,'rad/s',['telemetry','fan82_named_endpoint','world',body,field],3)
    return defs,fields


def panel_payload(snapshot,spec,identity,geometry):
    defs,fields=fan_records(snapshot,spec);okay=snapshot.get('variant')=='cad_doors_four_fans' and snapshot.get('model_identity_sha256')==identity and snapshot.get('source_geometry_sha256')==geometry
    if not okay:
        for row in fields:row.update(value=None,available=False,availability='MODEL_OR_GEOMETRY_IDENTITY_MISMATCH')
    t=snapshot.get('telemetry',{});columns=t.get('energy_work_columns',[]);work=t.get('energy_work_columns_J',[])
    extra=t.get('four_door_fans',{}).get('work',{})
    named_work={n:work[columns.index(n)] for n in extra if len(columns)==len(work) and columns.count(n)==1}
    return {'schema':'gtsd-four-fan-console-observations-v1','variant':snapshot.get('variant'),'model_identity_sha256':snapshot.get('model_identity_sha256'),'source_geometry_sha256':snapshot.get('source_geometry_sha256'),'identity_match':okay,'sample_time_s':snapshot.get('time_s'),'steps':snapshot.get('steps'),'fans':defs,'observations':fields,'inventory':t.get('fan82_named_endpoint',{}).get('inventory',{}),'extra_work_columns_J':named_work if okay else {},'extra_work_columns_counted_once':okay and len(named_work)==len(extra) and all(named_work[k]==v for k,v in extra.items()),'hardware_connected':False,'hardware_calibrated':False,'renews_run_lease':False,'limits':['四门独立理想电源支路；新增三台电机/转子按同规格工程假设，未硬件标定。','新增三台铜耗/轴承热仅独立诊断，未接入原机柜温度/气路热网络。','0电压仍为连接电阻支路；power_failed=true 为断路。指令状态与最后接受区间电流分列。','安装反力为原生虚拟传感器；起点力评价时间与区间终点分开，不重复施力/计功。','辅机压力峰值/全状态收敛仍未完成；不能以软件响应判定实物已标定。','风机罩固定于定子；未提供真实叶片几何，不能将整罩旋转充当转子。','冻结82恢复校验仍有已知边界：重签后的 last.air_speed_rad_s / relative_speed_rad_s 派生诊断可篡改接受；它们不参与物理推进。本视图当前速度独立按原生名字读取，不将这些历史诊断标记为已逐字段验证。']}
