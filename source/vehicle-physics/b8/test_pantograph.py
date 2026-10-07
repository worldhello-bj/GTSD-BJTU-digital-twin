"""Standalone mechanical verification. Pressure is a force-equivalent diagnostic,
not the integrated variable-volume pneumatic model. Does not command any qpos.

Run from vehicle-physics with:
  PYTHONPATH=vendor python b8/test_pantograph.py
"""
from pathlib import Path
import csv
import json
import os
import sys
from xml.etree.ElementTree import fromstring, SubElement, tostring

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent/'vendor'))
os.environ.setdefault('MUJOCO_GL', 'egl')
os.environ.setdefault('MESA_SHADER_CACHE_DIR', '/tmp/b8-pantograph-mesa')
os.environ.setdefault('MPLCONFIGDIR', '/tmp/b8-pantograph-mpl')
os.environ.setdefault('XDG_CACHE_HOME', '/tmp/b8-pantograph-cache')
import numpy as np
import mujoco
from pantograph import PARAMS, standalone_xml, cylinder_length_from_angle, cylinder_length_jacobian

OUT = ROOT/'pantograph_results'


def run(timestep=.0002, save=False, compliant_overhead=False, force_N=100., output_stem=None):
    xml, spec = standalone_xml(timestep=timestep)
    if compliant_overhead:
        tree=fromstring(xml);world=tree.find('worldbody')
        rail=world.find("geom[@name='panto_overhead_rail']");world.remove(rail)
        body=SubElement(world,'body',name='force_disturbed_overhead',pos='0 0 1.27',gravcomp='1')
        SubElement(body,'joint',name='overhead_vertical',type='slide',axis='0 0 1',stiffness='6000',damping='35',limited='true',range='-.03 .03')
        SubElement(body,'inertial',pos='0 0 0',mass='.5',diaginertia='.001 .001 .001')
        rail.set('pos','0 0 0');rail.set('size','1 .012 .01');body.append(rail)
        xml=tostring(tree,encoding='unicode')
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model);bound=spec.bind(model)
    mujoco.mj_forward(model,data)
    lt=model.site('panto_left_top_pin').id;rt=model.site('panto_right_top_pin').id
    snapshots={};rows=[];max_loop=0.;max_work_error=0.;max_formula_error=0.;max_contact=0.;max_penetration=0.
    impulse_body=model.body('force_disturbed_overhead').id if compliant_overhead else bound.head_body_id
    nstep=round(4./timestep)
    # Only force controls and applied forces are changed in the integration loop.
    # The pressure trace is an explicit diagnostic input, not simulated pneumatics.
    for i in range(nstep):
        t=i*timestep
        gauge_pressure=(force_N/PARAMS['cylinder_area_m2'])*min(1.,max(0.,(t-.1)/.3)) if t<2.7 else 0.
        force=gauge_pressure*spec.params['cylinder_area_m2']
        bound.set_force(data,force)
        disturbed=1.6<=t<1.65
        data.xfrc_applied[impulse_body,2]=(-20. if compliant_overhead else -12.) if disturbed else 0.
        mujoco.mj_step(model,data)
        q=float(data.qpos[bound.qpos_address]);theta=PARAMS['folded_angle_rad']+q
        # mj_step updates qpos at end; refresh derived sites/lengths consistently
        # before collecting telemetry, without changing state or controls.
        mujoco.mj_forward(model,data)
        gap=max(float(np.linalg.norm(data.site_xpos[lt]-data.site_xpos[rt])),float(np.linalg.norm(data.site('panto_lower_carrier_pin').xpos-data.site('panto_lower_balance_pin').xpos)));max_loop=max(max_loop,gap)
        jac= bound.moment_arm(data)
        work_error=abs(float(data.qfrc_actuator@data.qvel)-force*bound.velocity(data))
        max_work_error=max(max_work_error,work_error)
        max_formula_error=max(max_formula_error,abs(bound.travel(data)-spec.cylinder_travel_from_q(q)))
        normal_force=bound.contact_force(model,data)
        max_contact=max(max_contact,normal_force)
        for c in data.contact:
            if c.geom1 in bound.collector_geom_ids or c.geom2 in bound.collector_geom_ids:max_penetration=max(max_penetration,-float(c.dist))
        row={
            'time_s':float(data.time),'gauge_pressure_diagnostic_Pa':gauge_pressure,
            'cylinder_force_N':force,'cylinder_extension_m':bound.travel(data),
            'cylinder_speed_mps':bound.velocity(data),'head_z_m':float(data.site_xpos[bound.head_site_id,2]),
            'contact_normal_N':normal_force,'loop_closure_error_m':gap,
            'external_disturbance_z_N':float(data.xfrc_applied[impulse_body,2]),
            'cylinder_jacobian_mprad':jac,
        }
        if compliant_overhead:
            row['overhead_underside_z_m']=float(data.xpos[impulse_body,2])-.01
        if i%max(1,round(.001/timestep))==0:rows.append(row)
        for label,stamp in [('folded',.05),('raised',1.3),('disturbed',1.65),('vented',3.95)]:
            if label not in snapshots and data.time>=stamp:
                snapshots[label]=data.qpos.copy()
    arr=lambda k,a,b:np.array([r[k] for r in rows if a<=r['time_s']<b])
    contact_times=[r['time_s'] for r in rows if .2<r['time_s']<1.5 and r['contact_normal_N']>.2]
    recontact_times=[r['time_s'] for r in rows if 1.72<r['time_s']<2.6 and r['contact_normal_N']>.5]
    baseline_z=float(np.mean(arr('head_z_m',1.3,1.5)))
    stats={
        'timestep_s':timestep,'mujoco_version':mujoco.__version__,
        'hinge_qpos_count':int(model.nq)-(1 if compliant_overhead else 0),
        'equality_constraint_count':int(model.neq),
        'nominal_pressure_gauge_Pa':force_N/PARAMS['cylinder_area_m2'],'piston_area_m2':PARAMS['cylinder_area_m2'],
        'nominal_force_N':force_N,'source_topology':'B2 single-arm Z, two offset balancing parallelograms','pneumatic_positive_direction':'retraction' ,'first_contact_s':float(contact_times[0]) if contact_times else None,
        'folded_head_z_m':float(np.mean(arr('head_z_m',.02,.09))),
        'raised_head_z_m':baseline_z,
        'lift_m':baseline_z-float(np.mean(arr('head_z_m',.02,.09))),
        'steady_contact_normal_N':float(np.mean(arr('contact_normal_N',1.3,1.5))),
        'peak_contact_normal_N':max_contact,
        'max_collector_contact_penetration_m':max_penetration,
        'disturbance_head_drop_m':baseline_z-float(np.min(arr('head_z_m',1.6,2.1))),
        'recontact_s':float(recontact_times[0]) if recontact_times else None,
        'post_disturbance_contact_normal_N':float(np.mean(arr('contact_normal_N',2.4,2.6))),
        'vented_head_z_m':float(np.mean(arr('head_z_m',3.5,3.99))),
        'vented_contact_normal_N':float(np.mean(arr('contact_normal_N',3.5,3.99))),
        'max_loop_closure_error_m':max_loop,
        'max_cylinder_formula_error_m':max_formula_error,
        'max_virtual_work_power_error_W':max_work_error,
        'warning_counts':[int(w.number) for w in data.warning],
        'finite_state':bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()),
        'mechanical_test_only_no_gas_model':True,
        'disturbance':'20 N downward force on 0.5 kg sprung overhead for 0.05 s, 6 kN/m, 35 Ns/m' if compliant_overhead else '12 N downward force on collector for 0.05 s',
    }
    if compliant_overhead:
        stats['overhead_force_disturbance_drop_m']=float(np.mean(arr('overhead_underside_z_m',1.3,1.5))-np.min(arr('overhead_underside_z_m',1.6,1.85)))
    assert stats['finite_state'] and not any(stats['warning_counts']), stats
    assert stats['lift_m']>.13 and 1.<stats['steady_contact_normal_N']<60.,stats
    assert abs(stats['vented_head_z_m']-1.1)<.0004,stats
    assert stats['vented_contact_normal_N']==0.,stats
    assert stats['max_loop_closure_error_m']<.0001,stats
    assert max_formula_error<1e-10 and max_work_error<1e-9,stats
    assert stats['post_disturbance_contact_normal_N']>1.,stats
    if compliant_overhead:
        assert stats['overhead_force_disturbance_drop_m']>.002,stats
    else:
        if force_N<120.:assert stats['disturbance_head_drop_m']>.01,stats
        assert stats['recontact_s'] is not None,stats
    if save:
        OUT.mkdir(exist_ok=True)
        stem=output_stem or ('compliant_overhead' if compliant_overhead else 'force_diagnostic')
        (OUT/f'{stem}.xml').write_text(xml)
        (OUT/f'{stem}_stats.json').write_text(json.dumps(stats,indent=2)+'\n')
        with (OUT/f'{stem}.csv').open('w') as fp:
            wr=csv.DictWriter(fp,fieldnames=rows[0]);wr.writeheader();wr.writerows(rows)
        if not compliant_overhead and output_stem is None:
            render(model,snapshots)
            plot(rows)
    return stats,rows


def render(model,snapshots):
    """Read-only reconstruction for visualization, never the simulation loop."""
    from PIL import Image,ImageDraw,ImageFont
    renderer=mujoco.Renderer(model,height=640,width=960)
    camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance=.86;camera.azimuth=115;camera.elevation=-14;camera.lookat[:]=[.04,0,1.13]
    option=mujoco.MjvOption();option.sitegroup[:]=0
    data=mujoco.MjData(model)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
    images=[]
    for label,qpos in snapshots.items():
        data.qpos[:]=qpos;mujoco.mj_forward(model,data)
        renderer.update_scene(data,camera=camera,scene_option=option)
        im=Image.fromarray(renderer.render())
        draw=ImageDraw.Draw(im);draw.rectangle((0,0,960,65),fill=(15,22,31))
        draw.text((20,17),f'B8 | SOURCE B2 Z PANTOGRAPH | {label.upper()}',font=font,fill='white')
        im.save(OUT/f'{label}.png');images.append(im.resize((480,320)))
    montage=Image.new('RGB',(960,640))
    for i,im in enumerate(images):montage.paste(im,((i%2)*480,(i//2)*320))
    montage.save(OUT/'pantograph_contact_sequence.png');renderer.close()


def plot(rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=np.array([r['time_s'] for r in rows])
    fig,ax=plt.subplots(3,1,figsize=(9,8),sharex=True)
    for a,key,label in zip(ax,['cylinder_force_N','head_z_m','contact_normal_N'],['Cylinder force (N)','Collector center z (m)','Contact normal (N)']):
        a.plot(t,[r[key] for r in rows]);a.set_ylabel(label);a.grid(alpha=.25)
        a.axvspan(1.6,1.65,color='orange',alpha=.18);a.axvline(2.7,color='gray',ls='--')
    ax[2].set_xlabel('Time (s)')
    fig.suptitle('Force-driven pantograph | pressure-equivalent diagnostic, assumed parameters\nOrange: external downward load; dashed: vent force removed')
    fig.tight_layout();fig.savefig(OUT/'pantograph_force_contact.png',dpi=150);plt.close(fig)


def verify_geometry():
    for theta in np.linspace(PARAMS['folded_angle_rad'],PARAMS['max_angle_rad'],21):
        eps=1e-6
        num=(cylinder_length_from_angle(theta+eps)-cylinder_length_from_angle(theta-eps))/(2*eps)
        assert abs(num-cylinder_length_jacobian(theta))<1e-9
    xml,spec=standalone_xml();m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m);s=spec.bind(m)
    # Set consistent initial conditions for a geometry test only, no integration.
    for q in np.linspace(0,PARAMS['max_angle_rad']-PARAMS['folded_angle_rad'],8):
        for name,value in [('lower_hinge',q),('elbow_hinge',-q),('upper_hinge',-q),('head_level_hinge',q),('upper_balance_hinge',-q),('lower_balance_hinge',q)]:
            d.qpos[m.joint('panto_'+name).qposadr[0]]=value
        mujoco.mj_forward(m,d)
        assert abs(s.travel(d)-spec.cylinder_travel_from_q(q))<1e-10
        theta=PARAMS['folded_angle_rad']+q
        L=PARAMS['link_length_m']
        assert np.linalg.norm(d.body('panto_elbow_carrier').xpos-np.array([L*np.cos(theta),0,1+L*np.sin(theta)]))<1e-10
        assert np.linalg.norm(d.body('panto_collector').xpos-np.array([0,0,1+2*L*np.sin(theta)]))<1e-10
        assert np.linalg.norm(d.body('panto_collector').xmat.reshape(3,3)-np.eye(3))<1e-10
        assert np.linalg.norm(d.site('panto_lower_carrier_pin').xpos-d.site('panto_lower_balance_pin').xpos)<1e-10
        assert np.linalg.norm(d.site('panto_upper_head_pin').xpos-d.site('panto_upper_balance_pin').xpos)<1e-10
        j=s.moment_arm(d)
        assert abs(j+cylinder_length_jacobian(PARAMS['folded_angle_rad']+q))<1e-10
    s.set_force(d,-10.)
    mujoco.mj_forward(m,d)
    assert abs(float(d.actuator_force[s.actuator_id])+10.)<1e-12
    return {'geometry_formula':True,'moment_arm_finite_difference':True,'transmission_jacobian':True,'negative_pressure_force_not_clipped':True,'source_B2_E_H_positions':True,'two_real_parallelogram_closures':True,'source_head_leveling_geometry':True}


if __name__=='__main__':
    OUT.mkdir(exist_ok=True)
    geometry=verify_geometry()
    nominal,_=run(.0002,save=True)
    fine,_=run(.0001)
    overhead,_=run(.0002,save=True,compliant_overhead=True)
    stress,_=run(.0002,save=True,force_N=140.,output_stem='stress_140N')
    stress_fine,_=run(.0001,force_N=140.)
    conv={k:abs(nominal[k]-fine[k]) for k in ['first_contact_s','raised_head_z_m','steady_contact_normal_N','disturbance_head_drop_m','vented_head_z_m','peak_contact_normal_N']}
    assert conv['first_contact_s']<.002
    assert conv['raised_head_z_m']<.00002
    assert conv['steady_contact_normal_N']<.02
    assert conv['disturbance_head_drop_m']<.001
    assert conv['peak_contact_normal_N']/fine['peak_contact_normal_N']<.02
    stress_convergence={k:abs(stress[k]-stress_fine[k]) for k in ['first_contact_s','raised_head_z_m','steady_contact_normal_N','peak_contact_normal_N']}
    assert stress_convergence['peak_contact_normal_N']/stress_fine['peak_contact_normal_N']<.02
    report={'passed':True,'geometry':geometry,'nominal':nominal,'fine':fine,'compliant_overhead':overhead,'stress_140N':stress,'stress_140N_fine':stress_fine,'timestep_absolute_differences':conv,'stress_timestep_absolute_differences':stress_convergence}
    (OUT/'verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
