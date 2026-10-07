"""Source-compatible B8 display adapter. Dynamics come only from recorded inputs."""
import bpy,sys,os,json,hashlib,math,re,argparse,importlib.util
from pathlib import Path
from mathutils import Vector,Matrix,Quaternion
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'source/GTSD_BJTU_B7_detailed.blend'

def run(trajectory,xml,final=False):
    spec=importlib.util.spec_from_file_location('replay_contract',Path(__file__).with_name('replay_contract.py'));contract=importlib.util.module_from_spec(spec);spec.loader.exec_module(contract)
    j,doc,sites,world_geoms,check,jb,xb=contract.inspect(trajectory,xml,final)
    for n in ('reports','inputs','assets','renders'): (ROOT/n).mkdir(exist_ok=True,parents=True)
    tag='final' if final else 'provisional'
    (ROOT/'inputs'/f'{tag}_replay.json').write_bytes(jb);(ROOT/'inputs'/f'{tag}_model.xml').write_bytes(xb)
    before=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    bpy.context.preferences.filepaths.use_scripts_auto_execute=False;bpy.context.preferences.edit.keyframe_new_interpolation_type='LINEAR'
    bpy.ops.wm.open_mainfile(filepath=str(SOURCE),load_ui=False,use_scripts=False)
    old=bpy.data.scenes['B7_MAIN_ASSET_KINEMATIC'];bpy.context.window.scene=old;old.frame_set(1)
    ctl=bpy.data.objects['Panto_Control'];ctl.animation_data_clear();ctl['extension']=0.;ctl.update_tag();bpy.context.view_layer.update()
    has_access_doors=any(n.startswith('access_door_') for n in j['reference_zero_qpos_body_transforms'])
    if has_access_doors:
        bpy.data.objects['B5_Inspection_Control']['cabinet_open_deg']=0.
        bpy.data.objects['B5_Inspection_Control'].update_tag();bpy.context.view_layer.update()
    rig=bpy.data.objects['GTSD_Rig'];dg=bpy.context.evaluated_depsgraph_get()
    def mat4(t):return Matrix.Translation(Vector(t['position']))@Quaternion(t['quaternion_wxyz']).to_matrix().to_4x4()
    refs={n:mat4(t) for n,t in j['reference_zero_qpos_body_transforms'].items()}
    def site_point(name,mats):
        si=sites[name];return (mats[si['body']] if si['body'] else Matrix.Identity(4))@Vector(si['position'])
    def span_matrix(a,b,transverse):
        x=(b-a).normalized();y=(transverse-x*x.dot(transverse)).normalized();z=x.cross(y).normalized()
        m=Matrix.Identity(4)
        for row in range(3):m[row][0]=x[row];m[row][1]=y[row];m[row][2]=z[row];m[row][3]=a[row]
        return m
    def span_quaternion(direction):
        z=direction.normalized();hint=Vector((1,0,0)) if abs(z.x)<.9 else Vector((0,1,0));x=(hint-z*hint.dot(z)).normalized();y=z.cross(x).normalized()
        return Matrix(((x.x,y.x,z.x),(x.y,y.y,z.y),(x.z,y.z,z.z))).to_quaternion()
    def virtuals(mats):
        a=site_point('panto_cylinder_base',mats);b=site_point('panto_cylinder_rod',mats);r=span_matrix(a,b,mats['panto_mount'].to_3x3()@Vector((0,1,0)))
        rod=r.copy();rod.translation=b;return {'panto_barrel_visual':r,'panto_rod_visual':rod}
    refs.update(virtuals(refs))
    source_frames={}
    for pre,car in [('A','31'),('B','32')]:
        root='CarBody_ROOT' if pre=='A' else 'CarBody_B_ROOT';fr='Bogie_ROOT' if pre=='A' else 'Bogie_B_ROOT'
        source_frames[pre+'_carbody']=bpy.data.objects[root].matrix_world.copy();source_frames[pre+'_frame']=bpy.data.objects[fr].matrix_world.copy()
        for idx,ax in [(1,'front'),(2,'rear')]:
            wn=('' if pre=='A' else 'B_')+f'Wheelset_{idx:02d}_ROOT';m=bpy.data.objects[wn].matrix_world.copy()
            source_frames[f'{pre}_{ax}_carrier']=m;source_frames[f'{pre}_{ax}_wheelset']=m
            for side,ysign in [('L',-1),('R',1)]:
                for pn,psign in [('minus',-1),('plus',1)]:source_frames[f'{pre}_{ax}_{side}_{pn}_pad']=Matrix.Translation((m.translation.x+.075,ysign*.179+psign*.0115,.194))
    source_frames['A_motor_rotor']=Matrix.Translation((.75,.088,.255))
    panto_map={'Panto_LowerArm':'panto_lower_arm','Panto_UpperArm':'panto_upper_arm','Panto_LowerBalance':'panto_lower_balance','Panto_UpperBalance':'panto_upper_balance','Panto_ElbowCarrier':'panto_elbow_carrier','Panto_Head':'panto_collector','Panto_Actuator':'panto_barrel_visual','Panto_Piston':'panto_rod_visual'}
    for bone,n in panto_map.items():
        m=rig.matrix_world@rig.pose.bones[bone].matrix@Matrix.Rotation(math.pi/2,4,'Z')
        if 'Balance' in bone:m=m@Matrix.Translation((0,.091,0))
        source_frames[n]=m
    source_frames['panto_mount']=Matrix.Translation((.45,0,1.704))
    ab_spec=importlib.util.spec_from_file_location('source_access_bindings',Path(__file__).with_name('source_access_bindings.py'));ab_mod=importlib.util.module_from_spec(ab_spec);ab_spec.loader.exec_module(ab_mod)
    access=ab_mod.bindings(ROOT/'reports/source_door_closed_rest.json',{n:[list(row) for row in m] for n,m in refs.items()},before)
    source_frames.update({n:Matrix(m) for n,m in access['source_frames'].items()})
    sc=bpy.data.scenes.new('B8_SOURCE_TOPOLOGY_REPLAY');sc.world=old.world;sc.render.engine='CYCLES';sc.cycles.device='CPU';sc.cycles.samples=32;sc.cycles.use_denoising=True;sc.render.resolution_x=1600;sc.render.resolution_y=1000;sc.render.resolution_percentage=100;sc.render.fps=30;sc.render.threads_mode='FIXED';sc.render.threads=8
    sc['status']=tag.upper();sc['mode']='RECORDED_FORWARD_DYNAMICS_DISPLAY_ONLY';sc['scope']=j.get('scope','');sc['source_layout']='Two-car source-model assumption, not confirmed as-built train topology';sc['solver']=(j.get('solver','MuJoCo')+' + explicit Hunt-Crossley/Coulomb wheel-rail forces' if j['parameters'].get('compliant_wheel_rail') else j.get('solver','MuJoCo'));sc['trajectory_sha256']=check['trajectory_sha256'];sc['model_xml_sha256']=check['xml_sha256']
    nodes={};clones={};mapping=[];excluded=[];hooks=[]
    for n,m in refs.items():
        o=bpy.data.objects.new('B8_BODY_'+n,None);sc.collection.objects.link(o);o.matrix_world=m;o.rotation_mode='QUATERNION';o['physics_body']=n;o['display_only_derived']=n.endswith('_visual');nodes[n]=o
    def owner(o):
        while o:
            if o.parent_type=='BONE' and o.parent==rig:return o.parent_bone
            for md in o.modifiers:
                if md.type=='ARMATURE' and md.object==rig:
                    gs=[g.name for g in o.vertex_groups if g.name in rig.pose.bones]
                    if len(gs)!=1:raise ValueError('Non-rigid source mesh '+o.name)
                    return gs[0]
            o=o.parent
        return 'STATIC'
    def classify(o):
        n=o.name;cols={c.name for c in o.users_collection};own=owner(o)
        if o.hide_render or any(c.hide_render for c in o.users_collection):return None,'source hidden/archived'
        if cols & {'14_SENSORS','17_TWIN_SENSORS'}:return None,'legacy markers are not B8 measured channels'
        if n in ('B7S_DUMP_Title','B7S_ADD_Independent','B7S_Air_System_Note'):return None,'legacy floating annotation replaced by B8 screen caption'
        if access['enabled']:
            if n in access['object_targets']:return access['object_targets'][n],'source leaf member follows force-solved door hinge; handles rigid on leaf'
            if n in ('B5_PWR 01_BondingLoop','B5_DAQ 01_BondingLoop'):return None,'legacy stretching bonding-lead animation is not a physical cable solve'
            if own=='STATIC':
                if n.startswith('B4_PWR 01_') or '25_B5_POWER_CABINET' in cols or 'B7_ELECTRICAL_FUNCTIONAL' in cols:return 'access_door_cabinet_PWR_fixed_support','source fixed power cabinet; not a moving chassis'
                if n.startswith('B4_DAQ 01_') or '26_B5_DAQ_CABINET' in cols:return 'access_door_cabinet_DAQ_fixed_support','source fixed DAQ cabinet; not a moving chassis'
        if n.startswith(('Coupler_','AirSpring_','B_AirSpring_','B1_PrimarySeat_','B1_B_PrimarySeat_','B7_AxleArm_')) or n.endswith('_ReactionBracket'):return None,'rebuilt endpoint display or unresolved primary guide detail'
        if n.startswith('B7_Brake_'):
            m=re.match(r'B7_Brake_(31|32)_([12])_([LR])_(.*)',n)
            if not m:raise ValueError(n)
            pre='A' if m[1]=='31' else 'B';ax='front' if m[2]=='1' else 'rear'
            if m[4].startswith(('Pad_','Piston_')):return f'{pre}_{ax}_{"R" if m[3]=="L" else "L"}_{"minus" if m[4].endswith("-1") else "plus"}_pad','moving brake pad/piston'
            return pre+'_'+ax+'_carrier','fixed caliper on axle carrier reduction'
        if '30_B7_DRIVETRAIN_CUTAWAY' in cols:
            if o.parent and o.parent.name=='B7_PinionRotation':return 'A_motor_rotor','rotating motor/pinion assembly'
            if o.parent and o.parent.name=='B7_OutputRotation':return 'A_front_wheelset','rotating output assembly'
            return 'A_front_carrier','axle-carried gearbox/motor casing'
        if n.startswith(('B1_GearboxBearingBoss','B1_GearboxOutputSeal','B1_GearboxDrainPlug','B1_MotorEndPlateBolts','B1_MotorPowerCable')):return 'A_front_carrier','retained traction detail'
        if '05_AXLEBOX' in cols:
            pre='B' if own=='Bogie_32' else 'A';ax='front' if ('01_' in n or '_1_' in n) else 'rear';return f'{pre}_{ax}_carrier','axlebox follows carrier'
        if own in panto_map:return panto_map[own],'source Z-link member'
        if own in ('Body_31','Body_32'):return ('A' if own=='Body_31' else 'B')+'_carbody','rigid body-mounted source detail'
        if own.startswith('BayDoor_'):return 'A_carbody','inspection door latched closed; no independent door solve'
        if own in ('Bogie_31','Bogie_32'):return ('A' if own=='Bogie_31' else 'B')+'_frame','source frame detail'
        if own.startswith('Wheelset_'):
            _,car,idx=own.split('_');return ('A' if car=='31' else 'B')+('_front_wheelset' if idx=='1' else '_rear_wheelset'),'rotating wheelset'
        return None,'not part of moving source vehicle'
    for ob in list(old.objects):
        if ob.type not in ('MESH','CURVE','FONT'):continue
        key,reason=classify(ob)
        if key is None:excluded.append({'source':ob.name,'reason':reason});continue
        if key not in refs:raise ValueError('Missing solver target '+key)
        ev=ob.evaluated_get(dg);me=bpy.data.meshes.new_from_object(ev,preserve_all_data_layers=True,depsgraph=dg)
        if not me or not len(me.vertices):continue
        c=bpy.data.objects.new('B8_MESH_'+ob.name,me);sc.collection.objects.link(c);c.parent=nodes[key];c.matrix_parent_inverse=Matrix.Identity(4);c.matrix_basis=source_frames[key].inverted()@ev.matrix_world
        c['source_object']=ob.name;c['solver_body']=key;c['display_role']=reason;c['geometry_is_not_collision_definition']=True
        clones[ob.name]=c;mapping.append({'source':ob.name,'copy':c.name,'body':key,'role':reason})
        for md in ob.modifiers:
            if md.type=='HOOK' and md.object:hooks.append((c,md.object.name,list(md.vertex_indices)))
    if access['enabled']:
        missing=set(access['object_targets'])-set(clones)
        if missing:raise ValueError('Missing source door visual members: '+str(sorted(missing)))
        source_error=0.
        for n,r in access['source_part_records'].items():
            expected=Matrix(r['source_hinge_local_matrix']);actual=clones[n].matrix_basis
            source_error=max(source_error,max(abs(actual[a][b]-expected[a][b]) for a in range(4) for b in range(4)))
        if source_error>3e-6:raise ValueError('Door source is not at audited closed bind: '+str(source_error))
        access['checks']['part_bind_max_matrix_error']=source_error
    bpy.context.window.scene=sc;bpy.context.view_layer.update()
    for c,target,indices in hooks:
        if target not in clones:raise ValueError('Missing dynamic hose target '+target)
        h=c.modifiers.new('Recorded endpoint deformation','HOOK');h.object=clones[target];h.vertex_indices_set(indices);h.matrix_inverse=clones[target].matrix_world.inverted()@c.matrix_world
    def material(n,col,metal=0):
        m=bpy.data.materials.new(n);m.diffuse_color=(*col,1);m.use_nodes=True;p=m.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=(*col,1);p.inputs['Metallic'].default_value=metal;p.inputs['Roughness'].default_value=.42;return m
    steel=material('B8_Steel',(.42,.48,.50),.75);rubber=material('B8_Bellows',(.018,.028,.032));copper=material('B8_ContactWire',(.49,.27,.09),.7);white=material('B8_Annotation',(.9,.95,1));wp=white.node_tree.nodes.get('Principled BSDF');wp.inputs['Emission Color'].default_value=(.9,.95,1,1);wp.inputs['Emission Strength'].default_value=.8
    captionmat=material('B8_CaptionBackground',(.022,.036,.048));cp=captionmat.node_tree.nodes.get('Principled BSDF');cp.inputs['Emission Color'].default_value=(.022,.036,.048,1);cp.inputs['Emission Strength'].default_value=1
    def mesh(n,v,f,mat):
        me=bpy.data.meshes.new(n);me.from_pydata(v,[],f);me.update();o=bpy.data.objects.new(n,me);sc.collection.objects.link(o);me.materials.append(mat);return o
    def box(n,pos,size,mat):
        v=[tuple(pos[i]+sg[i]*size[i]/2 for i in range(3)) for sg in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        return mesh(n,v,[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],mat)
    def lathe(n,profile,mat,segments=36):
        v=[(r*math.cos(2*math.pi*k/segments),r*math.sin(2*math.pi*k/segments),z) for r,z in profile for k in range(segments)];f=[]
        for q in range(len(profile)):
            q2=(q+1)%len(profile)
            for k in range(segments):k2=(k+1)%segments;f.append((q*segments+k,q*segments+k2,q2*segments+k2,q2*segments+k))
        o=mesh(n,v,f,mat)
        for p in o.data.polygons:p.use_smooth=True
        return o
    def attach_site(o,name):
        si=sites[name];o.parent=nodes[si['body']];o.matrix_parent_inverse=Matrix.Identity(4);o.location=si['position'];o.rotation_mode='QUATERNION';o.rotation_quaternion=si['quat']
    # Geometry is kept fixed in world coordinates; the model may deliberately use
    # a longer diagnostic track than the source's ten-metre room installation.
    static=[]
    for g in world_geoms:
        n=g.get('name','')
        if n.startswith('rail_') or 'overhead' in n:
            if g.get('type')!='box':raise ValueError('Unsupported static contact geometry '+n)
            p=list(map(float,g.get('pos','0 0 0').split()));sz=[2*x for x in map(float,g['size'].split())];o=box('B8_STATIC_'+n,p,sz,copper if 'overhead' in n else steel);static.append(o)
    length=j['parameters']['rail_length_m']
    for i in range(int(length/.4)+1):static.append(box('B8_TestTrack_CrossBeam_'+str(i),(.4*i,0,-.018),(.055,.74,.028),steel))
    spans=[]
    tendons=doc.find('tendon')
    for td in tendons.findall('spatial'):
        name=td.get('name','');ss=[x.get('site') for x in td.findall('site')]
        if len(ss)!=2:continue
        if name.startswith('air_'):radius=.050;role='air bellows shell display; pressure force is in solver'
        elif 'primary_visual' in name:radius=.038;role='equivalent primary support display; force is an equivalent joint'
        elif name=='coupler':radius=.025;role='compliant endpoint coupler display'
        else:continue
        a,b=[site_point(n,refs) for n in ss];L=(b-a).length
        if L<=0:raise ValueError('Zero span '+name)
        if name=='coupler':profile=[(.012,0),(.025,0),(.025,1),(.012,1)]
        else:profile=[(.7*radius,0),(.94*radius,.035)]+[(radius*(1 if q%2 else .72),q/12) for q in range(1,12)]+[(.94*radius,.965),(.7*radius,1),(.55*radius,1),(.55*radius,0)]
        o=lathe('B8_SPAN_'+name,profile,steel if name=='coupler' else rubber);o.rotation_mode='QUATERNION';o['display_role']=role;spans.append((o,ss[0],ss[1],L))
        for k,site in enumerate(ss):
            cap=lathe('B8_END_'+name+'_'+str(k),[(radius*.45,-.003),(radius*1.04,-.003),(radius*1.04,.003),(radius*.45,.003)],steel);attach_site(cap,site)
            if name=='coupler':cap.rotation_quaternion=cap.rotation_quaternion@Quaternion((0,1,0),math.pi/2)
    tracker=bpy.data.objects.new('B8_TranslationOnlyCameraTrack',None);sc.collection.objects.link(tracker)
    ptrack=bpy.data.objects.new('B8_PantoTranslationOnlyTrack',None);sc.collection.objects.link(ptrack)
    stats={'minimum_span_length_m':1e9,'maximum_span_length_m':0,'body_matrix_error':0,'span_endpoint_error_m':0}
    telemetry=[]
    for rec in j['frames']:
        f=1+rec['time_s']*sc.render.fps;mats={n:mat4(t) for n,t in rec['body_transforms'].items()};mats.update(virtuals(mats))
        for n,m in mats.items():
            o=nodes[n];o.matrix_world=m;o.keyframe_insert('location',frame=f);o.keyframe_insert('rotation_quaternion',frame=f)
        for o,a,b,_ in spans:
            va=site_point(a,mats);vb=site_point(b,mats);d=vb-va;L=d.length
            if not math.isfinite(L) or L<=0:raise ValueError('Invalid mechanical display span')
            stats['minimum_span_length_m']=min(stats['minimum_span_length_m'],L);stats['maximum_span_length_m']=max(stats['maximum_span_length_m'],L)
            o.location=va;o.rotation_quaternion=span_quaternion(d);o.scale=(1,1,L)
            for path in ('location','rotation_quaternion','scale'):o.keyframe_insert(path,frame=f)
        av=mats['A_carbody'].translation;bv=mats['B_carbody'].translation;tracker.location=((av.x+bv.x)/2,(av.y+bv.y)/2,0);tracker.keyframe_insert('location',frame=f);ptrack.location=(av.x,av.y,0);ptrack.keyframe_insert('location',frame=f)
        telemetry.append({k:v for k,v in rec.items() if k not in ('qpos','body_transforms')})
    sc.frame_start=1;sc.frame_end=math.ceil(1+j['frames'][-1]['time_s']*sc.render.fps)
    def camera(name,track,pos,target,scale):
        d=bpy.data.cameras.new(name);o=bpy.data.objects.new(name,d);sc.collection.objects.link(o);o.parent=track;o.location=pos;o.rotation_euler=(Vector(target)-Vector(pos)).to_track_quat('-Z','Y').to_euler();d.type='ORTHO';d.ortho_scale=scale;d.clip_start=.01;return o
    cam=camera('B8_MainCamera',tracker,(6,-8,4.6),(0,0,.95),7.2);sc.camera=cam
    pcam=camera('B8_PantographCamera',ptrack,(-.28,-1.2,2.30),(-.78,0,1.84),.82)
    for n,p,energy,size in [('Key',(2,-3,6),1000,5),('Fill',(-2,3,4),800,4)]:
        d=bpy.data.lights.new('B8_'+n,'AREA');d.energy=energy;d.size=size;o=bpy.data.objects.new('B8_'+n,d);sc.collection.objects.link(o);o.parent=tracker;o.location=p;o.rotation_euler=(Vector((0,0,.9))-o.location).to_track_quat('-Z','Y').to_euler()
    def label(name,body,cam,pos,size):
        d=bpy.data.curves.new(name,'FONT');d.body=body;d.size=size;d.materials.append(white);o=bpy.data.objects.new(name,d);sc.collection.objects.link(o);o.parent=cam;o.location=(*pos,-.2);o.visible_shadow=False;return o
    band=box('B8_MainCaptionBand',(0,0,0),(7.2,.90,.001),captionmat);band.parent=cam;band.location=(0,1.80,-.25);band.visible_shadow=False;band.visible_diffuse=False;band.visible_glossy=False
    label('B8_Header',('PROVISIONAL / ' if not final else '')+'B8 SOURCE-TOPOLOGY DYNAMICS',cam,(-3.3,1.86),.13)
    label('B8_Subheader','Two source-model-assumption bodies / A powered / pneumatic actuation / Z-link pantograph',cam,(-3.3,1.65),.070)
    label('B8_Caution',('Hybrid force solve: explicit wheel/rail + MuJoCo joints/contacts; uncalibrated parameters' if j['parameters'].get('compliant_wheel_rail') else 'Recorded solver replay; assumed parameters and reduced contacts, not a calibrated physical specimen'),cam,(-3.3,1.47),.065)
    band2=box('B8_PantoCaptionBand',(0,0,0),(.82,.075,.001),captionmat);band2.parent=pcam;band2.location=(0,.219,-.25);band2.visible_shadow=False;band2.visible_diffuse=False;band2.visible_glossy=False
    label('B8_PantoHeader','SOURCE Z-LINK / PRESSURE-DRIVEN REPLAY',pcam,(-.38,.226),.016)
    label('B8_PantoCaution','Recorded solver motion; dimensions and air parameters are uncalibrated',pcam,(-.38,.199),.0083)
    sc.frame_set(1);bpy.context.view_layer.update()
    # Verify every recorded pose, never just first/middle/last.
    for rec in j['frames']:
        f=1+rec['time_s']*sc.render.fps;sc.frame_set(int(f),subframe=f-int(f));bpy.context.view_layer.update();mats={n:mat4(t) for n,t in rec['body_transforms'].items()};mats.update(virtuals(mats))
        for n,m in mats.items():stats['body_matrix_error']=max(stats['body_matrix_error'],max(abs(nodes[n].matrix_world[a][b]-m[a][b]) for a in range(4) for b in range(4)))
        for o,a,b,_ in spans:
            actuala=o.matrix_world@Vector((0,0,0));actualb=o.matrix_world@Vector((0,0,1));err=max((actuala-site_point(a,mats)).length,(actualb-site_point(b,mats)).length)
            if err>stats['span_endpoint_error_m']:
                stats['span_endpoint_error_m']=err;stats['worst_span']={'name':o.name,'time':rec['time_s'],'actual_a':list(actuala),'expected_a':list(site_point(a,mats)),'actual_b':list(actualb),'expected_b':list(site_point(b,mats)),'scale':list(o.scale),'quaternion':list(o.rotation_quaternion)}
    (ROOT/'reports'/f'{tag}_display_check.json').write_text(json.dumps(stats,indent=2))
    if stats['body_matrix_error']>2e-5 or stats['span_endpoint_error_m']>2e-5:raise ValueError('Display reconstruction error '+str(stats))
    manifest=check|{'source_blend_sha256':before,'mapped_source_objects':len(mapping),'mapping':mapping,'excluded_source_objects':excluded,'derived_endpoint_spans':[(o.name,a,b,L) for o,a,b,L in spans],'checks':stats,'physics_validation':'Not performed by adapter; see matched solver validation.','solver_description':sc['solver'],'access_door_binding':access['checks'],'locked_parts':('Six access door leaves follow recorded force-solved hinge bodies; handles stay rigid and latch detail remains solver-declared.' if access['enabled'] else 'Inspection doors and rigid equipment are attached in their closed/assembled source state.'),'unresolved_geometry':'Primary trailing arms/reaction brackets excluded where equivalent solver joints do not preserve their literal linkage; cable mass/contact not solved.'}
    bpy.data.texts.new('B8_DISPLAY_MAPPING.json').write(json.dumps(manifest,ensure_ascii=False,indent=2));bpy.data.texts.new('B8_REPLAY_TELEMETRY.json').write(json.dumps(telemetry,ensure_ascii=False))
    (ROOT/'reports'/f'{tag}_mapping.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    close=bpy.data.scenes.new('B8_PANTOGRAPH_CLOSEUP');close.world=sc.world;close.render.engine=sc.render.engine;close.cycles.device='CPU';close.cycles.samples=32;close.cycles.use_denoising=True;close.render.resolution_x=1600;close.render.resolution_y=1000;close.render.resolution_percentage=100;close.render.fps=sc.render.fps;close.render.threads_mode='FIXED';close.render.threads=8;close.camera=pcam;close.frame_start=1;close.frame_end=sc.frame_end
    main_captions={'B8_MainCaptionBand','B8_Header','B8_Subheader','B8_Caution'};close_captions={'B8_PantoCaptionBand','B8_PantoHeader','B8_PantoCaution'}
    for o in list(sc.objects):
        if o.name not in main_captions:close.collection.objects.link(o)
        if o.name in close_captions:sc.collection.objects.unlink(o)
    close['mode']=sc['mode'];close['solver']=sc['solver'];close['trajectory_sha256']=check['trajectory_sha256'];close['model_xml_sha256']=check['xml_sha256'];close.frame_set(1)
    sc.frame_set(1);bpy.context.window.scene=sc
    for other in list(bpy.data.scenes):
        if other not in (sc,close):bpy.data.scenes.remove(other)
    bpy.data.orphans_purge(do_recursive=True)
    dest=ROOT/'assets'/('GTSD_BJTU_B8_source_dynamics.blend' if final else 'B8_PROVISIONAL_MAPPING_ONLY.blend')
    bpy.ops.wm.save_as_mainfile(filepath=str(dest),compress=True)
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest()!=before:raise ValueError('B7 source changed')
    print(json.dumps({'output':str(dest),'status':tag,'mapped_objects':len(mapping),'bodies':len(j['reference_zero_qpos_body_transforms']),'frames':len(j['frames']),'checks':stats},indent=2),flush=True)
    return dest

if __name__=='__main__':
    argv=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else sys.argv[1:]
    ap=argparse.ArgumentParser();ap.add_argument('--trajectory',required=True);ap.add_argument('--model-xml',required=True);ap.add_argument('--final',action='store_true');a=ap.parse_args(argv)
    run(a.trajectory,a.model_xml,a.final);sys.stdout.flush();os._exit(0)
