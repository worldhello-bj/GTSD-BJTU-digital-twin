"""Source-B2 single-arm Z pantograph with two actual parallelogram loops.

No time trajectory or position controller. Pressure acts on an assumed annular
RETRACTION chamber, whose positive gas coordinate is l_folded - l_current.
B2 source defines geometry/kinematic relationship only; masses, springs, contact
properties, and ideal passive fold synchronization remain unverified assumptions.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import cos, sin, pi, sqrt, asin
from xml.etree.ElementTree import Element, SubElement, tostring, indent

PARAMS = {
 'link_length_m': .16, 'minimum_lift_m': .055, 'maximum_lift_m': .220,
 'parallel_link_offset_m': .034, 'pivot_height_above_base_m': .034,
 'folded_angle_rad': asin(.055/.32), 'max_angle_rad': asin(.220/.32),
 'link_mass_kg': .025, 'balance_mass_kg': .007, 'carrier_mass_kg': .010,
 'link_radius_m': .007, 'head_mass_kg': .080,
 'head_half_length_y_m': .180, 'head_half_width_x_m': .010,
 'head_thickness_m': .009, 'head_contact_offset_z_m': .045,
 'head_contact_spacing_x_m': .062,
 'cylinder_base_x_m': -.085, 'cylinder_base_y_m': -.105,
 'cylinder_base_z_m': 0., 'cylinder_arm_attachment_m': .080,
 'cylinder_area_m2': .0002, 'cylinder_force_sign': -1,
 'return_stiffness_Npm': 80., 'root_return_stiffness_Nmprad': .03,
 'hinge_damping_Nmsprad': .003, 'cylinder_max_force_N': 200.,
 'contact_friction': .12, 'collision_bit': 8,
}


def _add(parent, tag, **attrs):
 return SubElement(parent, tag, {k:str(v) for k,v in attrs.items()})


def _vec(*v):
 return ' '.join(f'{float(x):.12g}' for x in v)


def _quat_y(angle):
 return _vec(cos(angle/2),0,sin(angle/2),0)


def cylinder_length_from_angle(theta, params=None):
 p=PARAMS | (params or {})
 r,bx,bz=(p[k] for k in ('cylinder_arm_attachment_m','cylinder_base_x_m','cylinder_base_z_m'))
 return sqrt((r*cos(theta)-bx)**2+(r*sin(theta)-bz)**2)


def cylinder_length_jacobian(theta, params=None):
 """d physical cylinder length / d theta, NEGATIVE for source geometry."""
 p=PARAMS | (params or {})
 r,bx,bz=(p[k] for k in ('cylinder_arm_attachment_m','cylinder_base_x_m','cylinder_base_z_m'))
 return r*(bx*sin(theta)-bz*cos(theta))/cylinder_length_from_angle(theta,p)


def pneumatic_travel_jacobian(theta, params=None):
 """d chamber positive retraction / d theta, POSITIVE during raising."""
 return -cylinder_length_jacobian(theta,params)


@dataclass(frozen=True)
class PantographSpec:
    prefix: str
    params: dict
    actuator_name: str
    tendon_name: str
    lower_joint_name: str
    head_body_name: str
    head_site_name: str
    collector_geom_name: str
    initial_cylinder_length_m: float
    maximum_cylinder_travel_m: float
    folded_head_center_above_mount_m: float

    def cylinder_travel_from_q(self, lower_q):
        return self.initial_cylinder_length_m-cylinder_length_from_angle(self.params['folded_angle_rad']+lower_q, self.params)

    def bind(self, model):
        """Resolve model IDs. Call update after mj_forward/mj_step for current lengths."""
        return PantographState(self, model)


class PantographState:
    def __init__(self, spec, model):
        import mujoco
        self.spec = spec
        self.actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec.actuator_name)
        self.tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, spec.tendon_name)
        self.head_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, spec.head_body_name)
        self.head_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, spec.head_site_name)
        self.collector_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, spec.collector_geom_name)
        self.collector_geom_ids = tuple(i for i in range(model.ngeom) if (mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith(spec.collector_geom_name))
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, spec.lower_joint_name)
        self.qpos_address = int(model.jnt_qposadr[j])
        self.qvel_address = int(model.jnt_dofadr[j])
        if min(self.actuator_id, self.tendon_id, self.head_body_id, self.head_site_id, self.collector_geom_id, j) < 0:
            raise ValueError('Pantograph is missing from this model')

    def travel(self, data):
        """Physical retraction [m], may be slightly below zero at a soft end stop."""
        return self.spec.initial_cylinder_length_m-float(data.ten_length[self.tendon_id])

    def velocity(self, data):
        return -float(data.ten_velocity[self.tendon_id])

    def moment_arm(self, data):
        """Pneumatic retraction Jacobian [m/rad] at the lower arm hinge."""
        start = int(data.moment_rowadr[self.actuator_id])
        count = int(data.moment_rownnz[self.actuator_id])
        for k in range(start, start+count):
            if int(data.moment_colind[k]) == self.qvel_address:
                return float(data.actuator_moment[k])
        return 0.

    def set_force(self, data, newtons):
        """Force input only. Positive force retracts the cylinder; no servo logic."""
        data.ctrl[self.actuator_id] = float(newtons)

    def contact_force(self, model, data):
        """Sum of normal forces [N] for actual collector contacts (nonnegative)."""
        import mujoco
        import numpy as np
        total = 0.
        wrench = np.zeros(6)
        for i in range(data.ncon):
            c = data.contact[i]
            if c.geom1 in self.collector_geom_ids or c.geom2 in self.collector_geom_ids:
                mujoco.mj_contactForce(model, data, i, wrench)
                total += max(0., float(wrench[0]))
        return total


def add_pantograph(parent_body, worldbody, equality, tendon, actuators,
                   prefix='panto', base_z=0., params=None):
 """Append source-consistent Z linkage. base_z is lower pivot A, parent-local.

 Axes x longitudinal/y transverse/z up; angle units radian. Main lower/upper
 links and their offset balance rods form two real closed parallelogram loops.
 A declared ideal passive inter-stage synchronizer couples equal fold angles.
 The source does not establish the actual hidden synchronization hardware.
 """
 p=PARAMS | (params or {})
 L,th,off=(p[k] for k in ('link_length_m','folded_angle_rad','parallel_link_offset_m'))
 beta=pi-th
 n=lambda s:f'{prefix}_{s}'
 mount=_add(parent_body,'body',name=n('mount'),pos=_vec(0,0,base_z))
 _add(mount,'inertial',pos='0 0 -.04',mass='.15',diaginertia='.001 .001 .001')
 _add(mount,'geom',name=n('base'),type='box',size='.17 .12 .01',pos=_vec(.015,0,-p['pivot_height_above_base_m']-.01),contype=0,conaffinity=0,rgba='.21 .26 .29 1')
 for xx,yy in [(0,-.074),(0,.074),(off,.091),(p['cylinder_base_x_m'],p['cylinder_base_y_m'])]:
  _add(mount,'geom',type='box',pos=_vec(xx,yy,-.017),size='.015 .012 .025',contype=0,conaffinity=0,rgba='.4 .44 .47 1')
 _add(mount,'site',name=n('cylinder_base'),pos=_vec(p['cylinder_base_x_m'],p['cylinder_base_y_m'],p['cylinder_base_z_m']),size='.005',rgba='.75 .8 .84 1')

 def hinge(body,name,stiffness=0,limited=False):
  kw=dict(name=n(name),type='hinge',axis='0 -1 0',damping=p['hinge_damping_Nmsprad'],armature='.000001',limited='false')
  if limited:kw.update(limited='true',range=_vec(0,p['max_angle_rad']-th),stiffness=stiffness,springref=0,solreflimit='.002 1',solimplimit='.99 .999 .0001')
  return _add(body,'joint',**kw)

 def link(body,mass,ys,width,color):
  rad=width/2
  _add(body,'inertial',pos=_vec(L/2,0,0),mass=mass,diaginertia=_vec(max(1e-7,mass*rad*rad/2),mass*(L*L/12+rad*rad/4),mass*(L*L/12+rad*rad/4)))
  for y in ys:
   _add(body,'geom',type='capsule',fromto=_vec(0,y,0,L,y,0),size=rad,contype=0,conaffinity=0,rgba=color)
  _add(body,'geom',type='capsule',fromto=_vec(0,-.08,0,0,.10,0),size='.007',contype=0,conaffinity=0,rgba='.65 .7 .74 1')

 # Main lower arm A -> E.
 lower=_add(mount,'body',name=n('lower_arm'),quat=_quat_y(-th))
 hinge(lower,'lower_hinge',p['root_return_stiffness_Nmprad'],True)
 link(lower,p['link_mass_kg'],[-.052,.052],.019,'.23 .28 .31 1')
 _add(lower,'site',name=n('cylinder_rod'),pos=_vec(p['cylinder_arm_attachment_m'],p['cylinder_base_y_m'],0),size='.005',rgba='.75 .8 .84 1')
 _add(lower,'geom',type='capsule',fromto=_vec(p['cylinder_arm_attachment_m'],-.114,0,p['cylinder_arm_attachment_m'],-.048,0),size='.008',contype=0,conaffinity=0,rgba='.65 .7 .74 1')
 # Elbow carrier has its own real hinge. Lower offset rod closes the loop and
 # keeps this carrier parallel to the mount without any leveling trajectory.
 elbow=_add(lower,'body',name=n('elbow_carrier'),pos=_vec(L,0,0),quat=_quat_y(th))
 hinge(elbow,'elbow_hinge')
 _add(elbow,'inertial',pos=_vec(off/2,.045,0),mass=p['carrier_mass_kg'],diaginertia='.000015 .000004 .000015')
 _add(elbow,'geom',type='box',pos=_vec(off/2,.091,0),size=_vec(off/2+.009,.009,.0075),contype=0,conaffinity=0,rgba='.55 .61 .65 1')
 _add(elbow,'site',name=n('lower_carrier_pin'),pos=_vec(off,.091,0),size='.004')
 lowbal=_add(mount,'body',name=n('lower_balance'),pos=_vec(off,.091,0),quat=_quat_y(-th))
 hinge(lowbal,'lower_balance_hinge');link(lowbal,p['balance_mass_kg'],[0],.009,'.48 .55 .59 1')
 _add(lowbal,'site',name=n('lower_balance_pin'),pos=_vec(L,0,0),size='.004')
 _add(equality,'connect',name=n('lower_parallelogram'),site1=n('lower_carrier_pin'),site2=n('lower_balance_pin'),solref='.002 1',solimp='.99 .9999 .00001')
 # Main upper arm E -> H; the Z changes direction in x at E.
 upper=_add(elbow,'body',name=n('upper_arm'),quat=_quat_y(-beta))
 hinge(upper,'upper_hinge');link(upper,p['link_mass_kg'],[-.026,.026],.013,'.23 .28 .31 1')
 head=_add(upper,'body',name=n('collector'),pos=_vec(L,0,0),quat=_quat_y(beta))
 hinge(head,'head_level_hinge')
 hm=p['head_mass_kg'];hy=p['head_half_length_y_m'];hx=.05
 _add(head,'inertial',pos=_vec(off/2,0,.025),mass=hm,diaginertia=_vec(hm*(hy*hy+.01**2)/3,hm*(hx*hx+.01**2)/3,hm*(hx*hx+hy*hy)/3))
 _add(head,'geom',name=n('collector_carrier'),type='box',pos=_vec(off/2,0,.025),size='.025 .1825 .006',contype=0,conaffinity=0,rgba='.50 .57 .6 1')
 for yy in [-.054,.054]:
  _add(head,'geom',type='box',pos=_vec(0,yy,.013),size='.009 .0045 .013',contype=0,conaffinity=0,rgba='.50 .57 .6 1')
 for i,xx in enumerate([off/2-.031,off/2+.031]):
  _add(head,'geom',name=n('collector_contact'+('' if i==0 else '_2')),type='box',pos=_vec(xx,0,p['head_contact_offset_z_m']),size=_vec(p['head_half_width_x_m'],hy,p['head_thickness_m']/2),contype=p['collision_bit'],conaffinity=p['collision_bit'],condim=3,friction=_vec(p['contact_friction'],.001,.0001),solref='.004 1',solimp='.95 .99 .0005',rgba='.04 .05 .06 1')
 _add(head,'site',name=n('head_center'),pos=_vec(off/2,0,p['head_contact_offset_z_m']),size='.005')
 _add(head,'site',name=n('upper_head_pin'),pos=_vec(off,.091,0),size='.004')
 # Legacy site aliases now measure the upper parallelogram closure, NOT a
 # diamond. They keep existing vehicle telemetry compatible across correction.
 _add(head,'site',name=n('left_top_pin'),pos=_vec(off,.091,0),size='.003')
 upbal=_add(elbow,'body',name=n('upper_balance'),pos=_vec(off,.091,0),quat=_quat_y(-beta))
 hinge(upbal,'upper_balance_hinge');link(upbal,p['balance_mass_kg'],[0],.009,'.48 .55 .59 1')
 _add(upbal,'site',name=n('upper_balance_pin'),pos=_vec(L,0,0),size='.004')
 _add(upbal,'site',name=n('right_top_pin'),pos=_vec(L,0,0),size='.003')
 _add(equality,'connect',name=n('upper_parallelogram'),site1=n('upper_head_pin'),site2=n('upper_balance_pin'),solref='.002 1',solimp='.99 .9999 .00001')
 # This static mechanical ratio preserves the equal-fold relationship in B2.
 # The actual inter-stage transmission is not known from the source.
 _add(equality,'joint',name=n('assumed_passive_fold_synchronizer'),joint1=n('upper_hinge'),joint2=n('lower_hinge'),polycoef='0 -1 0 0 0',solref='.002 1',solimp='.99 .9999 .00001')
 l0=cylinder_length_from_angle(th,p)
 cyl=_add(tendon,'spatial',name=n('cylinder'),width='.010',stiffness=p['return_stiffness_Npm'],springlength=l0,damping=0,rgba='.60 .66 .69 1')
 _add(cyl,'site',site=n('cylinder_base'));_add(cyl,'site',site=n('cylinder_rod'))
 # RETRACTION chamber: +F acts to shorten tendon, positive gas travel=l0-l.
 _add(actuators,'general',name=n('cylinder_force'),tendon=n('cylinder'),gear='-1',dyntype='none',gaintype='fixed',gainprm=1,biastype='none',ctrllimited='true',ctrlrange=_vec(-p['cylinder_max_force_N'],p['cylinder_max_force_N']))
 return PantographSpec(prefix,p,n('cylinder_force'),n('cylinder'),n('lower_hinge'),n('collector'),n('head_center'),n('collector_contact'),l0,l0-cylinder_length_from_angle(p['max_angle_rad'],p),p['minimum_lift_m']+p['head_contact_offset_z_m'])


def add_overhead_rail(worldbody, underside_z, prefix='panto', center_x=0., half_length=5., center_y=0., params=None):
    """Static physical box wire; underside_z is world z. No mocap/trajectory."""
    p = PARAMS | (params or {})
    return _add(worldbody,'geom',name=f'{prefix}_overhead_rail',type='box',pos=_vec(center_x,center_y,underside_z+.01),size=_vec(half_length,.012,.01),contype=p['collision_bit'],conaffinity=p['collision_bit'],condim=3,friction=_vec(p['contact_friction'],.001,.0001),solref='.004 1',solimp='.95 .99 .0005',rgba='.77 .42 .16 1')


def standalone_xml(timestep=.0002, rail_z=1.26, base_z=1., params=None):
    root=Element('mujoco',model='B8_source_B2_Z_pantograph_force_diagnostic')
    _add(root,'compiler',angle='radian',autolimits='true',inertiafromgeom='false')
    _add(root,'option',timestep=timestep,gravity='0 0 -9.81',integrator='implicitfast',solver='Newton',iterations=100,tolerance='1e-10',cone='elliptic')
    visual=_add(root,'visual');_add(visual,'global',offwidth=960,offheight=640)
    world=_add(root,'worldbody');_add(world,'light',pos='0 -2 3',dir='0 .4 -1',directional='true')
    _add(world,'geom',type='plane',size='2 2 .1',contype=0,conaffinity=0,rgba='.15 .18 .22 1')
    roof=_add(world,'body',name='roof',pos=_vec(0,0,base_z))
    _add(roof,'geom',type='box',size='.7 .3 .025',pos='0 0 -.07',contype=0,conaffinity=0,rgba='.15 .45 .65 1')
    eq=_add(root,'equality');ten=_add(root,'tendon');act=_add(root,'actuator')
    spec=add_pantograph(roof,world,eq,ten,act,params=params)
    if rail_z is not None:add_overhead_rail(world,rail_z,params=params)
    indent(root)
    return tostring(root,encoding='unicode'),spec
