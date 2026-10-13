"""GTSD newly constructed bogie geometry from 1011 CAD and real photographs.

Interface: build(cfg) -> list of newly created Blender objects.
This module does not clear scenes, import old assets, add cameras, render/export,
or define any mass, joint, contact, force law, stiffness, damping or animation.
All dimensions are metres. Source references and uncertainty travel with objects.
"""
import math
import bpy
from mathutils import Vector
from geom_helpers import material, box, rod, sphere, finish, tags

VERSION = 'gtsd-bogie-cad-rail-display-20261011-v3'
CAD = 'cad_geometry_verified_v1.json:4AF7C;INSERT 526A4/4AF90/4AF91'
ARM_CAD = 'cad_motion_candidates_v1.json:axlebox_arm_center_pair:C2B/C4A;scale_clarification:Model_scale=1'
PHOTO = 'image_inventory.json:25;转向架侧面.jpg;bogie_mechanisms_crop.png'
PRIMARY = 'image_inventory.json:19/20/21;primary_spring_crop.png;left_axle_arm_crop.png'

# Added parameters are visibly inferred, not physical metrology. Names deliberately
# avoid rolling radius, track gauge, calibrated joint, or certified damper type.
PARAMETERS = {
    'wheel_visual_axial_extent_m': [-.030,.043],
    'wheel_flat_display_tread_axial_range_m': [-.007,.038],
    'reference_build_wheel_y_m': .300,
    'reference_build_axle_z_m': .16225,
    'wheel_bore_visual_radius_m': .026,
    'wheel_hub_visual_radius_m': .056,
    'wheel_disc_visual_radius_m': .127,
    'axle_visual_radius_m': .022,
    'axle_half_span_m': .390,
    'axlebox_center_abs_y_m': .372,
    'axlebox_housing_visual_radius_m': .047,
    'axlebox_housing_y_depth_m': .080,
    'axlebox_cap_visual_radius_m': .040,
    'arm_projected_delta_x_m': .16000031999186467,
    'arm_projected_delta_z_m': .0013333359994503553,
    'arm_projected_center_distance_m': .1600058754620796,
    'arm_pivot_outer_visual_radius_m': .030,
    'arm_pivot_inner_visual_radius_m': .010,
    'arm_web_y_depth_m': .032,
    'primary_spring_center_abs_y_m': .363,
    'primary_spring_coil_center_radius_m': .044,
    'primary_spring_wire_radius_m': .0068,
    'primary_spring_visual_turns': 2.25,
    'primary_spring_coil_bottom_z_m': .232,
    'primary_spring_coil_top_z_m': .299,
    'primary_spring_seat_radius_m': .056,
    'primary_spring_skirt_radius_m': .051,
    'primary_spring_skirt_bottom_z_m': .225,
    'primary_spring_skirt_top_z_m': .255,
    'primary_damper_outward_x_offset_m': .075,
    'primary_damper_center_abs_y_m': .389,
    'primary_damper_bottom_z_m': .170,
    'primary_damper_top_z_m': .323,
    'primary_damper_body_radius_m': .014,
    'primary_lower_transition_local_outward_u_dz_m': [[.034,.004],[.058,.004],[.088,.010],[.088,.024],[.065,.027],[.034,.018]],
    'primary_lower_transition_center_abs_y_m': .381,
    'primary_lower_transition_y_depth_m': .029,
    'frame_side_center_abs_y_m': .266,
    'frame_side_y_depth_m': .054,
    'frame_side_outline_xz_m': [
        [-.535,.298],[-.465,.332],[-.355,.325],[-.197,.250],
        [.197,.250],[.355,.325],[.465,.332],[.535,.298],
        [.535,.264],[.393,.264],[.203,.199],[-.203,.199],
        [-.393,.264],[-.535,.264]
    ],
    'secondary_bellows_count_displayed': 1,
    'secondary_bellows_center_xy_m': [0.,0.],
    'secondary_bellows_max_visual_radius_m': .148,
    'secondary_bellows_bottom_z_m': .264,
    'secondary_bellows_top_z_m': .355,
    'visible_side_y_sign': -1,
    'visible_side_mechanism_abs_y_m': .416,
    'longitudinal_tube_count_displayed': 2,
    'longitudinal_tube_endpoint_x_m': [-.178,.164],
    'longitudinal_tube_center_z_m': [.115,.143],
    'longitudinal_tube_body_radius_m': .014,
    'central_vertical_tube_count_displayed': 1,
    'central_vertical_tube_x_m': .205,
    'central_vertical_tube_bottom_z_m': .096,
    'central_vertical_tube_top_z_m': .339,
    'central_vertical_tube_body_radius_m': .019,
    'ball_end_link_count_displayed': 1,
    'ball_end_link_bottom_xz_m': [.261,.118],
    'ball_end_link_top_xz_m': [.247,.352],
    'ball_end_link_shaft_radius_m': .008,
    'ball_end_link_eye_visual_radius_m': .018,
    'ball_link_lower_mount_outline_xz_m': [[.202,.073],[.239,.073],[.279,.101],[.283,.117],[.276,.134],[.255,.134],[.241,.121],[.218,.110],[.201,.099]],
    'ball_link_lower_mount_center_abs_y_m': .400,
    'ball_link_lower_mount_y_depth_m': .020,
    'ball_link_lower_mount_boss_radius_m': .016,
    'near_side_hanging_plate_center_x_m': -.218,
    'near_side_hanging_plate_y_depth_m': .010,
}


def _mesh(name, verts, faces, mat, smooth=False, bevel=0):
    mesh=bpy.data.meshes.new(name+'_mesh');mesh.from_pydata(verts,[],faces);mesh.update()
    obj=bpy.data.objects.new(name,mesh);bpy.context.collection.objects.link(obj)
    finish(obj,name,mat)
    if smooth:
        for p in mesh.polygons:p.use_smooth=True
    if bevel:
        mod=obj.modifiers.new('Small cast edge radius','BEVEL');mod.width=bevel;mod.segments=2
        obj.modifiers.new('Weighted cast surface normals','WEIGHTED_NORMAL')
    return obj


def _prism(name, xz, y, depth, mat, bevel=.0015):
    xz=list(xz)
    area=sum(xz[i][0]*xz[(i+1)%len(xz)][1]-xz[(i+1)%len(xz)][0]*xz[i][1] for i in range(len(xz)))
    if area<0:xz.reverse()
    n=len(xz);verts=[(x,y-depth/2,z) for x,z in xz]+[(x,y+depth/2,z) for x,z in xz]
    faces=[tuple(range(n)),tuple(range(2*n-1,n-1,-1))]
    faces += [((i+1)%n,i,i+n,(i+1)%n+n) for i in range(n)]
    return _mesh(name,verts,faces,mat,bevel=bevel)


def _lathe_y(name, x, y, z, profile, mat, segments=64, side=1):
    """Closed sectional profile revolved about transverse Y, radius in X-Z."""
    verts=[]
    for axial,r in profile:
        verts += [(x+r*math.cos(i*2*math.pi/segments),y+side*axial,z+r*math.sin(i*2*math.pi/segments)) for i in range(segments)]
    faces=[];n=len(profile)
    for k in range(n):
        for i in range(segments):
            a=k*segments+i;b=k*segments+(i+1)%segments
            c=((k+1)%n)*segments+(i+1)%segments;d=((k+1)%n)*segments+i
            faces.append((d,c,b,a) if side>0 else (a,b,c,d))
    return _mesh(name,verts,faces,mat,smooth=True)


def _lathe_z(name, x, y, profile, mat, segments=56):
    verts=[]
    for z,r in profile:
        verts += [(x+r*math.cos(i*2*math.pi/segments),y+r*math.sin(i*2*math.pi/segments),z) for i in range(segments)]
    faces=[];n=len(profile)
    for k in range(n):
        faces += [(k*segments+i,k*segments+(i+1)%segments,((k+1)%n)*segments+(i+1)%segments,((k+1)%n)*segments+i) for i in range(segments)]
    return _mesh(name,verts,faces,mat,smooth=True)


def _tube_mesh(name, points, radius, mat, radial=8):
    """Mesh-based small pipe / spring; caps and genuine round cross-section."""
    pp=[Vector(p) for p in points];verts=[]
    for i,p in enumerate(pp):
        t=(pp[min(i+1,len(pp)-1)]-pp[max(0,i-1)]).normalized()
        ref=Vector((0,0,1)) if abs(t.z)<.95 else Vector((0,1,0))
        u=t.cross(ref).normalized();v=t.cross(u).normalized()
        for j in range(radial):
            a=2*math.pi*j/radial;verts.append(tuple(p+radius*(math.cos(a)*u+math.sin(a)*v)))
    faces=[tuple(range(radial-1,-1,-1))]
    for k in range(len(pp)-1):
        faces += [(k*radial+i,k*radial+(i+1)%radial,(k+1)*radial+(i+1)%radial,(k+1)*radial+i) for i in range(radial)]
    faces.append(tuple(range((len(pp)-1)*radial,len(pp)*radial)))
    return _mesh(name,verts,faces,mat,smooth=True)


def build(cfg):
    p=dict(PARAMETERS)
    # Shared assembly anchors are consumed directly, without overall scaling.
    ax=cfg['axle_distance_m']/2
    # All hard-coded heights below belong to the original CAD-linked template.
    # Construct once in that datum, then translate EVERY object rigidly in Z.
    az=p['reference_build_axle_z_m'];wy=p['reference_build_wheel_y_m']
    dz=cfg.get('bogie_vertical_datum_shift_m',cfg['wheel_axis_height_m']-az)
    transverse_shift=cfg.get('bogie_transverse_shift_m',cfg['wheel_center_y_m']-wy)
    if abs(az+dz-cfg['wheel_axis_height_m'])>1e-8:
        raise ValueError('Bogie Z-datum shift disagrees with axle-height contract')
    if abs(wy+transverse_shift-cfg['wheel_center_y_m'])>1e-8:
        raise ValueError('Bogie side shift disagrees with wheel-reference contract')
    outer=cfg['wheel_visual_outer_radius_m'];inner=cfg['wheel_visual_inner_ring_radius_m']
    visible=-1 if cfg.get('bogie_visible_side_y_sign',p['visible_side_y_sign'])<0 else 1
    objects=[]
    cast=material('GTSD bogie graphite grey cast steel',(.070,.088,.098,1),.12,.52)
    edge=material('GTSD bogie dark frame',(.035,.045,.054,1),.10,.54)
    machined=material('GTSD bogie turned steel',(.53,.56,.59,1),.87,.23)
    tread=material('GTSD bogie polished wheel contours',(.69,.72,.73,1),.94,.20)
    wheel_face=material('GTSD bogie wheel recessed face',(.065,.073,.080,1),.62,.41)
    black=material('GTSD bogie black spring steel',(.035,.041,.047,1),.64,.28)
    rubber=material('GTSD bogie secondary bellows rubber',(.024,.029,.033,1),.03,.64)
    bolts=material('GTSD bogie zinc fasteners',(.65,.67,.65,1),.88,.25)

    def keep(obj, group, source=PHOTO, evidence='PHOTO_INFERRED_GEOMETRY', registry='DTB-C0069', notes=''):
        tags(obj,evidence,source,registry)
        obj['evidence_level']=evidence;obj['geometry_group']=group;obj['geometry_version']=VERSION
        obj['functional_calibration']='STATIC_VISUAL_ONLY; no mechanics or calibrated rolling contact'
        if notes:obj['geometry_assumptions']=notes
        objects.append(obj);return obj
    def B(name,c,s,m=cast,group='frame',source=PHOTO,registry='DTB-C0066',bevel=.002,notes=''):
        return keep(box('bogie_'+name,c,s,m,bevel),group,source,registry=registry,notes=notes)
    def R(name,a,b,r,m=machined,group='fasteners',source=PHOTO,registry='DTB-C0069',vertices=24,notes=''):
        return keep(rod('bogie_'+name,a,b,r,m,vertices),group,source,registry=registry,notes=notes)
    def bolt_y(name,x,y,z,side=1,r=.0036,length=.004,registry='DTB-C0069'):
        R(name+'_washer',(x,y,z),(x,y+side*.0014,z),r*1.35,bolts,registry=registry,vertices=16)
        return R(name+'_hex',(x,y+side*.0013,z),(x,y+side*(length+.0013),z),r,bolts,registry=registry,vertices=6)
    def ring_y(name,x,y,z,ro,ri,depth,m=machined,group='bearing_visual',source=PHOTO,registry='DTB-C0069'):
        prof=[(-depth/2,ri),(-depth/2,ro),(depth/2,ro),(depth/2,ri)]
        return keep(_lathe_y('bogie_'+name,x,y,z,prof,m,40),group,source,registry=registry)
    def eye_y(name,x,y,z,ro=.012,ri=.005,depth=.014,m=machined,group='damper_eye'):
        ring_y(name,x,y,z,ro,ri,depth,m,group)
        R(name+'_pin',(x,y-depth*.8,z),(x,y+depth*.8,z),ri*.82,bolts,group)

    # Two independent transverse axles and four genuinely profiled wheels.
    # Only the two contour radii and X axle spacing are CAD anchors; the Y
    # section, bore, hub and dish are deliberately modeled from photographs.
    section=[(-.030,.026),(-.030,.056),(-.021,.064),(-.019,.127),
             (-.026,inner),(-.023,outer),(-.010,outer),(-.007,inner),
             (.038,inner),(.043,inner-.004),(.040,.129),(.025,.125),
             (.013,.062),(.024,.056),(.027,.026)]
    for ia,sx in enumerate((-1,1),1):
        x=sx*ax
        R('axle_%d'%ia,(x,-(p['axle_half_span_m']+transverse_shift),az),(x,p['axle_half_span_m']+transverse_shift,az),p['axle_visual_radius_m'],machined,'wheelset',CAD,'DTB-C0067',40,
          'Y span, section and axle height inferred; source circles do not certify rolling radius')
        for side in (-1,1):
            suffix=f'{ia}_{"near" if side==visible else "far"}'
            wheel=keep(_lathe_y('bogie_wheel_'+suffix,x,side*wy,az,section,wheel_face,72,side),'wheelset',CAD+';'+PRIMARY,'CAD_2D_OUTLINE_WITH_PHOTO_INFERRED_DEPTH','DTB-C0067',
                'Two CAD concentric visual radii; stepped Y profile, flange-like shoulder and wheel width are inferred, not measured contact geometry')
            wheel.data.materials.append(tread);wheel.data.materials.append(machined)
            for poly in wheel.data.polygons:
                band=poly.index//72
                poly.material_index=1 if band in (4,5,6,7,8) else (2 if band in (0,1,12,13,14) else 0)
            wheel['cad_outer_visual_radius_m']=outer;wheel['cad_inner_visual_radius_m']=inner
            wheel['axle_center_x_m']=x;wheel['axle_center_z_m']=az
            # Subtle turned concentric score lines are actual geometry.
            for j,rr in enumerate((.129,.135)):
                ring_y('wheel_face_score_'+suffix+str(j),x,side*(wy+.0404),az,rr+.0007,rr-.0007,.0012,tread,'wheel_turning_marks',PRIMARY,'DTB-C0067')
            R('wheel_inboard_collar_'+suffix,(x,side*(wy-.055),az),(x,side*(wy-.027),az),.031,machined,'wheelset',PRIMARY,'DTB-C0067',32)

    # Cast side frames, cross-members and gussets form an open underframe.
    # The outline is traced to broad photo features, not an unverified CAD solid.
    for side in (-1,1):
        y=side*p['frame_side_center_abs_y_m']
        keep(_prism('bogie_frame_side_'+str(side),p['frame_side_outline_xz_m'],y,p['frame_side_y_depth_m'],cast),'frame',PHOTO,registry='DTB-C0066')
        B('frame_edge_rail_'+str(side),(0,y,.246),(.405,.067,.015),edge)
        for sx in (-1,1):
            B(f'primary_top_bridge_{sx}_{side}',(sx*ax,side*.326,.313),(.146,.147,.017),cast)
            B(f'primary_saddle_roof_{sx}_{side}',(sx*ax,side*.364,.326),(.138,.110,.022),cast)
            # Upstanding end cheeks visible around each spring.
            for dx in (-.064,.064):
                B(f'primary_top_cheek_{sx}_{side}_{dx}',(sx*ax+dx,side*.362,.290),(.011,.099,.065),cast)
            # End suspension mounting posts meet the body bottom.
            B(f'body_mount_post_{sx}_{side}',(sx*ax,side*.278,.350),(.050,.051,.043),machined)
            B(f'body_mount_flange_{sx}_{side}',(sx*ax,side*.278,.372),(.095,.080,.006),cast)
            for dx in (-.030,.030):
                R(f'body_mount_fastener_{sx}_{side}_{dx}',(sx*ax+dx,side*.278,.368),(sx*ax+dx,side*.278,.378),.004,bolts,'fasteners',registry='DTB-C0066',vertices=6)
        for sx in (-1,1):
            # Stiffening rib follows the sloping cast side-frame zone.
            pts=[(sx*.206,y+side*.031,.240),(sx*.285,y+side*.031,.275),(sx*.368,y+side*.031,.310)]
            keep(_tube_mesh('bogie_frame_weld_rib_'+str(side)+'_'+str(sx),pts,.0038,edge),'frame',PHOTO,registry='DTB-C0066')
    for x in (-.194,.194):
        B('crossmember_'+str(x),(x,0,.228),(.052,.590+2*transverse_shift,.046),edge)
    B('central_transverse_bolster',(0,0,.243),(.365,.650+2*transverse_shift,.041),cast)
    for x in (-.149,.149):
        B('bolster_edge_'+str(x),(x,0,.265),(.013,.620+2*transverse_shift,.014),edge)

    # Four axle boxes and short inward arms. The source's 160.005875 mm is
    # preserved as a projected shape constraint only, never a real 3D hinge.
    for ia,sx in enumerate((-1,1),1):
        x=sx*ax;px=x-sx*p['arm_projected_delta_x_m'];pz=az+p['arm_projected_delta_z_m']
        for side in (-1,1):
            suffix=f'{ia}_{side}';y=side*p['axlebox_center_abs_y_m']
            d=p['axlebox_housing_y_depth_m']
            R('axlebox_housing_'+suffix,(x,y-d/2,az),(x,y+d/2,az),p['axlebox_housing_visual_radius_m'],cast,'axlebox_arm',ARM_CAD+';'+PRIMARY,vertices=32)
            # Square-bodied cast saddle behind the circular cover.
            B('axlebox_cast_body_'+suffix,(x,y,az+.012),(.091,.074,.085),cast,'axlebox_arm',PRIMARY,'DTB-C0069',.006)
            capy=y+side*(d/2+.002)
            R('axlebox_round_cap_'+suffix,(x,capy-side*.006,az),(x,capy+side*.004,az),.040,cast,'axlebox_arm',ARM_CAD+';'+PRIMARY,vertices=48,
                notes='C2B 40 mm 2D visible circle used for cover outline; physical cover identity remains inferred')
            ring_y('axlebox_cap_lip_'+suffix,x,capy-side*.005,az,.043,.039,.005,edge,'axlebox_arm',PRIMARY)
            for j in range(6):
                a=math.pi/6+j*math.tau/6
                bolt_y(f'axlebox_cover_bolt_{suffix}_{j}',x+.042*math.cos(a),capy+side*.004,az+.042*math.sin(a),side)
            # Arm polygon is local u inward from the wheel axis.
            local=[(.021,.037),(.056,.025),(.091,.023),(.128,.022),(.154,.025),
                   (.172,.018),(.178,-.015),(.144,-.020),(.100,-.019),(.060,-.021),(.039,-.031),(.020,-.028)]
            armxz=[(x-sx*u,az+dz) for u,dz in local]
            keep(_prism('bogie_axlebox_arm_web_'+suffix,armxz,y,.032,cast,.002),'axlebox_arm',ARM_CAD+';'+PRIMARY,'CAD_2D_CANDIDATE_WITH_PHOTO_INFERRED_DEPTH',notes='Pivot assignment/3D axis not validated; candidate projection copied without dynamics')
            ring_y('arm_inner_eye_'+suffix,px,y,pz,.030,.010,.040,cast,'axlebox_arm',ARM_CAD)
            R('arm_eye_bushing_'+suffix,(px,y-side*.026,pz),(px,y+side*.026,pz),.010,machined,'axlebox_arm',ARM_CAD,vertices=24)
            # Upper triangular stiffener, visually separate from arm web.
            rib=[(x-sx*.026,az+.067),(x-sx*.038,az+.067),(x-sx*.139,az+.026),(x-sx*.125,az+.023)]
            keep(_prism('bogie_arm_upper_gusset_'+suffix,rib,y+side*.015,.009,cast),'axlebox_arm',PRIMARY)
            B('spring_lower_saddle_'+suffix,(x,y,.214),(.121,.100,.014),cast,'primary_suspension',PRIMARY,'DTB-C0069',.003)
            # Primary suspension: visible steel helical spring above a skirt.
            sy=side*p['primary_spring_center_abs_y_m']
            R('primary_lower_seat_'+suffix,(x,sy,.219),(x,sy,.226),.056,cast,'primary_spring',PRIMARY,vertices=40)
            R('primary_skirt_'+suffix,(x,sy,.225),(x,sy,.255),.051,black,'primary_spring',PRIMARY,vertices=48)
            for zz in (.228,.252):
                keep(_lathe_z('bogie_primary_skirt_bead_'+suffix+'_'+str(zz),x,sy,[(zz-.0014,.049),(zz-.0014,.053),(zz+.0014,.053),(zz+.0014,.049)],machined,40),'primary_spring',PRIMARY)
            points=[];count=100
            for i in range(count):
                t=i/(count-1);a=math.tau*p['primary_spring_visual_turns']*t
                points.append((x+.044*math.cos(a),sy+.044*math.sin(a),.232+(.299-.232)*t))
            keep(_tube_mesh('bogie_primary_helical_spring_'+suffix,points,.0068,black,10),'primary_spring',PRIMARY,notes='Photo-visible spiral reconstructed with assumed 2.25 turns and wire diameter; no spring rate')
            R('primary_top_seat_'+suffix,(x,sy,.303),(x,sy,.309),.056,cast,'primary_spring',PRIMARY,vertices=40)
            # Independent vertical primary damper mounted outside the coil.
            dx=x+sx*.075;dy=side*.389
            # Cast transition closes the visible lower-foot/axlebox gap. Its
            # shape and out-of-plane thickness remain photographic inference.
            transition=[(x+sx*u,az+dz) for u,dz in p['primary_lower_transition_local_outward_u_dz_m']]
            keep(_prism('bogie_primary_damper_lower_transition_'+suffix,transition,side*p['primary_lower_transition_center_abs_y_m'],p['primary_lower_transition_y_depth_m'],cast,.002),'primary_damper',PRIMARY,notes='Photo-inferred cast connection between axlebox and independent damper lower foot; not a verified structural or joint dimension')
            B('primary_damper_lower_ear_'+suffix,(dx,dy,.176),(.030,.028,.014),cast,'primary_damper',PRIMARY,'DTB-C0069')
            B('primary_damper_upper_ear_'+suffix,(dx,dy,.322),(.026,.031,.017),cast,'primary_damper',PRIMARY,'DTB-C0069')
            eye_y('primary_damper_bottom_eye_'+suffix,dx,dy,.180,.010,.004,.020,cast,'primary_damper')
            eye_y('primary_damper_top_eye_'+suffix,dx,dy,.314,.010,.004,.020,cast,'primary_damper')
            R('primary_damper_piston_'+suffix,(dx,dy,.189),(dx,dy,.252),.0055,machined,'primary_damper',PRIMARY)
            R('primary_damper_barrel_'+suffix,(dx,dy,.232),(dx,dy,.306),.014,black,'primary_damper',PRIMARY,vertices=32)
            R('primary_damper_gland_'+suffix,(dx,dy,.229),(dx,dy,.237),.0155,cast,'primary_damper',PRIMARY,vertices=24)

    # Exactly one central bellows is shown; the photographed one cannot justify
    # doubling an unseen opposite-side unit. Its depth placement is provisional.
    profile=[(.264,.030),(.264,.126),(.269,.133),(.277,.141),(.285,.147),
             (.293,.148),(.300,.145),(.307,.140),(.314,.142),(.321,.144),
             (.329,.141),(.336,.137),(.345,.130),(.351,.128),(.355,.120),(.355,.030)]
    keep(_lathe_z('bogie_central_secondary_bellows',0,0,profile,rubber,72),'secondary_bellows',PHOTO,notes='One visible unit reconstructed centrally; circular plan and unseen depth/total count not validated')
    for zz,rr in ((.284,.147),(.294,.148),(.305,.143),(.316,.144),(.328,.141)):
        keep(_lathe_z('bogie_secondary_bellows_moulded_rib_'+str(zz),0,0,[(zz-.001,rr-.002),(zz-.001,rr+.002),(zz+.001,rr+.002),(zz+.001,rr-.002)],rubber,64),'secondary_bellows',PHOTO)
    R('secondary_bottom_pedestal',(0,0,.252),(0,0,.269),.129,cast,'secondary_bellows',PHOTO,vertices=64)
    R('secondary_upper_plate',(0,0,.350),(0,0,.359),.143,cast,'secondary_bellows',PHOTO,vertices=64)
    R('secondary_upper_spigot',(0,0,.358),(0,0,.373),.088,edge,'secondary_bellows',PHOTO,vertices=48)
    # Broad shallow mounting tray and visible bolts remain open at the sides.
    tray=[(-.180,.262),(-.150,.249),(-.055,.240),(.055,.240),(.150,.249),(.180,.262),(.145,.269),(-.145,.269)]
    for side in (-1,1):
        keep(_prism('bogie_secondary_tray_edge_'+str(side),tray,side*.156,.012,cast),'secondary_bellows',PHOTO)
        for x in (-.110,0,.110):
            R('secondary_tray_bolt_'+str(side)+'_'+str(x),(x,side*.151,.263),(x,side*.151,.273),.0042,bolts,'fasteners',vertices=6)

    # The near-side suspension bracket is a perforated plate, with true openings
    # assembled from solid webs rather than dark painted rectangles.
    yy=visible*.400;cx=-.218;dep=.010
    B('hanging_bracket_top_flange',(cx,yy,.344),(.150,dep,.022),cast,'visible_secondary_brackets',PHOTO,'DTB-C0069')
    for xx in (cx-.039,cx+.039):
        B('hanging_bracket_tall_web_'+str(xx),(xx,yy,.261),(.018,dep,.155),cast,'visible_secondary_brackets',PHOTO,'DTB-C0069')
    for zz in (.095,.126,.158,.188):
        B('hanging_bracket_lower_cross_'+str(zz),(cx,yy,zz),(.096,dep,.010),cast,'visible_secondary_brackets',PHOTO,'DTB-C0069')
    for xx in (cx-.043,cx-.014,cx+.015,cx+.043):
        B('hanging_bracket_lower_stile_'+str(xx),(xx,yy,.142),(.010,dep,.095),cast,'visible_secondary_brackets',PHOTO,'DTB-C0069')
    B('hanging_bracket_upper_cross',(cx,yy,.335),(.095,dep,.012),cast,'visible_secondary_brackets',PHOTO,'DTB-C0069')
    # Upper-frame bridge along the photographed side, not a filled side wall.
    B('visible_side_upper_bridge',(-.070,yy,.351),(.526,.025,.032),edge,'visible_secondary_brackets',PHOTO,'DTB-C0066')
    # Curved lower clevis profile at right of the two longitudinal tubes.
    right_outline=[(.158,.173),(.190,.187),(.219,.161),(.231,.108),(.230,.073),(.201,.068),(.174,.089),(.161,.112)]
    keep(_prism('bogie_right_lower_clevis',right_outline,visible*.406,.032,cast,.003),'visible_secondary_brackets',PHOTO)
    B('right_vertical_anchor',(.185,visible*.383,.248),(.032,.035,.173),cast,'visible_secondary_brackets',PHOTO,'DTB-C0069')
    for z in (.115,.143):
        y=visible*p['visible_side_mechanism_abs_y_m']
        group='visible_longitudinal_tubes';label=str(z)
        R('longitudinal_exposed_rod_'+label,(-.174,y,z),(.040,y,z),.0065,machined,group,PHOTO)
        R('longitudinal_black_barrel_'+label,(.018,y,z),(.157,y,z),.014,black,group,PHOTO,vertices=32,
          notes='Visible longitudinal tube/rod shape; identity as yaw damper, actuator or other device is unconfirmed')
        R('longitudinal_gland_'+label,(.012,y,z),(.024,y,z),.0155,edge,group,PHOTO)
        for ex in (-.178,.164):
            eye_y('longitudinal_eye_'+label+'_'+str(ex),ex,y,z,.010,.004,.017,cast,group)
            bolt_y('longitudinal_end_bolt_'+label+'_'+str(ex),ex,y+visible*.010,z,visible,.0036)

    # Independent visible central vertical tube and adjustable ball-end link.
    y=visible*.424;x=.205;group='visible_central_vertical_tube'
    eye_y('central_vertical_top_eye',x,y,.339,.012,.005,.021,cast,group)
    eye_y('central_vertical_bottom_eye',x+.010,y,.096,.012,.005,.021,cast,group)
    R('central_vertical_piston',(x,y,.286),(x,y,.331),.0065,machined,group,PHOTO)
    R('central_vertical_barrel',(x+.003,y,.116),(x,y,.290),.019,black,group,PHOTO,vertices=32)
    R('central_vertical_gland',(x,y,.286),(x,y,.296),.021,edge,group,PHOTO)
    B('central_vertical_lower_foot',(.215,y,.080),(.058,.042,.015),cast,group,PHOTO,'DTB-C0069')
    # Round upper mount plate with distinct six fasteners.
    R('central_vertical_round_upper_plate',(x,visible*.394,.358),(x,visible*.419,.358),.027,cast,group,PHOTO,vertices=40)
    for j in range(6):
        a=math.tau*j/6
        bolt_y('central_vertical_mount_bolt_'+str(j),x+.022*math.cos(a),visible*.420,.358+.022*math.sin(a),visible,.0025)
    # Extend the lower clevis into a visible load-path-shaped mounting ear.
    # This supports the eye pin without pretending to establish its real DOF.
    keep(_prism('bogie_ball_link_lower_cast_mount',p['ball_link_lower_mount_outline_xz_m'],visible*p['ball_link_lower_mount_center_abs_y_m'],p['ball_link_lower_mount_y_depth_m'],cast,.002),'visible_adjustable_ball_end_link',PHOTO,notes='Photo-inferred lower cast mounting ear; closes visual gap between right clevis and ball-link pin; actual joint topology unverified')
    ring_y('ball_link_lower_mount_boss',.261,visible*.410,.118,p['ball_link_lower_mount_boss_radius_m'],.0065,.012,cast,'visible_adjustable_ball_end_link',PHOTO)
    a=Vector((.261,visible*.427,.118));b=Vector((.247,visible*.427,.352));v=b-a
    group='visible_adjustable_ball_end_link'
    R('ball_link_main_shaft',a+v*.13,a+v*.87,.008,machined,group,PHOTO,
      notes='Silver adjustable double ball-end rod seen in photo; function and kinematic connection unconfirmed')
    R('ball_link_adjuster',a+v*.39,a+v*.60,.0105,machined,group,PHOTO,vertices=12)
    for t in (.27,.36,.63,.72):
        R('ball_link_locknut_'+str(t),a+v*t,a+v*(t+.035),.013,bolts,group,PHOTO,vertices=6)
    for t in (.2,.225,.25,.675,.70,.725,.75):
        R('ball_link_thread_'+str(t),a+v*t,a+v*(t+.007),.0093,machined,group,PHOTO,vertices=24)
    for label,pos in (('lower',a),('upper',b)):
        eye_y('ball_link_'+label+'_eye',pos.x,pos.y,pos.z,.018,.009,.016,machined,group)
        keep(sphere('bogie_ball_link_'+label+'_spherical_inner',pos,.010,bolts),group,PHOTO)
        R('ball_link_'+label+'_mount_pin',(pos.x,pos.y-visible*.025,pos.z),(pos.x,pos.y+visible*.026,pos.z),.006,bolts,group,PHOTO)
        bolt_y('ball_link_'+label+'_end_nut',pos.x,pos.y+visible*.024,pos.z,visible,.008,length=.009)

    # Two photo-supported flexible leads at the known near-side axleboxes. These
    # are exterior installation representations, with no invented inner sensor.
    for ia,sx in enumerate((-1,1),1):
        x=sx*ax;y=visible*.403;registry='DTB-C0071' if ia==1 else 'DTB-C0072'
        R('axle_temperature_probe_'+str(ia),(x+sx*.025,y,.142),(x+sx*.025,y,.119),.0034,machined,'axle_temperature_probe',PRIMARY,registry)
        pts=[(x+sx*.025,y,.120),(x+sx*.003,y-visible*.007,.096),(x-sx*.053,y-visible*.012,.103),
             (x-sx*.089,y-visible*.005,.164),(x-sx*.109,visible*.384,.236),(x-sx*.11,visible*.357,.311)]
        keep(_tube_mesh('bogie_axle_probe_flexible_lead_'+str(ia),pts,.0024,black,8),'axle_temperature_probe',PRIMARY,registry=registry,
            notes='Only exterior route shown; inferred cable slack/depth and probe insertion not measured')
    # Side assemblies are translated as rigid pieces. No whole-car Y scale
    # or wheel-section distortion is used. Centered bellows/tray keep size.
    bpy.context.view_layer.update()
    for obj in objects:
        center_y=sum((obj.matrix_world@v.co).y for v in obj.data.vertices)/len(obj.data.vertices)
        side_shift=(1 if center_y>0 else -1)*transverse_shift if abs(center_y)>.180 else 0.0
        obj.location.y += side_shift
        obj.location.z += dz
        obj['display_z_datum_shift_m']=dz
        obj['display_rigid_side_shift_y_m']=side_shift
        obj['display_alignment_status']='GEOMETRY_ONLY; rail section roles and rolling contour remain uncalibrated'
        if obj.name.startswith('bogie_wheel_') and obj.get('geometry_group')=='wheelset' and 'collar' not in obj.name:
            obj['axle_center_z_m']=cfg['wheel_axis_height_m']
            obj['wheel_reference_abs_y_m']=cfg['wheel_center_y_m']
            obj['display_flat_tread_axial_range_m']=[-.007,.038]
            obj['display_tread_contour_radius_m']=inner
            obj['source_refs']=obj['source_refs']+';environment_source_crosscheck.json:LINE4ED11/13/15/17 (rail contour center candidate only)'
    return objects
