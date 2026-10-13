"""Photo-led GTSD pantograph reconstruction, metres; no dynamics or animation.

Sources actually inspected on 2026-10-11: 受电弓.jpg, 图片2.png,
图片3.png, 图片19.png, both pantograph detail crops, CAD renders 05/10.
The 0.819998716 m lower-link length is a CAD 2D candidate transferred into
this visual model. Its assignment to a physical link is not calibrated.
All depths, cross sections, mounting offsets and raised angles are explicit
PHOTO_INFERRED_GEOMETRY. No old mesh or old actuator law is consumed.
"""
import math
import bpy
from mathutils import Vector
from geom_helpers import material, finish, box, rod, tube_path, tags

PHOTO = 'PHOTO_INFERRED_GEOMETRY'
SOURCE_PHOTO = '图片素材/受电弓.jpg;图片素材/图片2.png;图片素材/图片3.png;图片素材/图片19.png'
SOURCE_CAD = 'cad_motion_candidates_v1.json:folded_pantograph_main_link_pair;cad_motion_candidates_v1_scale_clarification.json;renders/05_pantograph_folded_side.png;renders/10_pantograph_main_link_local.png'
LOWER_LINK_CANDIDATE_M = 0.8199987164238119


def parameters(cfg):
    """Separate display-pose parameters, never a solved mechanism state."""
    p = dict(cfg.get('pantograph', {}))
    roof = float(cfg['roof_top_z_m'])
    pose = p.get('pose', 'photo_raised')
    folded = pose == 'folded_visual'
    lower_deg = float(p.get('lower_angle_deg', 0.0 if folded else 12.0))
    upper_deg = float(p.get('upper_angle_deg', 0.0 if folded else 11.0))
    lower_len = LOWER_LINK_CANDIDATE_M
    upper_len = float(p.get('upper_visual_length_m', 1.08))
    a = Vector((-0.242231787197227, 0.0, roof + 0.172373228219755))
    e = a + Vector((lower_len * math.cos(math.radians(lower_deg)), 0,
                    lower_len * math.sin(math.radians(lower_deg))))
    u = e + Vector((0.015, 0, 0.053))
    h = u + Vector((-upper_len * math.cos(math.radians(upper_deg)), 0,
                    upper_len * math.sin(math.radians(upper_deg))))
    return {'roof_z_m': roof, 'pose': pose, 'lower_angle_deg': lower_deg,
            'upper_angle_deg': upper_deg, 'lower_link_m': lower_len,
            'upper_visual_length_m': upper_len,
            'base_joint': list(a), 'knee_lower_joint': list(e),
            'knee_upper_joint': list(u), 'head_joint': list(h),
            'carbon_top_z_m': h.z + 0.068,
            'pose_status': 'DISPLAY_POSE_ONLY_NOT_VALIDATED_KINEMATICS'}


def _mesh(name, verts, faces, mat, bevel=0):
    mesh = bpy.data.meshes.new(name + '_mesh')
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    o = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(o)
    finish(o, name, mat)
    if bevel:
        mod = o.modifiers.new('Small fabricated edge radius', 'BEVEL')
        mod.width = bevel
        mod.segments = 2
        o.modifiers.new('Weighted fabrication normals', 'WEIGHTED_NORMAL')
    return o


def _tube_beam(name, a, b, width_a, height_a, width_b, height_b, wall, mat):
    """A hollow rectangular metal section, with independently sized ends."""
    a, b = Vector(a), Vector(b)
    axis = (b - a).normalized()
    w = Vector((0, 1, 0))
    if abs(axis.dot(w)) > .98:
        w = Vector((1, 0, 0))
    w = (w - axis * axis.dot(w)).normalized()
    n = axis.cross(w).normalized()
    verts = []
    for c, width, height in ((a, width_a, height_a), (b, width_b, height_b)):
        for inset in (0, wall):
            for sw, sn in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                verts.append(tuple(c + w * (width / 2 - inset) * sw + n * (height / 2 - inset) * sn))
    faces = []
    for i in range(4):
        j = (i + 1) % 4
        faces.extend([(i, j, 8+j, 8+i), (4+j, 4+i, 12+i, 12+j),
                      (j, i, 4+i, 4+j), (8+i, 8+j, 12+j, 12+i)])
    return _mesh(name, verts, faces, mat, .0011)


def _plate_xz(name, polygon, y, thickness, mat, bevel=.001):
    verts = [(x, y - thickness/2, z) for x, z in polygon]
    verts += [(x, y + thickness/2, z) for x, z in polygon]
    n = len(polygon)
    faces = [tuple(range(n-1, -1, -1)), tuple(range(n, 2*n))]
    faces += [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)]
    return _mesh(name, verts, faces, mat, bevel)


def _lathe(name, a, b, profile, mat, segments=48):
    """Closed revolved profile about arbitrary axis; profile is [(t,r),...]."""
    a, b = Vector(a), Vector(b)
    closed_profile = profile[0] == profile[-1]
    if closed_profile:
        profile = profile[:-1]
    axis = (b-a).normalized()
    ref = Vector((0,1,0)) if abs(axis.y) < .95 else Vector((1,0,0))
    v = axis.cross(ref).normalized()
    w = axis.cross(v).normalized()
    verts = []
    for t, radius in profile:
        c = a.lerp(b, t)
        for j in range(segments):
            ang = math.tau * j / segments
            verts.append(tuple(c + radius * (v*math.cos(ang) + w*math.sin(ang))))
    faces = []
    for i in range(len(profile) if closed_profile else len(profile)-1):
        ni=(i+1)%len(profile)
        for j in range(segments):
            k=(j+1)%segments
            faces.append((i*segments+j, i*segments+k, ni*segments+k, ni*segments+j))
    if not closed_profile:
        faces.extend([tuple(range(segments-1,-1,-1)),
                      tuple((len(profile)-1)*segments+j for j in range(segments))])
    o = _mesh(name, verts, faces, mat)
    for poly in o.data.polygons:
        poly.use_smooth = len(poly.vertices)==4
    return o


def _washer(name, p, radius, bore, depth, mat, axis=(0,1,0)):
    p, n = Vector(p), Vector(axis).normalized()
    # The profile closes through the bore; end caps are not used for this ring.
    o = _lathe(name, p-n*depth/2, p+n*depth/2,
               [(0,bore),(0,radius),(1,radius),(1,bore),(0,bore)], mat, 32)
    return o


def _bezier(a, b, c, d, n=28):
    a,b,c,d=map(Vector,(a,b,c,d))
    out=[]
    for j in range(n):
        t=j/(n-1); s=1-t
        out.append(a*s*s*s+b*3*s*s*t+c*3*s*t*t+d*t*t*t)
    return out


def _braid(name, points, mat, pale_mat):
    """Flattened curved copper braid, not a rigid mechanism link."""
    points=[Vector(p) for p in points]
    output=[]
    verts=[]
    for p in points:
        verts += [tuple(p+Vector((0,-.0055,0))), tuple(p+Vector((0,.0055,0)))]
    faces=[(2*i,2*i+1,2*i+3,2*i+2) for i in range(len(points)-1)]
    output.append(_mesh(name+'_flat_braid',verts,faces,mat))
    # Counter-woven strands give actual braid geometry in close views.
    for family in (-1,1):
        for strand in range(3):
            path=[]
            for i,p in enumerate(points):
                ph=family*math.tau*5*i/(len(points)-1)+strand*math.tau/3
                path.append(tuple(p+Vector((.0008*math.cos(ph),.0048*math.sin(ph),.0011*math.cos(ph)))))
            o=tube_path(name+'_weave_%+d_%d'%(family,strand),path,.00055,pale_mat if strand==1 else mat)
            o.data.bevel_resolution=1
            output.append(o)
    return output


def _swept_strip(name, x, center_z, mat, width, thickness, half_span=.235, horn_end=.320):
    """Broad flat transverse strip with distinctly downturned curved horns."""
    ys=[-.320,-.312,-.297,-.280,-.261,-.244,-.235,0,.235,.244,.261,.280,.297,.312,.320]
    drops=[.063,.042,.025,.013,.006,.0015,0,0,0,.0015,.006,.013,.025,.042,.063]
    verts=[]
    for y,dz in zip(ys,drops):
        z=center_z-dz
        verts.extend([(x-width/2,y,z),(x+width/2,y,z),(x+width/2,y,z-thickness),(x-width/2,y,z-thickness)])
    faces=[]
    for i in range(len(ys)-1):
        for j in range(4):faces.append((4*i+j,4*i+(j+1)%4,4*(i+1)+(j+1)%4,4*(i+1)+j))
    faces.extend([(3,2,1,0),tuple(4*(len(ys)-1)+j for j in range(4))])
    return _mesh(name,verts,faces,mat,.0008)


def build(cfg):
    par=parameters(cfg)
    r=par['roof_z_m']; A=Vector(par['base_joint']); E=Vector(par['knee_lower_joint'])
    U=Vector(par['knee_upper_joint']); H=Vector(par['head_joint'])
    objects=[]
    steel=material('Panto charcoal fabricated steel',(.054,.075,.082,1),.74,.29)
    steel_edge=material('Panto bracket steel',(.083,.105,.114,1),.65,.33)
    silver=material('Panto bright steel links',(.48,.53,.55,1),.87,.24)
    dark=material('Panto black rubber',(.021,.025,.027,1),.02,.65)
    carbon=material('Panto graphite carbon strips',(.053,.059,.060,1),.12,.66)
    copper=material('Panto copper braid',(.42,.235,.104,1),.85,.32)
    copperlight=material('Panto copper braid highlights',(.62,.40,.22,1),.90,.33)
    brown=material('Panto brown ribbed insulators',(.22,.037,.027,1),.12,.24)
    red=material('Panto red folded cover',(.56,.018,.012,1),.46,.28)
    white_tube=material('Panto translucent air line',(.51,.63,.65,1),.03,.28)
    brass=material('Panto pneumatic fittings',(.38,.28,.10,1),.76,.33)

    def add(o, group='PANTOGRAPH_BASE', ref=SOURCE_PHOTO, note=None):
        tags(o,PHOTO,ref,group)
        o['component_family']='pantograph'
        o['evidence_level']=PHOTO
        o['physics_status']='GEOMETRY_ONLY_UNCALIBRATED'
        if note:o['interpretation_note']=note
        objects.append(o)
        return o

    # Three-point base: two outboard forward supports and one central aft
    # support; count/placement inferred from complementary photos, not 3D CAD.
    supports=[('aft',-.520,0),('near',-.150,-.225),('far',-.150,.225)]
    for key,x,y in supports:
        add(box('Panto_insulator_'+key+'_foot',(x,y,r+.006),(.092,.086,.012),steel_edge,.002))
        body_a=(x,y,r+.015); body_b=(x,y,r+.145)
        profile=[(0,.029),(.08,.029),(.095,.022)]
        for k in range(9):
            t=.12+k*.086
            profile.extend([(t,.022),(t+.018,.0355),(t+.035,.036),(t+.055,.022)])
        profile.extend([(.94,.023),(.95,.034),(1,.034)])
        add(_lathe('Panto_insulator_'+key+'_nine_sheds',body_a,body_b,profile,brown),note='Ribbed brown external insulator envelope; material specification and creepage rating unknown.')
        add(rod('Panto_insulator_'+key+'_top_stud',(x,y,r+.142),(x,y,r+.158),.009,silver,20))
        for ix in (-1,1):
            for iy in (-1,1):
                add(rod('Panto_insulator_'+key+'_foot_bolt_%s_%s'%(ix,iy),(x+ix*.033,y+iy*.029,r+.012),(x+ix*.033,y+iy*.029,r+.019),.005,silver,6))

    # Bent welded base frame. Open space remains around the bellows.
    for side in (-1,1):
        y=side*.137
        poly=[(-.545,r+.160),(-.335,r+.107),(-.047,r+.107),(-.047,r+.136),(-.326,r+.137),(-.536,r+.186)]
        add(_plate_xz('Panto_base_bent_side_rail_%+d'%side,poly,y,.017,steel_edge,.002))
        add(_tube_beam('Panto_base_splayed_support_%+d'%side,(-.24,y,r+.143),(-.15,side*.225,r+.151),.021,.026,.026,.023,.003,steel_edge))
        add(_plate_xz('Panto_base_pivot_cheek_%+d'%side,[(A.x-.056,r+.132),(A.x+.059,r+.132),(A.x+.039,r+.211),(A.x-.048,r+.216)],side*.110,.011,steel_edge,.002))
        add(rod('Panto_base_pivot_bearing_%+d'%side,(A.x,side*.100,A.z),(A.x,side*.132,A.z),.025,steel,32),'DTB-C0017',SOURCE_PHOTO+';'+SOURCE_CAD)
        add(rod('Panto_base_pivot_hex_%+d'%side,(A.x,side*.134,A.z),(A.x,side*.143,A.z),.010,silver,6),'DTB-C0017')
    add(box('Panto_base_aft_bridge',(-.516,0,r+.168),(.035,.287,.024),steel_edge,.002))
    add(box('Panto_base_forward_bridge',(-.049,0,r+.121),(.031,.300,.030),steel_edge,.002))
    add(rod('Panto_lower_arm_transverse_pivot',(A.x,-.125,A.z),(A.x,.125,A.z),.017,steel,32),'DTB-C0017')

    # Central box-section lower arm; CAD candidate applies only here.
    lower=add(_tube_beam('Panto_B29_lower_main_box_arm',A,E,.048,.045,.036,.036,.0035,steel),'DTB-C0017',SOURCE_PHOTO+';'+SOURCE_CAD,
          '0.819998716 m is transferred from a folded CAD 2D circle-pair candidate; physical assignment, depth and raised attitude remain inferred.')
    lower['candidate_center_distance_m']=LOWER_LINK_CANDIDATE_M
    lower['source_local_distance_mm']=1639.9974328476237
    lower['source_to_Model_uniform_scale']=.5
    lower['length_evidence']='CAD_2D_GEOMETRY_CANDIDATE'
    lower['center_A_world_m']=list(A);lower['center_B_world_m']=list(E)
    lower['display_angle_deg']=par['lower_angle_deg']
    # Knee has two separate axes and visible outer connecting cheeks.
    for side in (-1,1):
        y=side*.034
        poly=[(E.x-.029,E.z-.016),(E.x-.008,E.z-.079),(E.x+.040,E.z-.055),(U.x+.022,U.z+.022),(U.x-.018,U.z+.028),(E.x-.037,E.z+.008)]
        add(_plate_xz('Panto_knee_link_cheek_%+d'%side,poly,y,.011,steel_edge,.002),'DTB-C0017')
        for name,p in [('lower',E),('upper',U)]:
            add(_washer('Panto_knee_'+name+'_washer_%+d'%side,(p.x,side*.042,p.z),.014,.005,.004,silver),'DTB-C0017')
            add(rod('Panto_knee_'+name+'_hex_%+d'%side,(p.x,side*.043,p.z),(p.x,side*.052,p.z),.008,silver,6),'DTB-C0017')
    add(rod('Panto_knee_lower_shaft',(E.x,-.040,E.z),(E.x,.040,E.z),.017,steel,32),'DTB-C0017')
    add(rod('Panto_knee_upper_shaft',(U.x,-.043,U.z),(U.x,.043,U.z),.018,steel,32),'DTB-C0018')

    # The upper arm forks transversely near the collector. This is visible
    # in both the oblique original and the roof photographs.
    fork=U.lerp(H,.74)
    upper=add(_tube_beam('Panto_B30_upper_central_tapered_box',U,fork,.030,.036,.034,.033,.003,steel),'DTB-C0018')
    upper['visual_centerline_length_m']=par['upper_visual_length_m']
    upper['length_evidence']=PHOTO
    for side in (-1,1):
        end=H+Vector((0,side*.125,0))
        add(_tube_beam('Panto_B30_upper_fork_branch_%+d'%side,fork,end,.025,.030,.023,.026,.0025,steel),'DTB-C0018')
        # Triangular gusset close to fork root, welded outline distinct from rod.
        brace_a=U.lerp(H,.86)+Vector((0,side*.058,-.006))
        brace_b=H+Vector((.084,side*.125,-.019))
        add(_tube_beam('Panto_B30_fork_underbrace_%+d'%side,brace_a,brace_b,.012,.015,.014,.016,.002,steel),'DTB-C0018')
    add(rod('Panto_B30_head_cross_shaft',(H.x,-.145,H.z),(H.x,.145,H.z),.012,steel,32),'DTB-C0018')

    # Two separate slender adjustable links below the two main arm members.
    # They are static visual links; the eye placement is not solved closure.
    links=[('lower',Vector((-.046,-.119,r+.111)),E+Vector((.019,-.054,-.074))),
           ('upper',H+Vector((.056,-.126,-.061)),E+Vector((-.019,-.054,-.027)))]
    for name,p,q in links:
        group='DTB-C0017' if name=='lower' else 'DTB-C0018'
        delta=q-p
        add(rod('Panto_'+name+'_slender_adjustable_link',p,q,.0045,silver,20),group,note='One of the two visibly separate narrow rods; exact ball-centre constraint unknown.')
        for label,t in [('base',.025),('tip',.965)]:
            pp=p+delta*t;qq=p+delta*(t+.019 if t<.5 else t-.019)
            add(rod('Panto_'+name+'_link_locknut_'+label,pp,qq,.0072,silver,6),group)
        for label,pt in [('base',p),('tip',q)]:
            add(_washer('Panto_'+name+'_link_eye_'+label,pt,.011,.0045,.010,silver),group)
            add(rod('Panto_'+name+'_link_pin_'+label,pt+Vector((0,-.009,0)),pt+Vector((0,.009,0)),.0043,steel,16),group)

    # Collector head: paired graphite contact strips, downturned end horns,
    # cross-car carrier and two short vertical cylindrical guide assemblies.
    head='DTB-C0019'
    add(box('Panto_B31_head_carriage',(H.x,0,H.z+.003),(.071,.312,.020),steel_edge,.002),head)
    for side in (-1,1):
        y=side*.121
        add(_plate_xz('Panto_B31_head_hanger_%+d'%side,[(H.x-.033,H.z-.012),(H.x+.038,H.z-.012),(H.x+.027,H.z+.049),(H.x-.025,H.z+.049)],y,.009,steel_edge,.001),head)
        add(rod('Panto_B31_head_guide_outer_%+d'%side,(H.x,y,H.z+.010),(H.x,y,H.z+.047),.0105,silver,32),head,note='Small cylindrical head guide seen in photo 3; suspension law and internal spring specification unknown.')
        add(rod('Panto_B31_head_guide_pin_%+d'%side,(H.x,y,H.z-.026),(H.x,y,H.z+.059),.0046,silver,20),head)
        add(rod('Panto_B31_head_guide_collar_%+d'%side,(H.x,y,H.z+.019),(H.x,y,H.z+.025),.012,steel_edge,32),head)
        add(box('Panto_B31_head_top_yoke_%+d'%side,(H.x,y,H.z+.053),(.139,.025,.016),steel_edge,.002),head)
        add(_washer('Panto_B31_head_pivot_washer_%+d'%side,(H.x,side*.151,H.z),.012,.004,.004,silver),head)
    for k,dx in enumerate((-.056,.056),1):
        add(_swept_strip('Panto_B31_carbon_contact_strip_%d'%k,H.x+dx,H.z+.068,carbon,.031,.0085),head,note='Paired transverse carbon/contact envelopes and downturned horns inferred from photos; composition at horn ends is not resolved.')
        add(_swept_strip('Panto_B31_collector_metal_carrier_%d'%k,H.x+dx,H.z+.058,steel_edge,.024,.011),head)
        for side in (-1,1):
            add(rod('Panto_B31_strip_mount_bolt_%d_%+d'%(k,side),(H.x+dx,side*.119,H.z+.041),(H.x+dx,side*.119,H.z+.058),.0038,silver,6),head)

    # Three-convolution air-bellows-like envelope. A continuous surface of
    # revolution, oriented diagonally as seen in the side CAD/photo, not a
    # decorative cylinder. Do NOT assign B32's pneumatic force law to it.
    bell_a=Vector((-.494,0,r+.197));bell_b=Vector((-.336,0,r+.139))
    profile=[]
    for i in range(97):
        t=i/96
        rad=.054+.027*(math.sin(math.pi*3*t)**2)**.56
        profile.append((t,rad))
    bell=add(_lathe('Panto_base_three_convolution_bellows',bell_a,bell_b,profile,dark,64),
             'PANTOGRAPH_BELLOWS_UNRESOLVED',SOURCE_PHOTO+';crops/pantograph_bellows_crop.png;renders/05_pantograph_folded_side.png',
             'Observed multi-convolution flexible body, separate from the black barrel. Exact identity, internal construction, effective area and force path remain unknown.')
    bell['convolution_count_visual']=3
    bell['force_path']='UNKNOWN_DO_NOT_DRIVE_FROM_OLD_RETRACTED_CYLINDER'
    n=(bell_b-bell_a).normalized()
    for label,pt in [('aft',bell_a),('front',bell_b)]:
        add(rod('Panto_bellows_'+label+'_end_flange',pt-n*.005,pt+n*.005,.060,steel,48),'PANTOGRAPH_BELLOWS_UNRESOLVED')
        for j in range(4):
            an=j*math.tau/4
            pp=pt+Vector((0,.043*math.cos(an),.043*math.sin(an)))
            add(rod('Panto_bellows_'+label+'_flange_bolt_%d'%j,pp-n*.005,pp+n*.009,.004,silver,6),'PANTOGRAPH_BELLOWS_UNRESOLVED')
    # A lug visually meets each cap; no actuator-to-link law is implied.
    for side in (-1,1):
        add(_tube_beam('Panto_bellows_aft_mount_strap_%+d'%side,(-.535,side*.047,r+.177),(-.487,side*.047,r+.221),.012,.014,.012,.014,.002,steel_edge),'PANTOGRAPH_BELLOWS_UNRESOLVED')
    add(rod('Panto_bellows_air_elbow',(-.496,-.047,r+.174),(-.514,-.053,r+.171),.007,brass,16),'PANTOGRAPH_BELLOWS_UNRESOLVED')
    add(tube_path('Panto_bellows_visible_air_hose',[(-.512,-.053,r+.171),(-.547,-.068,r+.153),(-.564,-.077,r+.112),(-.555,-.078,r+.057),(-.546,-.074,r+.012)],.0029,white_tube),'PANTOGRAPH_BELLOWS_UNRESOLVED')

    # Distinct compact black barrel under the lower pivot, clearly present
    # in photos 2 and the mechanism crop. Tentative B32 correspondence only.
    ca=Vector((-.239,-.054,r+.127)); cb=Vector((-.061,-.054,r+.145))
    direction=(cb-ca).normalized()
    barrel=add(rod('Panto_B32_separate_black_barrel',ca,ca.lerp(cb,.74),.0175,steel,40),'DTB-C0020',SOURCE_PHOTO+';crops/pantograph_mechanisms_crop.png',
       'B32 lists a cylinder; assignment to this independent visible barrel is tentative. Bellows and barrel remain distinct; no hidden piston/air circuit invented.')
    barrel['registration_certainty']='TENTATIVE_VISUAL_CORRESPONDENCE'
    add(rod('Panto_B32_barrel_end_collar',ca.lerp(cb,.66),ca.lerp(cb,.76),.020,steel_edge,40),'DTB-C0020')
    add(rod('Panto_B32_exposed_small_shaft',ca.lerp(cb,.73),cb,.0065,silver,24),'DTB-C0020')
    for label,pt in [('aft',ca),('front',cb)]:
        add(_washer('Panto_B32_mount_eye_'+label,pt,.011,.004,.016,steel_edge),'DTB-C0020')
        add(rod('Panto_B32_mount_bolt_'+label,pt+Vector((0,-.014,0)),pt+Vector((0,.014,0)),.004,silver,6),'DTB-C0020')

    # Actual soft copper bridge loops at the primary hinge locations.
    for side in (-1,1):
        p0=A+Vector((-.061,side*.127,.015));p3=A+Vector((.060,side*.042,.020))
        points=_bezier(p0,p0+Vector((-.018,0,-.052)),p3+Vector((-.035,0,-.052)),p3)
        for o in _braid('Panto_copper_base_bridge_%+d'%side,points,copper,copperlight):
            add(o,'PANTOGRAPH_ELECTRICAL_BRAID',note='Flexible braided electrical bridge; no rigid mechanical constraint.')
        p0=E+Vector((-.051,side*.048,.008));p3=U+Vector((-.045,side*.041,.001))
        points=_bezier(p0,E+Vector((.075,side*.087,-.098)),U+Vector((.075,side*.072,-.058)),p3)
        for o in _braid('Panto_copper_knee_bridge_%+d'%side,points,copper,copperlight):
            add(o,'PANTOGRAPH_ELECTRICAL_BRAID',note='Visible flexible bridge around the knee; soft lead not a linkage member.')
        p0=H+Vector((.075,side*.098,-.002));p3=H+Vector((-.025,side*.140,.021))
        for o in _braid('Panto_copper_head_bridge_%+d'%side,_bezier(p0,p0+Vector((0,0,-.055)),p3+Vector((0,0,-.058)),p3),copper,copperlight):
            add(o,'PANTOGRAPH_ELECTRICAL_BRAID')

    # Red removable folded-sheet hood is offset beside the mechanism.
    # Only visible external panels are rebuilt; there is no invented internals.
    cx=.095;cy=.190; L=.238;W=.172;h=.154
    x0=cx-L/2;x1=cx+L/2;y0=cy-W/2;y1=cy+W/2
    for label,y in [('near',y0),('far',y1)]:
        poly=[(x0,r+.004),(x1,r+.004),(x1,r+.131),(x1-.031,r+h),(x0+.012,r+h)]
        add(_plate_xz('Panto_red_cover_'+label+'_sheet',poly,y,.0035,red,.001),'PANTOGRAPH_RED_COVER',note='External red cover from photos; enclosed contents and function unknown.')
    add(box('Panto_red_cover_top',(cx-.009,cy,r+h-.001),(.200,W,.0035),red,.001),'PANTOGRAPH_RED_COVER')
    add(box('Panto_red_cover_back',(x0+.002,cy,r+.076),(.0035,W,.147),red,.001),'PANTOGRAPH_RED_COVER')
    add(box('Panto_red_cover_open_face_shadow',(x1-.006,cy,r+.058),(.004,W-.019,.097),dark,.001),'PANTOGRAPH_RED_COVER')
    add(box('Panto_red_cover_front_lower_lip',(x1,cy,r+.018),(.004,W,.026),red,.001),'PANTOGRAPH_RED_COVER')
    for x in (x0+.015,x1-.025):
        for z in (r+.018,r+.129):
            add(rod('Panto_red_cover_visible_fastener_%03d_%03d'%(round(x*1000),round(z*1000)),(x,y0-.003,z),(x,y0-.006,z),.0032,silver,12),'PANTOGRAPH_RED_COVER')
    add(rod('Panto_red_cover_cable_grommet',(x1+.0005,cy+.038,r+.047),(x1+.004,cy+.038,r+.047),.007,dark,24),'PANTOGRAPH_RED_COVER')

    # Every part carries a conservative label. Candidate length is recorded
    # separately so it cannot silently become a physical calibration claim.
    for o in objects:
        o['pose_status']=par['pose_status']
        o['pose_name']=par['pose']
        o['mass_inertia_status']='NOT_ASSIGNED'
    return objects
