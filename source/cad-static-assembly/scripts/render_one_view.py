"""Render one saved scene in one process; remove irrelevant geometry first."""
import bpy,os
from pathlib import Path
from mathutils import Vector
R=Path(__file__).resolve().parents[1];O=R/'outputs'/os.environ.get('GTSD_OUTPUT_PASS','pass3')
view=os.environ.get('GTSD_VIEW','bogie')
bpy.ops.wm.read_factory_settings(use_empty=True)
scene_name='09_EXPLODED_DISPLAY_OFFSETS_NOT_SOURCE_POSE' if view=='exploded'else '01_CAD_STATIC_ASSEMBLY_WORKING'
with bpy.data.libraries.load(str(O/'GTSD_CAD_full_review_working.blend'),link=False)as(src,loaded):
 assert scene_name in src.scenes;loaded.scenes=[scene_name]
scene=loaded.scenes[0];bpy.context.window.scene=scene
for s in list(bpy.data.scenes):
 if s!=scene:bpy.data.scenes.remove(s)
allowed={'bogie':{'bogie'},'panto':{'pantograph'},'interior':{'body','bogie'},'bench':{'body','bogie','pantograph','environment'},'exploded':{'body','bogie','pantograph'}}[view]
for ob in list(bpy.data.objects):
 if ob.type=='MESH'and (ob.get('group')not in allowed or (view=='interior'and ob.get('source_name','').startswith('CAD_body_skin_'))):
  bpy.data.objects.remove(ob,do_unlink=True)
for c in scene.collection.children:
 c.hide_render=False
bpy.data.orphans_purge(do_local_ids=True,do_linked_ids=True,do_recursive=True)
cam={'bogie':'CAM_bogie_source_stack','panto':'CAM_pantograph_multiview','interior':'CAM_interior_source_review','bench':'CAM_complete_bench','exploded':'CAM_exploded'}[view]
scene.camera=next(o for o in scene.objects if o.type=='CAMERA'and o.name==cam)
if view in ('panto', 'exploded'):
 bpy.context.view_layer.update()
 points=[ob.matrix_world@Vector(p) for ob in scene.objects if ob.type=='MESH' for p in ob.bound_box]
 lo=Vector(tuple(min(p[i]for p in points)for i in range(3)))
 hi=Vector(tuple(max(p[i]for p in points)for i in range(3)))
 center=(lo+hi)/2
 camera=scene.camera;camera.location=center+Vector((-1.8,-2.6,1.5)).normalized()*4
 camera.rotation_euler=(center-camera.location).to_track_quat('-Z','Y').to_euler()
 bpy.context.view_layer.update()
 projected=[camera.matrix_world.inverted()@p for p in points]
 width=max(p.x for p in projected)-min(p.x for p in projected)
 height=max(p.y for p in projected)-min(p.y for p in projected)
 aspect=scene.render.resolution_x/scene.render.resolution_y
 # A world AABB center is not necessarily the center of the projected union.
 # Center the actual projected mesh boxes before fitting the frame.
 shift=Vector(((min(p.x for p in projected)+max(p.x for p in projected))/2,
               (min(p.y for p in projected)+max(p.y for p in projected))/2,0))
 camera.location += camera.matrix_world.to_3x3()@shift
 camera.data.ortho_scale=max(width,height*aspect)*1.2
 bpy.context.view_layer.update()
 from bpy_extras.object_utils import world_to_camera_view
 for attempt in range(4):
  ndc=[world_to_camera_view(scene,camera,p)for p in points]
  radius=max(max(abs(p.x-.5),abs(p.y-.5))for p in ndc)
  if radius<=.45:break
  camera.data.ortho_scale*=radius/.44
  bpy.context.view_layer.update()
 ndc=[world_to_camera_view(scene,camera,p)for p in points]
 assert all(.049<p.x<.951 and .049<p.y<.951 for p in ndc), ('camera framing',view)
 print('CAMERA_BOUNDS_VERIFIED',view,
       [min(p.x for p in ndc),min(p.y for p in ndc),max(p.x for p in ndc),max(p.y for p in ndc)],flush=True)
scene.render.threads_mode='FIXED';scene.render.threads=1
scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=12;scene.cycles.use_denoising=True
file={'bogie':'02_bogie_CAD_stack_working.png','bench':'03_complete_bench_CAD_additions_working.png','panto':'04_pantograph_multiview_working.png','interior':'05_interior_cutaway_working.png','exploded':'06_exploded_display_working.png'}[view]
scene.render.filepath=str(O/file)
print('ONE_VIEW',view,'meshes',sum(o.type=='MESH'for o in scene.objects),flush=True)
bpy.ops.render.render(write_still=True)
print('ONE_VIEW_DONE',view,flush=True)
