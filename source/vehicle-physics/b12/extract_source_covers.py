"""Read four removable cover meshes, with no inferred hinge or source mutation."""
import hashlib
import json
from pathlib import Path
import bpy
from mathutils import Vector

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
SOURCE=REPO/'local-replay-base/GTSD_BJTU_B8/source/GTSD_BJTU_B7_detailed.blend'
NAMES=('B1_InverterParallelFins','B5_InverterLid','B1_ControllerHeatsink','B5_DriveControllerLid')


def main():
    digest=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    bpy.ops.wm.open_mainfile(filepath=str(SOURCE),load_ui=False,use_scripts=False)
    scene=bpy.data.scenes['B7_MAIN_ASSET_KINEMATIC']; bpy.context.window.scene=scene; scene.frame_set(1)
    # Hidden collections are excluded from evaluated depsgraphs. Enable this
    # collection only in memory before querying world geometry; never save.
    collection=bpy.data.collections['27_B5_COMPONENT_COVERS']
    hidden=dict(hide_viewport=collection.hide_viewport,hide_render=collection.hide_render)
    collection.hide_viewport=False; collection.hide_render=False
    def enable(layer):
        if layer.name=='27_B5_COMPONENT_COVERS':
            layer.exclude=False; layer.hide_viewport=False
        for child in layer.children:
            enable(child)
    enable(scene.view_layers[0].layer_collection)
    for name in NAMES:
        bpy.data.objects[name].hide_set(False)
        bpy.data.objects[name].hide_viewport=False
        bpy.data.objects[name].update_tag()
    bpy.data.objects['GTSD_Rig'].update_tag()
    scene.frame_set(2); scene.frame_set(1)
    scene.view_layers[0].update(); dg=bpy.context.evaluated_depsgraph_get()
    records=[]
    for name in NAMES:
        obj=bpy.data.objects[name]; ev=obj.evaluated_get(dg)
        mesh=ev.to_mesh(); vertices=[ev.matrix_world@v.co for v in mesh.vertices]
        lo=Vector([min(v[i] for v in vertices) for i in range(3)]); hi=Vector([max(v[i] for v in vertices) for i in range(3)])
        center=(lo+hi)/2; dims=hi-lo
        owner=obj
        attachment=[]
        while owner:
            attachment.append(dict(name=owner.name,parent_type=owner.parent_type,parent_bone=owner.parent_bone))
            owner=owner.parent
        records.append(dict(name=name,world_matrix=[list(row) for row in ev.matrix_world],
                            object_location=list(obj.location),matrix_basis=[list(row) for row in obj.matrix_basis],
                            matrix_parent_inverse=[list(row) for row in obj.matrix_parent_inverse],
                            aabb_center_world_m=list(center),aabb_dimensions_m=list(dims),
                            evaluated_vertex_count=len(vertices),evaluated_polygon_count=len(mesh.polygons),
                            collection_names=[c.name for c in obj.users_collection],source_hide_render=obj.hide_render,
                            attachment_chain=attachment,drivers=len(obj.animation_data.drivers) if obj.animation_data else 0,
                            source_properties={k:str(obj[k]) for k in obj.keys()},
                            center_local_m=list(ev.matrix_world.inverted()@center)))
        ev.to_mesh_clear()
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest()!=digest:
        raise RuntimeError('source modified')
    record=dict(source_sha256=digest,reference_frame=1,covers=records,
                original_cover_collection_hidden=hidden,
                scope='source removable cover geometry; hidden visibility-only members; no hinge, fastener mechanism, mass or material evidence')
    (ROOT/'source_covers.json').write_text(json.dumps(record,ensure_ascii=False,indent=2),encoding='utf-8',newline='\n')
    print(json.dumps(record,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':
    main()
