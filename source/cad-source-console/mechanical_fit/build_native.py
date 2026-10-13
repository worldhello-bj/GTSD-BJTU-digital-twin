"""Reproducible Blender assembly with source and opt-in engineering fit scenes.

Run export_scene.js first. No physics, imports from external files, or images.
Unchanged mesh data is shared between scenes; source geometry is retained.
"""
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import struct
import sys
import bpy

parser = argparse.ArgumentParser()
parser.add_argument('--input', type=Path, required=True)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index('--')+1:])
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.context.preferences.filepaths.save_version = 0
default_scene = bpy.context.scene
materials, objects = {}, {}


def witness(v, i):
    return hashlib.sha256(struct.pack('<'+'f'*len(v), *[0. if x == 0 else x for x in v]) +
                          struct.pack('<'+'I'*len(i), *i)).hexdigest()


for item in json.loads((args.input/'scenes.json').read_text())['scenes']:
    data = json.load(gzip.open(args.input/item['file'], 'rt'))
    scene = default_scene if item['profile'] == 'source' else bpy.data.scenes.new('FIT_'+item['profile'])
    scene.name = 'SOURCE_REVIEW' if item['profile'] == 'source' else 'FIT_'+item['profile']
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 1
    scene['as_built'] = False
    scene['new_physics'] = False
    scene['engineering_fit_profile'] = item['profile']
    scene['source_display_revision_sha256'] = data['source_display_revision_sha256']
    for m in data['meshes']:
        key = (m['name'], m['geometry_witness_sha256'])
        if key in objects:
            continue
        assert witness(m['vertices'], m['indices']) == m['geometry_witness_sha256']
        me = bpy.data.meshes.new(m['name'])
        me.from_pydata(list(zip(m['vertices'][::3], m['vertices'][1::3], m['vertices'][2::3])), [],
                       list(zip(m['indices'][::3], m['indices'][1::3], m['indices'][2::3])))
        me.update()
        ob = bpy.data.objects.new(m['name'], me)
        rgba = tuple(m['color'])
        if rgba not in materials:
            material = bpy.data.materials.new('source_display_material_'+str(len(materials)))
            material.diffuse_color = rgba
            materials[rgba] = material
        me.materials.append(materials[rgba])
        ob['source_name'] = m['name']
        ob['geometry_witness_sha256'] = m['geometry_witness_sha256']
        ob['source_metadata_json'] = json.dumps(m['metadata'], ensure_ascii=False)
        ob['new_physics'] = False
        ob['as_built'] = False
        ob['mechanical_fit_candidate'] = m['mechanical_fit_candidate']
        ob['group'] = m['group']
        objects[key] = ob
    refs = data.get('mesh_refs') or [{'name':m['name'],'geometry_witness_sha256':m['geometry_witness_sha256']} for m in data['meshes']]
    names = [row['name'] for row in refs]
    assert len(names) == item['mesh_count'] and len(names) == len(set(names))
    for row in refs:
        scene.collection.objects.link(objects[(row['name'], row['geometry_witness_sha256'])])
    scene['mesh_count'] = len(names)
if 'FIT_combined' in bpy.data.scenes:
    bpy.context.window.scene = bpy.data.scenes['FIT_combined']
args.output.parent.mkdir(parents=True, exist_ok=True)
bpy.ops.wm.save_as_mainfile(filepath=str(args.output), compress=True)
print(json.dumps({'ok': True, 'scenes': len(bpy.data.scenes), 'unique_meshes': len(objects),
                  'images': len(bpy.data.images), 'new_physics': False,
                  'file': args.output.name}))
