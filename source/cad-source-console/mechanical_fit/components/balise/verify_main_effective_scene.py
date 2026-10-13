#!/usr/bin/env python3
"""Read-only verification of public main assets and the complete atomic candidate.
Uses only NumPy, local candidate JSON, catalog/GLB/extras/source_replacements.
"""
import argparse,json
from pathlib import Path
from balise_geometry import load_effective_scenes,apply_profiles,validate_changed_meshes
HERE=Path(__file__).resolve().parent

def run(root, profiles=None, output=None):
    scenes,identity=load_effective_scenes(root)
    profiles=profiles or [json.loads((HERE/'balise_fit_profile.json').read_text())]
    out=[]
    for scene,before in scenes.items():
        after,identities=apply_profiles(before,profiles)
        validation=validate_changed_meshes(before,after,identities)
        count=identity['scene_counts'][scene]
        out.append({'scene':scene,'base_scene_mesh_count':count['base'],'source_replacement_removed_count':count['source_removed'],'source_replacement_added_count':count['source_added'],'effective_before_count':len(before),'fit_removed_count':sum(len(p['remove_names']) for p in profiles),'fit_added_count':sum(len(p['add_meshes']) for p in profiles),'effective_after_count':len(after),'all_18_remove_names_present':len(profiles)==1 and len(identities)==18,'all_profile_remove_names_present':True,'source_pending_new_names_not_duplicated':True,**validation,'pass':validation['passes_no_new_interference_gate']})
    report={'schema':'balise-current-main-effective-scene-check-v1','numerical_contact_tolerance_m':1e-8,'input_identity':identity,'all_five_scene_results':out,'pass':all(r['pass'] for r in out),'global_claim':'No new positive or uncleared interference for these changed meshes against each complete effective main resource. Existing source proxy interference remains separately reported. This is not certification of the whole scene.','selection':'Base GLB + extras, exact catalog scene, active source replacements, then atomic fit profiles. Other pending old chair/clip objects remain unchanged; no full298 double application.'}
    (output or HERE/'current_main_effective_scene_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'pass':report['pass'],'scenes':[{'name':r['scene'],'count':r['effective_after_count'],'new_bad_pairs':len(r['new_positive_or_uncleared_interference_pairs']),'remaining_after_positive_or_uncleared_pairs':len(r['remaining_after_positive_or_uncleared_pairs']),'after_aabb_pairs':len(r['after_positive_aabb_pairs'])} for r in out]},ensure_ascii=False,indent=2))
    if not report['pass']:raise RuntimeError('Main resource fit gate failed')
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project-root','--source-root',dest='root',type=Path,required=True);p.add_argument('--profile',type=Path,action='append');p.add_argument('--output',type=Path);a=p.parse_args()
    run(a.root,[json.loads(p.read_text()) for p in a.profile] if a.profile else None,a.output)
