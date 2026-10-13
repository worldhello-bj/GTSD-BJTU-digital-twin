#!/usr/bin/env python3
"""Small generated mesh-plan preview. No original photograph is embedded."""
from pathlib import Path
import argparse,json,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from balise_geometry import load_candidate_geometry
HERE=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--project-root',type=Path,required=True);args=parser.parse_args()
g,_,_=load_candidate_geometry(args.project_root,json.loads((HERE/'balise_source_seed.json').read_text()))
p=json.loads((HERE/'balise_fit_profile.json').read_text());h=dict(g)
for m in p['add_meshes']:h[m['metadata']['replaces_mesh_name']]=(np.array(m['vertices']).reshape(-1,3),np.array(m['indices']).reshape(-1,3))
fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for row,(idx,st,xc) in enumerate([(2,12,-7.15),(3,17,-6.17)]):
    for col,(state,title) in enumerate([(g,'Before: full source chair position + inherited balise'),(h,'Candidate: rigid 90-degree yaw, source bay + rail corridor')]):
        a=ax[row,col]
        select=[n for n in state if (n.startswith('DETAIL_BENCH_long_sleeper_') or n.startswith('DETAIL_BENCH_branch_chair_') or n.startswith('DETAIL_BENCH_branch_fastener_') or n in ['ENV_main_rail_1','DETAIL_BENCH_turnout_inner_right_segment',f'ENV_balise_body_{idx:02}',f'ENV_balise_yellow_lid_{idx:02}'])]
        select.sort(key=lambda n:2 if 'balise' in n else(0 if 'sleeper' in n else 1))
        for n in select:
            v,f=state[n]
            if v[:,0].max()<xc-.25 or v[:,0].min()>xc+.25:continue
            t=v[f];mask=(t[:,:,0].max(1)>xc-.25)&(t[:,:,0].min(1)<xc+.25)
            color='#e2ae24' if 'balise' in n else ('#c6cbd0' if 'sleeper' in n else '#515a66')
            alpha=.9 if 'balise_body' in n else (.2 if 'yellow_lid' in n else .42)
            a.add_collection(PolyCollection(t[mask,:,:2],facecolors=color,edgecolors='none',alpha=alpha))
        a.set_xlim(xc-.25,xc+.25);a.set_ylim(-.16,.34);a.set_aspect('equal');a.grid(alpha=.18);a.set_xlabel('World X (m)');a.set_ylabel('World Y (m)');a.set_title(f'Balise {idx:02}: '+title,fontsize=9)
        if col:
            info=p['groups'][row];a.text(.02,.98,f"Body-to-sleeper gap: {info['body_each_side_geometric_gap_mm']:.3f} mm each\nBody and lid sizes unchanged; Z unchanged",transform=a.transAxes,ha='left',va='top',fontsize=8,bbox={'facecolor':'white','alpha':.8,'edgecolor':'none'})
fig.suptitle('Balise installation engineering candidate | source photographs inspected; exact installation unconfirmed',fontsize=12)
fig.savefig(HERE/'balise_fit_before_after.png',dpi=150);plt.close(fig)
