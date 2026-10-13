import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
HERE=Path(__file__).resolve().parent

def render(out=HERE/'build'):
 graph=json.loads((HERE/'native_edge_graph.json').read_text());p=json.loads((out/'plan_geometry.json').read_text());fig,axs=plt.subplots(2,1,figsize=(15,9),gridspec_kw={'height_ratios':[3,1]});ax=axs[0]
 for row in graph['source_curves']:
  q=np.asarray(row['points']);ax.plot(q[:,0],q[:,1],color='#7f8c95',lw=.7,alpha=.55)
 for name,g in p['plans'].items():
  coords=[g['coordinates']]if g['type']=='Polygon'else g['coordinates'];color='#58768a'if'WING'in name else('#c09852'if'FROG'in name else '#778a79')
  for rings in coords:
   ax.add_patch(Polygon(rings[0],closed=True,facecolor=color,alpha=.4,edgecolor=color,lw=1))
 for q,handles in [(p['lower_chain'],['4ED5C','4ECEE','4ECFA']),(p['upper_chain'],['4ED5D','4ED02','4ECEC'])]:
  q=np.asarray(q);ax.plot(q[:,0],q[:,1],color='#176db2',lw=2)
  for a,b,h in zip(q,q[1:],handles):
   c=(a+b)/2;ax.text(*c,h,fontsize=8,va='bottom')
 x,y=p['facts']['nose_world_mm'];ax.annotate(f'Finite CAD nose ({x:.3f}, {y:.3f}) mm\nPhoto association only',xy=(x,y),xytext=(x-130,y+165),arrowprops={'arrowstyle':'->'},fontsize=10)
 ax.annotate('15.333 mm source-associated separation\nNot a photo measurement',xy=(-9000,231.5),xytext=(-9290,115),arrowprops={'arrowstyle':'->'},fontsize=9)
 ax.text(-9440,438,'39.5 mm paired native head bands; separate finite wings; no straight full-rail crossing',fontsize=10)
 ax.set(xlim=(-9500,-8220),ylim=(80,470),xlabel='Source world X / mm',ylabel='Source world Y / mm',title='Photo-supported turnout candidate: plan / finite source overlay');ax.set_aspect('equal');ax.grid(alpha=.2)
 ax=axs[1]
 import gzip
 import cap_geometry as C
 import geos_kernel as K
 payload=json.load(gzip.open(out/'candidate_meshes.json.gz','rt'))
 for m in payload['meshes']:
  if m['metadata']['role']=='unchanged_source_remainder':continue
  psec=C.actual_section(m,0,-9.0,np.zeros(3))
  color='#c09852'if'FROG'in m['name']else('#58768a'if'WING'in m['name']else('#778a79'if'long_plate'in m['name']else'#bb904b'))
  for rings in K.polys(psec):
   if not rings or len(rings[0])<3:continue
   ax.add_patch(Polygon(rings[0],closed=True,color=color,alpha=.7))
  K.G.GEOSGeom_destroy(psec)
 ax.set(xlim=(130,390),ylim=(-80,8),ylabel='Source Z / mm',xlabel='Source world Y / mm',title='Actual triangle section at X = -9000 mm: inherited I-section, explicit 65 mm slot-depth choice')
 ax.set_aspect('equal');ax.grid(alpha=.2)
 fig.suptitle('Engineering alternative; as-built unconfirmed; no solver / wheel-passage certification',fontsize=13);fig.tight_layout();fig.savefig(out/'source_dimension_overlay.png',dpi=140);plt.close(fig)
if __name__=='__main__':render()
