"""Independent signed tetrahedron integration; no inferred material density."""
import numpy as np

def pa(m,c):
 c=np.asarray(c,dtype=float);return m*((c@c)*np.eye(3)-np.outer(c,c))

def integrate(mesh):
 v=np.asarray(mesh['vertices'],dtype=float).reshape(-1,3);tri=np.asarray(mesh['indices']).reshape(-1,3)
 edges={}
 for row in tri:
  for i,j in zip(row,np.roll(row,-1)):
   key=tuple(sorted((int(i),int(j))));edges[key]=edges.get(key,0)+1
 # Duplicate vertex indices can appear after exporters; supplier solid flag and
 # volume checks own their tolerance-weld validation. Positive geometry required.
 a,b,c=v[tri].transpose(1,0,2);s=a+b+c;dv=np.einsum('ij,ij->i',a,np.cross(b,c))/6
 V=float(dv.sum());F=np.sum(dv[:,None]*s/4,axis=0);Q=np.einsum('i,ij,ik->jk',dv,s,s)/20
 for x in (a,b,c):Q+=np.einsum('i,ij,ik->jk',dv,x,x)/20
 if not V>0 or not np.isfinite(np.r_[V,F,Q.reshape(-1)]).all():raise ValueError('Nonpositive or nonfinite source solid: '+mesh['name'])
 center=F/V;inertia=np.trace(Q)*np.eye(3)-Q-pa(V,center)
 return dict(name=mesh['name'],volume_m3=V,com_component_m=center.tolist(),inertia_unit_density_com_kgm2=inertia.tolist())

def physical(tensor,label):
 I=np.asarray(tensor,dtype=float)
 if I.shape!=(3,3) or np.max(abs(I-I.T))>1e-12:raise ValueError('Asymmetric inertia '+label)
 eig=np.linalg.eigvalsh(I)
 if not np.isfinite(I).all() or eig[0]<=0 or eig[-1]>sum(eig[:2])+1e-11:raise ValueError('Nonphysical inertia '+label)
 return eig

def combine(rows,mass,translation,origin):
 V=sum(r['volume_m3'] for r in rows);F=np.zeros(3);Io=np.zeros((3,3));parts=[]
 for r in rows:
  m=mass*r['volume_m3']/V;c=np.asarray(r['com_component_m'])+translation-origin;I=np.asarray(r['inertia_unit_density_com_kgm2'])*mass/V
  physical(I,r['name']);F+=m*c;Io+=I+pa(m,c);parts.append(dict(name=r['name'],mass_kg=m,com_local_m=c.tolist(),inertia_com_local_kgm2=I.tolist()))
 c=F/mass;I=Io-pa(mass,c);I=(I+I.T)/2;eig=physical(I,'combined residual')
 return dict(mass_kg=mass,source_total_volume_m3=V,com_local_m=c.tolist(),inertia_com_local_kgm2=I.tolist(),principal_inertias_kgm2=eig.tolist(),parts=parts,weighting_is_material_density=False)
