"""Plot final integrated numerical evidence, never used as an input trajectory."""
from pathlib import Path
import os,sys,json
os.environ.setdefault('MPLCONFIGDIR','/tmp/b8-mpl')
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT.parent/'vendor'))
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=ROOT/'results'
def a(n):return np.genfromtxt(O/(n+'.csv'),names=True,delimiter=',')
def m(n):return json.loads((O/(n+'_metrics.json')).read_text())
fig,ax=plt.subplots(2,3,figsize=(15,8),layout='constrained');plt.rcParams['font.size']=10
names=['low_adhesion_coarse_dt','low_adhesion','low_adhesion_half_dt'];v=[m(n) for n in names]
ax[0,0].plot([x['dt_s']*1e6 for x in v],[x['peak_flange_single_contact_N'] for x in v],'o-');ax[0,0].set(title='77-coordinate low-adhesion convergence',xlabel='Time step [microseconds]',ylabel='Unfiltered single-contact flange peak [N]',ylim=(0,20));ax[0,0].text(30,3,'15.7516 / 15.6162 / 15.5309 N\nAdjacent changes: 0.86% / 0.55%\nOriginal threshold: 20%',fontsize=9)
for n in ['service_brake','half_dt','flange_disturbance','flange_half_dt']:
 d=a(n);ax[0,1].plot(d['time_s'],d['mechanical_balance_residual_J'],label=n)
ax[0,1].axhline(.15,color='red',ls=':',label='unchanged acceptance 0.15 J');ax[0,1].set(title='Mechanical + contact storage closure',xlabel='Time [s]',ylabel='Residual [J]');ax[0,1].legend(fontsize=8)
d=a('doors_open_close')
for k in d.dtype.names:
 if k.startswith('door_') and k.endswith('_angle_rad'):ax[0,2].plot(d['time_s'],np.rad2deg(d[k]),label=k[5:-10])
ax[0,2].set(title='Six torque-driven doors / passive closure',xlabel='Time [s]',ylabel='Hinge angle [degrees]');ax[0,2].legend(fontsize=8)
for n in ['flange_disturbance','flange_half_dt']:
 d=a(n);ax[1,0].plot(d['time_s'],d['flange_normal_N'],label=n)
ax[1,0].set(title='Axle-side load; sampled flange sum',xlabel='Time [s]',ylabel='100 Hz sampled force [N]',xlim=(.9,1.3));ax[1,0].legend(fontsize=8)
for n in ['service_brake','half_dt','doors_open_close','doors_half_dt','flange_disturbance','flange_half_dt']:
 d=a(n);ax[1,1].plot(d['time_s'],d['momentum_balance_residual_Ns'],label=n)
ax[1,1].set(title='External impulse - train COM momentum',xlabel='Time [s]',ylabel='Vector residual norm [N s]');ax[1,1].legend(fontsize=8)
for n in ['service_brake','low_adhesion','low_adhesion_half_dt']:
 d=a(n);ax[1,2].plot(d['time_s'],d['contact_friction_work_J'],label=n)
ax[1,2].set(title='Actual wheel slip dissipates energy',xlabel='Time [s]',ylabel='Wheel/rail friction work [J]');ax[1,2].legend(fontsize=8)
for x in ax.flat:x.grid(alpha=.25)
fig.suptitle('B8 FINAL | 22 integrated cases, 273 checks | uncalibrated equivalent contact law',fontsize=15)
fig.savefig(O/'contact_and_doors_acceptance.png',dpi=150)
