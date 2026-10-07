"""Standalone technical plots from recorded solver and gas states, never motion inputs."""
from pathlib import Path
import os,sys,json
os.environ.setdefault('MPLCONFIGDIR','/tmp/b8-mpl')
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT.parent/'vendor'))
import numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT=ROOT/'results'

def load(n):return np.genfromtxt(OUT/f'{n}.csv',delimiter=',',names=True)
def main():
 plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.22})
 fig,ax=plt.subplots(3,3,figsize=(16,12),layout='constrained');A=load('service_brake');t=A['time_s']
 for case,lab in [('service_brake','Pressure + friction'),('coast','Coast'),('brake_mu_zero','Zero pad friction'),('no_supply_pressure','Empty service reservoir'),('loaded_brake','+60 kg payload')]:
  d=load(case);ax[0,0].plot(d['time_s'],d['speed_mps'],label=lab,lw=1.5)
 ax[0,0].set(title='Force and brake causality',ylabel='Car A speed [m/s]');ax[0,0].legend(fontsize=8)
 for key,lab in [('reservoir_pressure_Pa','Reservoir'),('brake_pressure_Pa','One brake chamber'),('pantograph_pressure_Pa','Pantograph chamber')]:ax[0,1].plot(t,(A[key]-101325)/1000,label=lab)
 ax[0,1].set(title='Finite-volume gas states',ylabel='Gauge pressure [kPa]');ax[0,1].legend(fontsize=8)
 for case,lab in [('service_brake','Quick exhaust'),('normal_exhaust','Normal exhaust'),('collector_disturbance','Downward head load')]:
  d=load(case);ax[0,2].plot(d['time_s'],d['collector_height_m'],label=lab)
 ax[0,2].set(title='Solved Z-arm head motion',ylabel='Top pivot height [m]');ax[0,2].legend(fontsize=8)
 for case,lab in [('service_brake','Nominal'),('collector_disturbance','Contact disturbance'),('loaded_brake','Added mass')]:
  d=load(case);ax[1,0].plot(d['time_s'],d['collector_normal_N'],label=lab)
 ax[1,0].set(title='Unilateral overhead contact',ylabel='Normal force, sampled sum [N]');ax[1,0].legend(fontsize=8)
 for case,lab in [('lateral_disturbance','250 N lateral'),('flange_disturbance','Axle-side load'),('air_spring_leak','One airspring vent')]:
  d=load(case);ax[1,1].plot(d['time_s'],1000*d['y_m'],label=lab)
 ax[1,1].set(title='Free 3D response',ylabel='Car A lateral position [mm]');ax[1,1].legend(fontsize=8)
 d=load('flange_disturbance');ax[1,2].plot(d['time_s'],d['flange_normal_N'],label='Flange contact');ax[1,2].set(title='Wheel-flange / rail-side limit',ylabel='Normal force, sampled sum [N]');ax[1,2].legend(fontsize=8)
 for key,lab in [('motor_work_J','Motor'),('pneumatic_work_J','Pneumatic mechanical'),('damping_work_J','Damping'),('constraint_work_J','Contact + constraints'),('mechanical_energy_delta_J','Mechanical energy change')]:ax[2,0].plot(t,A[key],label=lab)
 ax[2,0].set(title='Discrete work audit',ylabel='Energy / accumulated work [J]');ax[2,0].legend(fontsize=8)
 ax[2,1].plot(t,A['mechanical_balance_residual_J'],label='Mechanical closure');ax[2,1].plot(t,A['coupling_work_discrepancy_J'],label='Gas / mechanical coupling');ax[2,1].set(title='Numerical work residuals',ylabel='Residual [J]');ax[2,1].legend(fontsize=8)
 d=load('half_dt');ax[2,2].plot(t,A['speed_mps'],label='dt=0.05 ms');ax[2,2].plot(d['time_s'],d['speed_mps'],'--',label='dt=0.025 ms');ax[2,2].set(title='Independent half-step solve',ylabel='Car A speed [m/s]');ax[2,2].legend(fontsize=8)
 for a in ax.flat:a.set_xlabel('Physical time [s]')
 fig.suptitle('B8 | Source-layout two-car mechanical prototype\nAssumed parameters; raw forward dynamics; no hardware calibration or safety certification',fontsize=16)
 fig.savefig(OUT/'validation_summary.png',dpi=150);plt.close(fig)
 fig,axs=plt.subplots(2,2,figsize=(12,8),layout='constrained')
 for case in ['service_brake','reservoir_loss','brake_cylinder_branch_leak','air_spring_leak']:
  d=load(case);axs[0,0].plot(d['time_s'],(d['brake_pressure_Pa']-101325)/1000,label=case);axs[0,1].plot(d['time_s'],d['speed_mps'],label=case)
 axs[0,0].set(title='Service pressure failure cases',ylabel='Brake gauge pressure [kPa]');axs[0,1].set(title='Resulting motion',ylabel='Speed [m/s]')
 d=load('air_spring_leak');axs[1,0].plot(d['time_s'],(d['air_A_FL_pressure_Pa']-101325)/1000,label='Leaking FL');axs[1,0].plot(d['time_s'],(d['air_A_FR_pressure_Pa']-101325)/1000,label='Sealed FR');axs[1,0].set(title='Unequal bellows pressures',ylabel='Gauge pressure [kPa]')
 for case in ['service_brake','air_spring_leak','flange_disturbance']:
  d=load(case);axs[1,1].plot(d['time_s'],np.rad2deg(d['roll_rad']),label=case)
 axs[1,1].set(title='Pressure and load affect actual roll',ylabel='Car A roll [degrees]')
 for a in axs.flat:a.set_xlabel('Physical time [s]');a.legend(fontsize=8)
 fig.suptitle('B8 | Pneumatic failure and 3D response diagnostics',fontsize=15);fig.savefig(OUT/'failure_modes.png',dpi=150)
 print('plots ready')
if __name__=='__main__':main()
