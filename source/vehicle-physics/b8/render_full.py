"""Render stored solver poses only. Replay assignments never drive the simulation."""
import os,sys
os.environ.setdefault('MUJOCO_GL','egl');os.environ.setdefault('MESA_SHADER_CACHE_DIR','/tmp/b8-mesa')
from pathlib import Path
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT.parent/'vendor'))
import json,argparse,numpy as np,mujoco,imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont

def run(case='service_brake'):
 out=ROOT/'results';rec=json.loads((out/f'{case}_replay.json').read_text());m=mujoco.MjModel.from_xml_path(str(out/f'{case}.xml'));d=mujoco.MjData(m)
 # Visibility only: fixed room cabinets obscure the train from this camera.
 for g in range(m.ngeom):
  if m.body(int(m.geom_bodyid[g])).name.startswith('access_door_cabinet_'):m.geom_rgba[g,3]=0
 renderer=mujoco.Renderer(m,720,1280);inset=mujoco.Renderer(m,280,430)
 opt=mujoco.MjvOption();opt.geomgroup[3]=0;opt.sitegroup[:]=0
 camera=mujoco.MjvCamera();camera.type=mujoco.mjtCamera.mjCAMERA_FREE;camera.distance=7.8;camera.azimuth=-105;camera.elevation=-17
 zoom=mujoco.MjvCamera();zoom.type=mujoco.mjtCamera.mjCAMERA_FREE;zoom.distance=.88;zoom.azimuth=-90;zoom.elevation=-3
 big=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',24);small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
 with imageio.get_writer(out/f'{case}_solver_replay.mp4',fps=30,codec='libx264',quality=8,macro_block_size=None,ffmpeg_log_level='error') as wr:
  for i,f in enumerate(rec['frames']):
   d.qpos[:]=f['qpos'];mujoco.mj_forward(m,d)
   camera.lookat[:]=(d.body('A_carbody').xpos+d.body('B_carbody').xpos)/2+[0,0,-.10]
   renderer.update_scene(d,camera=camera,scene_option=opt);im=Image.fromarray(renderer.render())
   zoom.lookat[:]=d.body('panto_mount').xpos+[.06,0,.12]
   inset.update_scene(d,camera=zoom,scene_option=opt);im.paste(Image.fromarray(inset.render()),(830,340))
   dr=ImageDraw.Draw(im);dr.rectangle((0,0,1280,87),fill=(16,22,30));dr.text((22,12),'B8 | TWO-CAR FORCE + PRESSURE PHYSICS PROTOTYPE',font=big,fill='white');dr.text((22,47),'Source-model layout | 3D suspension | finite air mass | single-arm linked pantograph',font=small,fill=(193,212,221))
   t=f['time_s'];phase='MOTOR TORQUE' if t<3 else 'COAST' if t<4 else 'PNEUMATIC FRICTION BRAKE' if t<8 else 'PANTOGRAPH EXHAUST'
   if case=='lateral_disturbance' and 1<=t<1.2:phase='250 N SIDE LOAD + 35 Nm YAW MOMENT'
   if case=='collector_disturbance' and 5<=t<5.2:phase='50 N DOWNWARD COLLECTOR LOAD'
   dr.rectangle((0,630,1280,720),fill=(16,22,30));dr.text((22,641),f't={t:5.2f} s   v={f["speed_mps"]:+.3f} m/s   {phase}',font=big,fill=(255,203,87));dr.text((22,680),f'Brake chamber: {(f["pressure_Pa"]["A_front_L_minus_pad"]-101325)/1000:.1f} kPa gauge | Panto: {(f["pressure_Pa"]["pantograph"]-101325)/1000:.1f} kPa | Contact: {f["collector_normal_N"]:.1f} N',font=small,fill='white')
   dr.rectangle((830,308,1260,340),fill=(16,22,30));dr.text((840,315),'ACTUAL LINK POSES / CONTACT',font=small,fill='white')
   dr.text((22,108),'Assumed parameters; lab cabinets hidden for this train view.',font=small,fill=(210,215,224))
   if i==0:im.save(out/f'{case}_poster.png')
   if abs(t-5)<.02:im.save(out/f'{case}_braking.png')
   if abs(t-9.4)<.02:im.save(out/f'{case}_exhaust.png')
   wr.append_data(np.asarray(im))
 renderer.close();inset.close()
 print(out/f'{case}_solver_replay.mp4')
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--case',default='service_brake');run(ap.parse_args().case)
