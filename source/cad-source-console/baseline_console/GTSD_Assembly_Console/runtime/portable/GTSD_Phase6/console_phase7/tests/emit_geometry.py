from fixture import P,CONFIG
cfg=CONFIG['base'].copy();cfg['variant']='cad-body-mounted-research';S=P.phase6_module();S.verify_cad_freeze(cfg['cad_body_root']);S.source_modules(cfg)
if CONFIG['variant']==P.DOORS_VARIANT:
 P.door_factory(CONFIG)
 from cad_doors.geometry import resource
else:
 from cad_body.geometry import resource
import sys
sys.stdout.buffer.write(resource()[0])
