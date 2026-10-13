"""Runtime adapter. Release touches eq_active only; forces never drive poses."""
import copy,hashlib,json,math
import numpy as np
import mujoco

def digest(v):return hashlib.sha256(json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def finite(v):return type(v) in (float,int) and math.isfinite(v)
class CableRuntime:
    def __init__(self,model,data,spec):
        self.model,self.data,self.spec=model,data,copy.deepcopy(spec)
        self.routes={r['key']:r for r in spec['routes']};self.events=[];self.loads={k:[0.,0.,0.] for k in self.routes}
        self.identity={'schema':'gtsd-v5-cable-runtime-v1','spec_sha256':digest(spec),'dimensions':[model.nq,model.nv,model.neq],'dt_s':float(model.opt.timestep)}
        self.command_registry={'cable_release':{'label':'释放候选线缆端接（仅约束断开）','args':{'key':{'type':'string','enum':list(self.routes)},'endpoint':{'type':'integer','minimum':0,'maximum':1}}},'cable_force':{'label':'线缆末节外部试验力（N）','args':{'key':{'type':'string','enum':list(self.routes)},**{a:{'type':'number','minimum':-2.,'maximum':2.} for a in ('fx_N','fy_N','fz_N')}}}}
        self.controls=self.command_registry
    def command(self,name,args):
        if type(args) is not dict or name not in self.command_registry or set(args)!=set(self.command_registry[name]['args']):raise ValueError('invalid cable command schema')
        key=args['key']
        if type(key) is not str or key not in self.routes:raise ValueError('unknown cable route')
        if name=='cable_release':
            e=args['endpoint']
            if type(e) is not int or e not in (0,1):raise ValueError('endpoint must be 0 or 1')
            eq=self.model.equality(f'v5c_{key}_attachment_{e}').id
            if self.data.eq_active[eq]:
                self.data.eq_active[eq]=False;self.events.append({'key':key,'endpoint':e,'time_s':float(self.data.time)})
        else:
            f=[args[a] for a in ('fx_N','fy_N','fz_N')]
            if not all(finite(v) and abs(v)<=2 for v in f):raise ValueError('finite force components within +/-2 N required')
            self.loads[key]=list(map(float,f))
    def apply_forces(self):
        """Call once after global xfrc_applied reset, before solver step."""
        for k,f in self.loads.items():
            bid=self.model.body(self.routes[k]['segments'][-1]['body_name']).id
            self.data.xfrc_applied[bid,:3]+=f
    def snapshot(self):
        m,d=self.model,self.data;out={};proxies=[]
        for key,r in self.routes.items():
            errors=[];active=[]
            for e,(s,endpoint) in enumerate(((r['segments'][0],'a'),(r['segments'][-1],'b'))):
                point=d.site(s['body_name']+'_'+endpoint).xpos;anchor=d.site(r['anchors'][e]['site']).xpos
                errors.append(float(np.linalg.norm(point-anchor)));active.append(bool(d.eq_active[m.equality(f'v5c_{key}_attachment_{e}').id]))
            for s in r['segments']:
                bid=m.body(s['body_name']).id
                proxies.append(dict(kind='capsule',label='数值分节线缆代理',route=key,body_name=s['body_name'],position=d.xpos[bid].tolist(),quaternion=d.xquat[bid].tolist(),a_world_m=d.site(s['body_name']+'_a').xpos.tolist(),b_world_m=d.site(s['body_name']+'_b').xpos.tolist(),radius_m=s['radius_m']))
            out[key]={'endpoint_error_m':errors,'attachments_active':active,'external_force_N':self.loads[key], 'candidate_binding':True}
        return dict(schema='gtsd-v5-cable-snapshot-v1',time_s=float(d.time),identity=self.identity,routes=out,solver_proxies=proxies,hidden_source_meshes=self.spec['hidden_source_meshes'],release_events=copy.deepcopy(self.events))
    def checkpoint(self):
        return dict(schema='gtsd-v5-cable-checkpoint-v1',identity=self.identity,attachments={k:[bool(self.data.eq_active[self.model.equality(f'v5c_{k}_attachment_{e}').id]) for e in (0,1)] for k in self.routes},loads=copy.deepcopy(self.loads),events=copy.deepcopy(self.events))
    def restore(self,value):
        expected={'schema','identity','attachments','loads','events'}
        if type(value) is not dict or set(value)!=expected or value['schema']!='gtsd-v5-cable-checkpoint-v1' or value['identity']!=self.identity:raise ValueError('cable checkpoint identity mismatch')
        if type(value['attachments']) is not dict or set(value['attachments'])!=set(self.routes) or type(value['loads']) is not dict or set(value['loads'])!=set(self.routes):raise ValueError('cable checkpoint routes mismatch')
        for k in self.routes:
            a=value['attachments'][k];f=value['loads'][k]
            if type(a) is not list or len(a)!=2 or any(type(x) is not bool for x in a):raise ValueError('invalid attachment state')
            if type(f) is not list or len(f)!=3 or not all(finite(x) and abs(x)<=2 for x in f):raise ValueError('invalid force state')
        events=value['events']
        if type(events) is not list or len(events)>2*len(self.routes):raise ValueError('invalid release events')
        for e in events:
            if type(e) is not dict or set(e)!={'key','endpoint','time_s'} or type(e['key']) is not str or e['key'] not in self.routes or type(e['endpoint']) is not int or e['endpoint'] not in (0,1) or not finite(e['time_s']) or e['time_s']<0:raise ValueError('invalid release event')
        # The host restores the paired MjData integration state before this
        # module. Events must describe exactly its bounded release-only history.
        now=float(self.data.time)
        if not math.isfinite(now) or now<0:raise ValueError('invalid paired solver time')
        released={(k,end) for k in self.routes for end,active in enumerate(value['attachments'][k]) if not active}
        seen=set();last_time=-1.
        for event in events:
            tag=(event['key'],event['endpoint']);when=event['time_s']
            if tag in seen:raise ValueError('duplicate cable release event')
            if when>now+1e-12:raise ValueError('cable release event after paired solver time')
            if when<last_time:raise ValueError('cable release events out of time order')
            seen.add(tag);last_time=when
        if seen!=released:raise ValueError('cable release events do not match attachment state')
        for k in self.routes:
            for e,a in enumerate(value['attachments'][k]):self.data.eq_active[self.model.equality(f'v5c_{k}_attachment_{e}').id]=a
        self.loads=copy.deepcopy(value['loads']);self.events=copy.deepcopy(events)
