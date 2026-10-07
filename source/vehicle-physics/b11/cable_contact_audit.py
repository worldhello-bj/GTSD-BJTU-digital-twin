"""Observe every native position, including settling; never changes force/state."""
import numpy as np


class CableContactAudit:
    def __init__(self, model):
        self.names=[model.geom(i).name for i in range(model.ngeom)]
        self.radii=np.array([model.geom_size[i,0] if name.startswith('cable_') and
                             name.endswith('_capsule') else 0. for i,name in enumerate(self.names)])
        self.states=self.contacts=0
        self.first_time=self.last_time=None
        self.maximum_ratio=self.maximum_penetration=0.
        self.worst=None

    def record(self, data):
        self.states+=1
        self.last_time=float(data.time)
        if self.first_time is None:
            self.first_time=self.last_time
        if not data.ncon:
            return
        radii=self.radii[data.contact.geom]
        indices=np.flatnonzero(np.any(radii>0,axis=1))
        self.contacts+=len(indices)
        if not len(indices):
            return
        radius=np.min(np.where(radii[indices]>0,radii[indices],np.inf),axis=1)
        depth=np.maximum(0.,-data.contact.dist[indices])
        ratio=depth/radius
        if not np.isfinite(ratio).all():
            raise RuntimeError('non-finite native cable contact geometry')
        self.maximum_penetration=max(self.maximum_penetration,float(np.max(depth)))
        index=int(np.argmax(ratio))
        if self.worst is None or ratio[index]>self.maximum_ratio:
            self.maximum_ratio=float(ratio[index])
            ids=data.contact.geom[indices[index]]
            self.worst=dict(integration_time_s=self.last_time,geoms=[self.names[i] for i in ids],
                            penetration_m=float(depth[index]),limiting_cable_radius_m=float(radius[index]),
                            ratio=self.maximum_ratio)

    def snapshot(self):
        return dict(schema='B11 every-integrated-position native cable geometry v1',
                    integrated_position_states=self.states,native_cable_contacts_checked=self.contacts,
                    first_integration_time_s=self.first_time,last_integration_time_s=self.last_time,
                    maximum_penetration_m=self.maximum_penetration,
                    maximum_penetration_radius_ratio=self.maximum_ratio,worst_ratio=self.worst,
                    scope='Initial position plus every completed integration step, including settling. Geometry, not measured material compression or force accuracy.')
