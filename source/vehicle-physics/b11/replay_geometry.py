"""Independent position-stage geometry audit; never advances the live plant."""
import numpy as np
import mujoco


def contact_geometry(xml_path, replay, metrics):
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    names = [model.geom(i).name for i in range(model.ngeom)]
    radii = np.array([model.geom_size[i,0] if name.startswith('cable_') and
                      name.endswith('_capsule') else 0. for i,name in enumerate(names)])
    body_names = [model.body(i).name for i in range(1,model.nbody)]
    maximum_ratio = maximum_penetration = pose_error = 0.
    worst = None
    checked = 0
    for frame in replay['frames']:
        data.qpos[:] = frame['qpos']
        # Position-stage calculations include collision geometry, but no force
        # solve, integration, state correction or prescribed live trajectory.
        mujoco.mj_fwdPosition(model,data)
        expected = np.array([frame['body_transforms'][name]['position'] for name in body_names])
        pose_error = max(pose_error,float(np.max(np.abs(data.xpos[1:]-expected))))
        if not data.ncon:
            continue
        pair_radii = radii[data.contact.geom]
        indices = np.flatnonzero(np.any(pair_radii>0,axis=1))
        checked += len(indices)
        if not len(indices):
            continue
        limiting_radius = np.min(np.where(pair_radii[indices]>0,pair_radii[indices],np.inf),axis=1)
        penetration = np.maximum(0.,-data.contact.dist[indices])
        ratios = penetration/limiting_radius
        maximum_penetration = max(maximum_penetration,float(np.max(penetration)))
        index = int(np.argmax(ratios))
        if ratios[index]>maximum_ratio:
            maximum_ratio = float(ratios[index])
            ids = data.contact.geom[indices[index]]
            worst = dict(time_s=frame['time_s'],geoms=[names[i] for i in ids],
                         penetration_m=float(penetration[index]),limiting_cable_radius_m=float(limiting_radius[index]))
    lookup = dict(zip(names,radii))
    peak = metrics.get('worst_contact') or {}
    peak_radii = [lookup.get(name,0.) for name in peak.get('geoms',[]) if lookup.get(name,0.)>0]
    global_peak = (dict(time_s=peak['time_s'],geoms=peak['geoms'],penetration_m=peak['penetration_m'],
                        limiting_cable_radius_m=min(peak_radii),ratio=peak['penetration_m']/min(peak_radii))
                   if peak_radii else None)
    return dict(recorded_poses=len(replay['frames']),native_cable_contacts_checked=checked,
                maximum_native_pose_position_error_m=pose_error,maximum_sampled_cable_penetration_m=maximum_penetration,
                maximum_sampled_penetration_radius_ratio=maximum_ratio,worst_sampled_ratio=worst,
                all_step_global_worst_contact_if_cable=global_peak,
                scope='Contact geometry at recorded replay poses; global all-step worst is reported when it is a cable contact. Not every per-contact integration-step maximum is available.')
