# Full-passage numerical verification

The V1 100/50 µs study completed the 1.85 s route, including actual positive contact on the approach, frog, upper wing and right main rail. Four contact interfaces carried force, mass and source geometry were registered, and exact checkpoint continuation matched.

Three V1 gates failed:
- Maximum wheel/upper-wing penetration: 185.5005966 µm at 1.1726 s, above the 100 µm limit.
- Maximum aligned body-position difference: 55.0713633 µm, above the 50 µm limit.
- Wheel vertical-velocity difference: 0.0378726 m/s, above its 0.0216186 m/s limit. The other 20 native DOF velocity checks passed.

V1 terminal energy residual was small, but the full trace retained a larger 0.0209248722 J startup residual at 0.0111 s. V2 evaluates maximum absolute residual over the whole run against a whole-run energy/work scale.

The original V1 5 ms force calculation decimated fine native samples before averaging. Its saved impulse calculations are correct, but its force-window result is retained as a historical diagnostic only. V2 independently averages every 50 µs sample and every 25 µs sample over equal 5 ms windows.

The targeted V2 change uses a separate 1.2 ms wheel-contact response time, compared with the earlier 3 ms. Pin and jaw contacts remain at 3 ms. It shortens the regularized wheel-landing response without changing source geometry, wheel geometry, mass, friction or the other force paths. Default time step is 50 µs and the comparison is 25 µs. The original position, penetration, velocity, impulse and energy thresholds are retained.

V2 also requires actual positive contact on every route source family and less than 2 mm track-relative wheel drop. This is separate from mere X-position threshold crossings. The geometric 24.423310 mm tread-support gap for a 45 mm radius wheel corresponds to approximately 1.69 mm quasi-static sag; the dynamic bound is declared at 2 mm.

The 1.85 s endpoint marks completion of the commanded passage, not mechanical rest. The final snapshot preserves native translational/spin velocities and instantaneous contact state. Continuing native steps after this endpoint uses a zero drive target and resolves remaining motion; no velocity is forcibly reset.

V2 completed both native trajectories and passed source-route contact, all-branch motion, energy/work closure, exact recovery and warning checks. The full h/h2 protocol remains failed: the 25 µs case reports a 2.062432 mm pin/F643-cell contact distance at 1.490225 s; wheel vertical pose/velocity and selected pin force comparisons exceed the retained limits. Default 50 µs wheel landing penetration is 86.5585 µm. These outcomes are retained without relaxing thresholds.

V3 retains the 50/25 µs step sizes, 1.85 s passage, force laws and all declared acceptance thresholds. Its two targeted representation changes are:

1. The wheel's normative solid is an analytic cylinder. Native collision and analytic inertia use the same 45 mm radius, 12 mm width and 1 kg mass. The 192-face display is inscribed, with maximum radial difference 6.023793 µm; its small polygonal boundary error is explicitly recorded.
2. A model-specific compiled-geometry projection certificate removes only strictly separated pin/cell candidate pairs. Rejection requires positive axial separation greater than 1 pm plus a conservative floating-point error bound. Actual overlap and uncertain cases still use the original native contact law.

The V2 pin event was independently tested using the same preintegration pose. Compiled vertices are separated by 85.09134 pm and 25.93419 pm on the pin axis; long-double projection corroborates these signs. An 18-point perturbation sweep found four false deep contacts on the separated side and eight true shallow contacts on the overlap side. The V3 filter removes the four false contacts and preserves all eight real contacts, including point, normal and distance. Source float64 and native float32 mesh quantization are reported separately; a source-space overlap is not used to claim native compiled separation.

The guard code is part of the model identity. Mixed import names share one dispatcher, registration is native-model-specific, and the guard has no evolving integration state. Four unknown-model parent/weld/filterparent combinations retain exactly the native contact points, normals and distances. MuJoCo 3.3.7 body-pair filters occur in `mj_broadphase`; the geom callback replaces the later bitmask branch. The specific-version source is https://raw.githubusercontent.com/google-deepmind/mujoco/3.3.7/src/engine/engine_collision_driver.c .

V3 source and parameters were frozen at 2026-10-14T03:20:47Z. The full protocol completed at 2026-10-14T03:35:11.204149Z (Beijing 11:35:11). Twelve of fourteen gates pass. The pin artifact is eliminated; peak penetration is 81.319889 µm, normal-impulse error at most 0.271593%, and 5 ms force error at most 3.772989%. The wheel landing still fails aligned position (143.126983 µm) and vertical velocity (0.152181446 m/s) convergence. These two gates remain open; source geometry and the declared thresholds are unchanged.

Subsequent bounded diagnostics excluded artificial prism bottoms, internal triangle seams and insufficient Newton iterations as the first-impact cause. Common-state 1.10–1.20 s trials evaluated increased normal damping, a complete 0.5 mm wheel-rim fillet, and native RK4 without changing the acceptance thresholds. None was adopted into production. A final 25/12.5 µs native RK4 state-only comparison passed all body-position and all 21 velocity-DOF gates (15.271472 µm and wheel-Z 0.022148802 m/s against 0.024429740 m/s). RK4 stage work/force/impulse accounting and complete-route qualification remain unperformed. See CONTACT_DIAGNOSTICS.md and contact_window_native-rk4-refined.json; these do not change V3's 12/14 full-protocol result.
