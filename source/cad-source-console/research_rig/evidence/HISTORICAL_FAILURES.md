# Preserved earlier numerical findings

- Initial isotropic ball damping used minimum principal inertia. The slender
  4079 transverse mode remained underdamped at two seconds. The corrected model
  uses maximum principal inertia and records the resulting per-axis damping
  ratios. This is an explicit research damper choice, not parameter calibration.
- Nominal D5 pin/slot exact tangency made the zero-clearance transient sensitive
  to step size. It stays selectable and labeled unqualified.
- The earlier 200/100 µs contact studies and strict absolute-only 1e-4 velocity
  checks did not all pass. Complete source records and source-code versions are
  retained separately. They are not called successful because a later smaller
  step and distinctly named protocol pass.
- V2 explicitly evaluates each velocity DOF against max(1e-4, 2% of that DOF's
  peak), plus raw force peak, 1 ms mean peak, impulse, onset, pose, energy and exact
  replay. The delivered 100/50 µs D4.98 run is identified by its own model hash.
