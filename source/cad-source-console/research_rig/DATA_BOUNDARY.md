# What remains before an as-built 1:1 claim

The existing CAD and eight newly reviewed photographs establish more topology
than the earlier model. In particular, the photographed crossing has a tapered
nose, separate wing/guard faces, an open flangeway, a continuous base plate and
distinct pads/fasteners. Treating two complete I-section rails as intersecting
solids is not an adequate reconstruction. The separate photo revision implements
that topology while keeping dimensions inferred from CAD distinct from depth and
fastening hypotheses.

| Interface | Established evidence | Minimum unresolved field / unit | Why it matters / useful measurement |
|---|---|---|---|
| Frog, wing and rail tail | Native source edge paths and I-section levels; photo confirms separated working faces | Slot cut depth (mm), tail/end-circle function, which rail/joint version is installed | A perpendicular scaled side/end view or section/detail would distinguish full-depth separation from a relieved/overlapping foot. Do not assign the native 1.3333 mm end circles to M6/M8 holes. |
| Frog hold-down assemblies | Source station axes, rail envelope, metal sleepers and visible fastener topology | Actual bolt axes/diameters (mm), clip section (mm), underside fixing, preload (N or measured tightening method) | The former 10×10×16 mm octagonal heads are display proxies; their penetrations do not prove actual bolt clashes. New complete mounting candidates stay labeled until these fields close. |
| F643 pin / B535 bore | Native plate/slot material and competing B535/8A2C center references | Installed center reference, pin diameter/tolerance (mm), axial retention and available travel (mm) | D5 and D4.98 are distinct study geometries. Calipers plus both pin-end views resolve clearance/retention; a picture alone cannot establish a 20 µm diameter change. |
| 4079 link / upper clamp / wire | Source front projection and assembled visual topology | Actual depth/section (mm), upper/lower attachment, wire diameter and tension (N), sag (mm) | Determines lever arm, possible contact and whether the link should be a tie, hinge, slider or fastener. Current retained positional alternatives do not establish these constraints. |
| Balise support | Source enclosure dimensions and candidate source station bays; photo confirms a visible separate enclosure | Device ID vs station, installed yaw, screw/underside fixing coordinates (mm), complete mass (kg) | A readable label and underside/fixing view ties the particular device to a source bay. The study uses 0.6 kg explicitly; it does not infer mass from a solid plastic/steel envelope. |
| Elastic/dissipative properties | Real force paths and numerically closed energy accounting in the research rig | Component masses (kg), rig/support stiffness (N/m, N m/rad), damping, friction, joint preload | Weigh parts/groups; use bounded load–deflection and decay tests, then a documented friction/slip test. These parameters control quantitative response, not just appearance. |

## Software work that does not require new measurements

- Extend material-preserving collision decomposition and force-owner manifests to
  the explicitly excluded tube/ring/lower-jaw/rail support interfaces, using a
  chosen labeled research assembly. This remains possible software work; it is
  not counted as completed by the current two-pin contact tests.
- Give the photo-layered turnout its own mass/constraint/collision identity and
  repeat affected coupling tests after a particular mounting candidate is chosen.
  No old strict82/H1 or PR5 numeric acceptance transfers automatically.
- Real browser interaction requires a supported browser in the same network as
  the local server, or a separately authorized verified hosting setup. Current
  HTTP/DOM checks are not a substitute for that visual acceptance.

## This release's stopping condition

Freeze the source-indexed photo geometry, keep unresolved fastening candidates
visible, deliver the independently runnable three-branch rig and bounded console,
and finish actual contact/step-size/energy/checkpoint/HTTP tests. Report each
remaining interface rather than continuing unbounded guesses or relabeling a
numerical research success as full-machine or as-built calibration.
