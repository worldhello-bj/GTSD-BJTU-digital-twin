# Bounded photo-supported turnout research variants

This is a static research deliverable. There is no new physics binding, added mass, wheel-sweep certificate, manufacturing approval or as-built claim. Original photo bytes, original photo names and Library URLs are excluded.

## Source profile integration

Run `node build_source_profiles.js --source CONSOLE_ROOT`. Serve `photo_source_profiles.json` at a separate fixed photo-profile API. On explicit photo selection, validate its source identity header then merge the two profiles in memory. Keep the historical profiles file byte-exact; do not rewrite it. They are independent source-based alternatives, flattened from the existing PR5 combined profile. Do not apply them on top of `combined`. Keep all historical profiles unchanged.

`profile_mapping.json` lists every exact source remove/add ID and removal geometry witness. `photo_frog_proxy_review` removes 47 original source meshes and adds 46. `photo_frog_hold_down_research` removes 67 and adds 141. The underlying bounded changes relative to the 1928-mesh combined scene are 11 removed / 14 added and 33 removed / 111 added respectively. Their resulting whole-scene counts are 1931 and 2006. The source-safe profile also retains unrelated PR5 improvements.

Validate with `python verify_portable.py --source CONSOLE_ROOT`. Use `--rebuild` to rebuild both variants before profile composition. Dependencies: Python with NumPy/SciPy, system GEOS, Node and Blender 4.3.2 for exact I-section CSG. No network, solver or source-root writes. Run `python test_candidate.py` and `python hardware_revision/test_hardware.py`; material audits run separately with `python audit_candidate.py` and `python hardware_revision/audit_hardware.py`. Rendering is optional: `blender -b -t 1 --python render_native.py -- . hardware_revision`.

## Evidence and limits

The private observation supports a right-pointing taper, independent wings/flangeways, visible lower web, and separate plate/pad/sleeper layers. Source working faces, native finite endpoints and inherited I-section are retained. The 15.333333 mm associated plan gap is a source-CAD association, not a photo measurement. Slot depth, rail cross-section applicability, end joins, plate and pads remain engineering choices.

The preserved proxy review records exactly 18 actual modeled intersections: 17 identical generic octagonal head proxies plus one 0.4882476853 mm³ finite foot/end-joint. The first 17 do not establish actual installed hardware conflicts. Nine proxy placements are historical photo-inferred geometry, six are source-position-only inherited shapes, and two are engineering-offset inherited shapes. CAD fastener detail on another rail is not transplanted without association. The bounded six-station native search found endpoint circles, not verified mounting-hole evidence.

The second variant supplies 12 complete eight-part outside-foot hold-down assemblies at six exact native sleeper axes. Each has a toe clamp, washer, bolt head, shank, captive nut and three-layer seat. The 20 old heads replaced are the 17 intersecting proxies plus three paired heads. This is an explicitly different attachment interpretation with declared M6-sized design dimensions, no hidden rail holes, and no collision-driven shrinking. The retained rail core and five finite source remainders are byte-identical to the first variant. Long plate and six separate pads cover the new seats. Seat welding, physical threaded engagement, final fastening quantity, plate-to-sleeper attachment, frog nose uplift restraint and wheel passage remain unverified. The original through-foot/proxy interpretation is retained as the other profile.

The foreground tall post is the retained source-background object `ENV_substation_rack_upright.001`; it was not removed for presentation. Both Blend files contain only the regional seed plus exact profile replacements, not the whole source scene. Native reopen validation records counts, triangle indices and world-coordinate error. Whole-scene broadphase uses the full inventory; unchanged-versus-unchanged pairs are outside the bounded audit.

The PR5 reference is `ae463b951f831fb552bcd46f3528b4c10c1cf355`. Source geometric identity is enforced by native/base-GLB/display-revision hashes and per-object witnesses. This does not assert that the producer checkout's Git HEAD was that commit. New photo geometry is not bound to the existing PR5 research rig's physics.

## Freeze validation

Full hold-down audit completed at 2026-10-13 15:33:56 UTC / 23:33:56 Beijing: 235 actual material checks, only the 0.4882476853 mm³ inherited finite foot joint remains. The 12 proposed assemblies introduce no modeled positive material volume at the 1e-6 mm³ reporting threshold. This is not proof of field fixation. Native render and reopen completed at 15:34:44 UTC / 23:34:44 Beijing; 205 regional context meshes, 111 candidate meshes, maximum world-coordinate roundtrip error 0.001430512 mm, zero rigid bodies or external libraries. Render was actually viewed.
