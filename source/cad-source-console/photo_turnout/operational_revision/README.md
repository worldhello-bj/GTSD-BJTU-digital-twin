# Operational turnout assembly

Independent default profile: `operational_assembly_v1`.

- Source-based replacement: 141 additions / 67 removals, resulting in 2006 meshes.
- Three finite end meshes use a source-located common transverse cut and a total 0.5 mm assembly gap.
- 12 eight-part hold-down assemblies, one long plate, and six pads are retained.
- Actual original intersection: 0.48824768531221413 mm³; revised intersection: 0 mm³. All 233 bounded positive-AABB material checks pass.

## Integration

Read `operational_source_profile.json` and append its one profile to an in-memory profile catalog. It is already flattened against the original source scene. The identity header and source-removal witnesses use the existing registration contract. Keep historical profiles unchanged.

`regional_meshes.json.gz` contains the 205-mesh turnout region. The optional generated export `full_scene_meshes.json.gz` contains all 2006 meshes and is reproduced on demand rather than transferred again. Its expected 24,196,237 bytes and SHA-256 are recorded in `REPRODUCIBLE_EXPORT.json`; it is not required for runtime. Both use world-coordinate metres and flattened triangle indices, with stable source names. `candidate_meshes.json.gz` holds the 111 turnout replacement meshes. `mesh_interface.json` provides bounds, material partitions, topology, volume, centroid, geometry hashes and mechanical relations. `ASSUMPTIONS.md` is the single engineering-assumption page.

Consumers should read `parameters.json` for the default gap and dimensions, then apply their own documented rigid-body/contact mapping to these actual meshes. The independent operational solver is outside this geometry folder. Geometry hashes identify exactly which mesh was supplied to that mapping.

## Reproduce

Use Python with existing NumPy/SciPy/system GEOS and Blender 4.3.2. No extra packages are required. From this folder:

    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python build_operational.py --photo-assets ../ --source-scene SOURCE_SCENE_JSON_GZ --joint-gap-mm 0.5
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python audit_operational.py --photo-assets ../
    OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python test_operational.py
    blender -b --factory-startup -t 1 --python render_operational.py -- .

`--photo-assets` points to the restored historical photo assets containing `photo_source_profiles.json`, `source_seed.json.gz` and `hardware_revision/candidate_meshes.json.gz`. `--source-scene` must match the pinned SHA-256 in `change_manifest.json`. Generated outputs can be directed elsewhere with `--output DIR`. The gap builder supports values in (0, 2] mm and derives the cut plane from source handle `4ECED`; it never rewrites historical meshes. `joint_reference_meshes.json.gz` retains the three before-meshes needed for local parameter tests.

## Optional full-scene export

After the normal public-asset restore (`python prepare_assets.py`), run from `source/cad-source-console`:

    python photo_turnout/operational_revision/export_operational_scene.py --output /path/outside/console/export

The destination must be new or empty and outside the console. The output is `operational/full_scene_meshes.json.gz` inside that directory, with a `verification.json` report. The command checks both the original scene SHA and the exact regenerated export SHA. The optional export requires NumPy, Node and system GEOS; see `requirements-geometry.txt`. Full audits additionally use SciPy, and native rendering uses Blender. The exact tested versions are in `build_environment.json`.

## Evidence

- `geometry_audit.json`: original/revised actual material integration, measured plane gap, all 233 checks, topology and pad/sleeper interfaces
- `mesh_interface.json`: full-source frozen mesh contract
- `native_render_validation.json`: native reopen, exact object membership/triangle indices and world-coordinate roundtrip tolerance
- `operational_turnout.png`: Blender render of the complete regional assembly
- `whole_operational_assembly.png`: latest full 2006-mesh assembly, including the vehicle, pantograph, track and control cabinet; its source hash and full membership are in `whole_operational_render_validation.json`
- `finite_joint_detail.png`: Blender close-up of the actual finite joint
- `GTSD_Operational_Turnout.blend`: editable 205-mesh regional native model
- `test_validation.json`: six passing test groups, including 0.25 / 0.5 / 1.0 / 2.0 mm gap variants across all three end meshes
- `native_image_inventory.json`: one empty Blender Render Result viewer datablock, zero packed or file-backed images

## Native motion render

`native_solver_state.png` and `GTSD_Native_Solver_State.blend` show one actual native solver snapshot, with all bound source meshes transformed by the contract's reference pose and the snapshot's body pose at displacement scale 1. `native_state_render_validation.json` pins the snapshot, summary, assembly and runtime identities. The renderer does not step the solver.

Current image: V2 / h50us, 22,491 native steps at 1.12455 s, wheel crossing the frog-to-wing diagonal gap. This image records actual motion; V2 numerical acceptance remains pending the corrected V3 protocol. The final validated protocol will replace this one image.

To regenerate from a completed case:

    blender -b --factory-startup -t 1 --python render_native_state.py -- --contract CASE/contract.json --snapshot CASE/frog_wing_gap_midpoint_snapshot.json --output .

The case's `summary.json` separately binds runtime identity to assembly identity. Camera framing uses the actual nearest source gap. Reopening verifies every bound triangle index and the transformed world-coordinate tolerance.

## Recorded native-motion video

`animation_v2/operational_motion_v2.mp4` follows the wheel through the complete frozen V2 / h50us trajectory: 37,000 native steps over 1.85 seconds, sampled at 90 actual step numbers and displayed for 7.5 seconds at 12 fps. Every frame transforms all 268 source meshes by the recorded poses of their 11 bound bodies. The 720 × 450 view follows the actual wheel position; displacement scale is 1, with no pose interpolation or solver rerun.

`animation_v2/frame_samples.json` pins every sampled step and source hashes; `motion_inputs.json.gz` contains the frozen geometry, bindings and poses required to reproduce the frames independently of later solver changes. `VIDEO_MANIFEST.json`, `ffprobe.json`, raw-frame hash records and `visual_validation.json` document encoding and review. The five decoded review images and raw PNG frames remain local QA artifacts and are excluded from the public package. Each frame carries native time, step number and V2 runtime identity. A spatial-only non-local-means filter reduces render noise separately within each image; there is no temporal blending. The video records V2 motion; numerical acceptance is tracked by the corrected protocol separately.

    blender -b --factory-startup -t 1 --python render_motion_video.py -- --input animation_v2/motion_inputs.json.gz --frames-dir /tmp/gtsd-motion-frames
    python encode_motion_video.py --frames-dir /tmp/gtsd-motion-frames

All timestamps carry explicit UTC or UTC+08:00 offsets. `DELIVERY_MANIFEST.json` records the final file hashes and validation time.
