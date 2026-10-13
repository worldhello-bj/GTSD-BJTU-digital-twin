# GTSD CAD static assembly · 2026-10-12 draft stage

> This is the merged historical source-only snapshot. Continuing work is in [source/cad-source-console](../cad-source-console/README.md); its remote transfer status remains explicit. This historical directory was not silently filled with different geometry.

This is a reproducible static CAD/reference reconstruction and a read-only browser viewer. It is not a dynamics model, measured hardware result, complete CAD-library reconstruction, or proof of installed part counts.

## Publication status: source-only draft

The first remote commit contains the viewer and model/export source, tests, and provenance only. The 17 MB compressed assembly and generated native downloads are pending upload. A fresh checkout of this commit cannot yet load the model, run the complete data-dependent tests, or rebuild native models. The local 30-test result below used the full frozen input and is not a claim that this source-only commit is runnable. This draft will be updated with automatic geometry loading and complete verification after the data lands.

## Prepared full-stage contents

- Full compressed triangle geometry: 1,936 default meshes / 1,256,416 triangles; vehicle subset 822 meshes / 415,352 triangles. No decimation.
- 109 semantic registry rows, separate from physical part counts.
- Browser viewer, 30 Node tests, Blender build/export scripts and source provenance.
- Mutually exclusive prior geometry stays in a separate scene. Uninstalled candidate libraries remain separate work and are not automatically installed.

## Open and verify

From this directory:

```sh
node --test tests/test_viewer.js
python scripts/verify_publication.py
python -m http.server 8000
```

Open http://localhost:8000/web/ in a WebGL-capable browser. Files are read-only; the viewer does not upload data or connect to hardware. The CPU fallback refuses views over 350,000 triangles rather than silently dropping geometry. Full vehicle and bench views therefore require WebGL or Blender.

## Rebuild native models

Use official Blender 5.2 or a compatible newer version (with bundled NumPy):

```sh
blender -b --python scripts/build_review_scene.py
blender -b --python scripts/make_delivery_models.py
blender -b --python scripts/verify_final_exports.py
blender -b --python scripts/verify_delivery_models.py
```

Outputs go to outputs/pass3. The first command saves the full review .blend and both full-triangle GLBs without rendering. GTSD_RENDER=1 enables its optional render sequence; separate per-view rendering is preferable on low-memory machines. Generated native binaries and six existing rendered views may be added to this same draft separately; until then viewer download buttons require a local rebuild. The frozen input alone is sufficient for these build/export commands. Source preparation from every upstream engineering intermediate is not claimed by this stage.

## Evidence and limitations

The frozen upstream assembly passed 4,070 independent source/replacement checks and complete GLB triangle comparisons. Its Blender review verified 1,936 default meshes, 109 semantic rows, 371 mutually exclusive prior meshes and 822 display-only exploded meshes. New default objects explicitly have no physics binding. These checks concern the upstream artifacts, identified in evidence/upstream_independent_review_summary.json; this public copy is separately tested below.

The original CAD material is already in this repository under reference/试验台几何建模资料-1011 at source commit d78e0e4616f9a491e46a5c03637755971a9ec92c. Follow that existing source tree; no duplicate original drawings, course PDFs or original private photographs are included here. Photo-derived approximations are labelled as assumptions, not measured source solids.

Only a legacy shell-contact metadata key was renamed in the public input to avoid implying current collision physics. All mesh vertices, triangle indices and registry entries are unchanged. Upstream and published hashes are recorded in evidence/publication_provenance.json. The body floor was already correctly offset in the source and was not translated.

Browser end-to-end interaction has not been accepted in this stage. Thirty Node tests and static source checks are a narrower result. Unknown transverse depths, thickness, material and installation relations remain explicit candidates. Native local/source files and the prior B9–B12 PR are not modified by this addition.
