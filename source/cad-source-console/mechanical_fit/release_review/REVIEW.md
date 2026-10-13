# GTSD mechanical-fit release review

Read-only review, 2026-10-13. All paths below are relative to the console release root. No solver run, Blender run, source-original redistribution, or main-repository edit was performed by this review.

## Findings

1. Candidate detail identity could cross between alternatives. `integration/web/app.js` formerly collected all profile meshes in one name-keyed map. The two common-support meshes have the same names in the conforming-hull and flat-top alternatives but different triangle witnesses, volumes and support-mode metadata. The final combined profile overwrote the flat-top entries. Rendering remained flat-top while selection details described the conforming hull. The integration owner changed the map to the successfully selected profile and clears it on return to source. The added same-name/different-geometry regression passed during read-back.
2. Live display categories were not adjusted after mechanical-fit substitution. A one-world-mesh fixture replaced with a body-free candidate still displayed “1 world static / 0 static candidates.” In the real combined profile, 30 native world-static meshes, one existing static replacement, and six unbound meshes are replaced with 33 static candidates. The live category totals consequently described the pre-fit scene rather than the 1928-mesh candidate scene. The correction now adjusts the categories using actual removed mesh names and their world-static classification, then adds candidate meshes. The one-world-mesh → candidate → source regression passed during final read-back.

Both reproducible findings are resolved in the reviewed working tree. No remaining code blocker was found. The final publisher must refresh and verify the publication manifest after the integration edits.

## Verified coverage

- Reviewed `mechanical_fit/static_profiles.js`, `build_profiles.js`, `export_scene.js`, the assembly selector/detail/render flow in `integration/web/app.js`, `integration/web/index.html`, and the new fixed GET routes in `integration/source_console.py`.
- Reviewed focused fixtures in `integration/tests/app_harness.js`, `test_fit_profile_app.js`, `mechanical_fit/test_static_profiles.js`, `test_public_profiles.js`, and `test_rail_view.js`.
- Twelve initial focused Node tests passed. These include all 19 declared real profile/scene combinations; original source 1932, combined 1928, 37 removed and 33 added; unchanged mesh object preservation; zero candidate mass/collision; failed admission; explicit source restoration; atomic partial groups; and the finite-curve rail view. Final read-back ran 15 focused Node tests successfully, including the new same-name detail, live classification, and later-registered dynamic protection regressions. See `final_focused_tests.tap`.
- Independently delayed the initial profile response, changed to an unsupported scene, then completed loading. The candidate was rejected, the newer source scene remained selected, and the selector was re-enabled. A loaded clamp profile also returned to source on an unsupported scene change. Neither flow sent a solver POST. See `async_flow_results.json` and `review_async_flows.js`.
- Actual combined/clamp upper and lower scopes retain the original 11/19 source meshes, install zero candidate meshes, and report the incomplete clamp group. They do not apply a partial group.
- Every current profile removal was checked against frame/track registration rows and bogie rigid rows; no current removal intersects those registrations. The seven supplied solver/mapping file hashes match the physics-separation evidence. The final loader also rejects removal of meshes subsequently admitted as frame, rigid or coil dynamics, even when the historical mapping calls them unbound. The console, profile builder and scene exporter all pass the real protection names, including coil `mesh_name` values.
- Checked the README/report boundary claims against the supplied component and native-render evidence: known-overlap B535 is explicitly labelled, clamp/combined profiles only admit the default front-route scene, original seven overlaps are not attributed to the conservative default, retained 4079/contact-wire overlap is disclosed, and no full-model clearance or full-motion claim is made.
- H1 C uses the outer-hole axis, while the separate circular-fit study uses the tie-hole axis. The five research hinges have no hardware-validated travel limits. The 7.9 mm study pin and unresolved axial stack/tolerances remain explicit.
- The profile payload hash and native Blend hash match the report. The supplied reopened-Blend verification describes eight scenes and 1970 unique streams; the report and nine-render evidence agree on those identities and counts.

## Asset and manifest checks

- `ASSET_MANIFEST.json` retains all 57 original entries byte-for-byte at the data-field level and adds exactly 20 mechanical-fit assets.
- All new chunk sizes/hashes, concatenated packed sizes/hashes, decompressed sizes/hashes and current materialized asset bytes match.
- Restored all 20 new assets using the actual `prepare_assets.ensure_assets` code into an independent empty directory, then repeated its read-only verification: 20 restored, then zero restored and 20 checked. No network was used. See `asset_restore_check.json`.
- The checked publication-manifest snapshot had 1317 entries, all present with matching size/hash. Subsequent integration edits require the publisher's final manifest refresh and final check.

## Explicit limits

This is code, contract, identity, mocked-DOM and asset-restoration review. No actual browser/WebGL/canvas visual acceptance, browser downloading, hardware acceptance, long-running dynamics, swept clearance, fresh geometric collision solve, or fresh Blender reopen/render was performed. Existing HTTP/step/restore and broader Python/Node results were reported by the integration owner; they were not rerun wholesale here. Original private photos, drawings and PDFs were not needed or redistributed.
