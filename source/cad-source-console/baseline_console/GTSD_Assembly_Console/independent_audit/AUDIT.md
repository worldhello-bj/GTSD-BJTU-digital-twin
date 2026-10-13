# GTSD assembly console independent lightweight audit

Date: 2026-10-12 UTC. Scope: assembly_server.py, web/core.js, web/app.js, existing small tests and JSON evidence in `/workspace/shared/gtsd-console-assembly-review-1012`.

No main source files were edited by this audit. No MuJoCo, Blender, Chromium, hardware, user computer, full GLB parse, new numerical stepping or publication was used. The parent is separately performing native HTTP acceptance.

## Final result

All four reproduced UI/control issues and the synthetic invalid-pose source-selection mismatch were corrected by the parent and independently retested here. Final small suites: 11/11 UI VM regressions, 11/11 core tests, and 6/6 fake-controller route tests passed. No additional verified issue remains within this audit’s limited scope. This does not constitute browser/GPU or full-physics acceptance.

## Findings and disposition

1. **Running observation could renew the run lease while stale state disabled Pause.** Original app.js controls and core.command required freshness for Pause, while poll renewed `/api/state` unconditionally whenever phase was running. A 3-second-old running snapshot reproduced both behaviors in a Node VM. The parent has changed Pause to require a known session only, separated its pending receipt, stopped renewals as soon as Pause is sent, and made heartbeat conditional on mapping/freshness. The corresponding post-fix tests pass.
2. **An indefinitely pending GET left stale UI looking current.** Original poll scheduled its successor only after the network settled. Six seconds without a new snapshot left Run/live enabled and the last `真实原生姿态` badge unchanged. Core's command-time guard still prevented Run from being transmitted, so the test did not demonstrate unsafe motion. Parent added independent 200-ms freshness checks and bounded requests. The post-fix test shows expiration before the hung GET settles and polling resumes after its timeout.
3. **Recovery to the same paused session/step kept the stale badge.** Original rerender key only used session:steps. The parent invalidates that key on loss of freshness; recovery to the exact same frame now rerenders and clears the stale badge. Regression passes.
4. **Late POST transport failure can overwrite a confirmed applied ACK.** A command's POST can remain pending after an independent runtime poll has already returned its exact applied event. Original and first revised send.catch then replace the correct applied status with `命令结果未知` when the old request times out. `test_app_regressions.js` contains a deterministic reproduction. Remediation should associate late completion/errors with the exact outstanding command and ignore them after an ACK/session change or replacement. Parent fixed this by comparing the submitted request ID with the currently outstanding request before handling transport errors. The late-error-after-applied-ACK regression now passes.

## Lower-confidence hardening observation

With a deliberately malformed runtime response missing its required native body pose, `follow()` throws as intended. If a different source scene is then selected, `render()` catches without replacing the old resource, so the dropdown can say scene two while the retained resource is scene one. This is a synthetic invalid-payload test, not evidence that the real native server emits missing poses. Parent addressed it: failed pose frames explicitly name the retained scene and say it is not a current valid pose; selecting a new scene falls back to that exact static source. Both paths pass the final VM regression. This did not bypass numerical or collision guards.

## Verified controls and source integrity

- Freshness and native-model identity, native-source geometry identity, variant, latest native identity, and base-GLB identity are checked before pose following.
- Missing/nonfinite poses and nonunit quaternions fail closed; reference rotation is inverted before native rotation and translation, once; source arrays are not changed in the focused unit tests.
- Source selection is exclusive. The metadata-only cross-check confirms the catalog/mapping SHA-256 values, exact union names, default name-set digest, and all per-scene row memberships.
- All 1,950 mapping rows forbid new collisions. All pose-follow rows are rigid, non-world, exact world/oriented triangle matches with reference poses.
- Read-only `/api/assembly/runtime` explicitly calls `snapshot(heartbeat=False)`. Its fake-controller route tests verify no lease or state mutation. Parent inherited command routes, loopback host/origin authorization, receipt queue and single-owner numerical controller remain composed rather than replaced.
- HTTP 202 alone does not mark a command applied. ACK is matched by session and request ID; unrelated events do not release the pending control. Step ACK is not confused with completion: `single_steps_remaining` continues to gate Step. Unknown responses do not resend. Session rollover releases the old pending request without replay.

## Physics/completion language

No reviewed page, runtime response, mapping policy or existing asset-parsing report claims the new clamp geometry participates in solver collision or that the full physical model is completed. The page explicitly retains source-orientation/hole/interface conflicts, uncalibrated/as-built limits and `new collision = 0`. Catalog flags `new_physics_enabled` and `as_built` are false. Existing geometry evidence reports 466 dynamic visual meshes, 764 world-static meshes and 703 static/reference/candidate meshes in the default scene; it explicitly says this was initial-pose asset parsing, not browser or collision acceptance.

## Evidence

- `reviewed_sources/`: original code snapshot used for the first findings; hashes in `reviewed_source_sha256.txt`.
- `test_app_edge_cases.js`, `app_edge_cases.json`: original four-case Node VM reproductions. Reproduce against frozen originals with `ASSEMBLY_ROOT=/workspace/shared/gtsd-console-assembly-audit-1012/reviewed_sources node test_app_edge_cases.js`.
- `app_harness.js`, `test_app_regressions.js`, `app_regressions.log`: small reusable regression harness. No DOM renderer, real browser, network or full model needed. Set `ASSEMBLY_ROOT` to review a different target; when these files are placed in the console's tests directory, the adjacent source root is discovered automatically.
- `core_tests.log`: 10/10 original focused core tests passed. `core_tests_final.log`: 11/11 passed after the parent fixes and added Pause test.
- `routes_tests.log` and `routes_tests_final.log`: 6/6 existing stub-controller route tests passed before and after the parent fixes.
- `final_reviewed_source_sha256.txt`: exact final main-source versions independently tested.
- `metadata_guards.json`: actual small-file catalog/mapping checks, all expected checks true.

Existing `evidence/doctor.json` reports dependency admission success, but is not a browser/numerical acceptance test. `doctor_wrong_interpreter.json` correctly records dependency-version rejection. Existing full-asset parsing evidence was inspected, not independently rerun here. Browser pixel fidelity, interaction performance, WebGL/Canvas recovery, and long numerical convergence are outside this audit's verified scope.
