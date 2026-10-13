# Mechanical fit review and explicit engineering alternatives

Base: merged main `c5dd336857fe7b573443bc6c1721e4883584acc5`.

## Run

Use the console's normal `python prepare_assets.py` and launch instructions,
then open `/assembly`. The **mechanical fit** selector defaults to the original
source display. Select a support, balise, clamp, or combined alternative to
compare real triangle geometry. `/mechanical-fit/report` provides rendered
before/after views and the remaining source questions.

The combined default-scene alternative changes **37 old display meshes into
33 candidate meshes**, yielding **1928**, compared with the source display's
**1932**. No native moving geometry can be replaced by these profiles. The
loader verifies the base GLB, native identity, source-display revision and
each removed/added geometry witness before admission. Assemblies are atomic:
a partial scope never installs half a replacement group. Returning to Source
restores the exact source display. Clamp variants are admitted only for their
reviewed front-route scene.

## What the fit result means

The historical pending support range is **7 groups / 17 meshes / 7 positive
material pairs** when all proposed source-position movements are enabled.
The conservative default kept those seventeen old positions. Four pairs are
addressed by explicit common support/clip alternatives, and three by the
balise installation alternative. An additional six clamp non-connection
intersections are addressed by the primary clamp alternative. Component and
combined reports define the precise checked set; none certifies every pair
in the whole railway assembly.

The common support adds material rather than removing intersecting source
material. The balise keeps the inherited photo-inferred enclosure shape and
uses an explicit 90-degree mounting choice. The clamp selects an explicit
source hole-location version and an unconfirmed round-pin cross-section.
These are inspectable, configurable engineering candidates, **not an as-built
claim or a manufacturing drawing**. Dimensions, added volumes, clearances,
source handles, transformations and unapplied alternatives are preserved in
`components/` and `inputs/`.

The separate B535 review alternative intentionally retains the newly found
pin/profile conflict; its selector says so. Optional whole-messenger alignment
is separated from the primary six-piece clamp profile because its end fitting
remains unresolved. The two 4079/contact-wire interfaces remain positive
material intersections and cannot be reclassified as a confirmed weld or
fastener merely because they look like a connection.

## Physics and joints

All new candidate meshes are static, with zero new mass, bodies, constraints
and collision. No old physics result is attributed to the candidate supports
or clamps. Existing verified body-following and controller commands remain
available for the unchanged admitted rigid bodies. Runtime regression and
checkpoint tests check this separation.

`pantograph_clearance.py` evaluates the actual native C/D circle centers. The
0.005876/0.007679 mm offsets are not automatically interference: an 8 mm
nominal pin on the tie-hole axis fits the 8.2 mm outer hole. The default 7.9 mm
pin is an explicit replaceable engineering parameter, not a measured part.
The report separates exact circle fit, polygon-render fit, unknown tolerance,
axial stack and the unchanged source geometry. The existing H1 C joint uses the
outer-hole axis, whose admissible pin diameter is correspondingly smaller
(about 7.988/7.985 mm at C/D). The 7.9 mm study parameter fits either axis choice.
H1 joint travel is not hardware-limited; short-step solver regression is not
a full-motion swept-clearance test. See `evidence/physics_separation.json`.

## Reproduce

From the console root:

```sh
node mechanical_fit/build_profiles.js
node --test mechanical_fit/test_*.js
python -m unittest discover -s mechanical_fit -p 'test_*.py'
python mechanical_fit/pantograph_clearance.py --help
node mechanical_fit/export_scene.js /tmp/gtsd-fit-scenes
blender -b --factory-startup -t 1 --python mechanical_fit/build_native.py -- \
  --input /tmp/gtsd-fit-scenes --output /tmp/GTSD_Mechanical_Fit_Candidates.blend
blender -b --factory-startup -t 1 --python mechanical_fit/verify_render_native.py -- \
  --input /tmp/gtsd-fit-scenes --native /tmp/GTSD_Mechanical_Fit_Candidates.blend \
  --out /tmp/gtsd-fit-render-check
```

The eight native scenes share unchanged mesh data. Every scene membership and
every float32 vertex/index stream is verified after reopening the Blend.
The delivered model contains no embedded images, external libraries, text
scripts, historical producer paths, simulation or animation. Render evidence
contains nine actual Blender views and unchanged-geometry proof after removal
of historical machine-path metadata. HTTP/DOM checks and actual Blender image
review are distinct from browser WebGL visual acceptance.

For algorithmic component rebuilds, see `components/track_support/PORTABLE_REBUILD_ZH.md`,
`components/balise/README_ZH.md` and `components/clamps/README_ZH.md`. These
builders use public restored geometry plus included small source/configuration
seeds. Their extra geometry dependencies do not change the console's MuJoCo
environment requirements. No original photo, DWG or course PDF is required.

## Remaining precise inputs

See `report.json.remaining_fields`: switch/frog finite-curve ownership and
state, flangeway cross-section, common support manufacture and fixing holes,
balise measured mounting, 4079 depth/end fixing, F643 section, conflicting
B535/8A2C hole-location version, pin retention/tolerance, and C/D axial stack.
Unchanged rail-body crossing overlap is visible and separately reviewed; it
is not eliminated by changing support plates or suppressing a view.
