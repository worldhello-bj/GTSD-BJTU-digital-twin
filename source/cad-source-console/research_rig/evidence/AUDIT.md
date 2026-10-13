# Bounded numerical and console audit

The tested release has 33/35 distinct source mass owners, 50/56 velocity DOF and
zero/950 native collision geometries respectively. All collision geometries have
zero mass/density, and no source owner is counted twice. Native body mass equals
the declared sum. Native inertia tensor reconstruction differs from triangle
integration by at most 4.06e-7 relative (compiler principal-axis precision).

Complete D4.98 pin source/display and native collision vertices agree within
3.59e-10 m. Their mass ratio to D5 is 0.992016. At a nonzero native gravity state,
all 35 JavaScript source-to-body transformations agree with independent native
rotation-matrix transforms within 4.44e-16 m. No motion scale is applied.

In 60 independently checked native steps, contact point-wrench reconstruction
matches generalized constraint force to 3.33e-16 in each applicable N / N m
component. Native integration state, command input, clock and energy ledgers
restore and continue exactly. An injected failure after native mutation rolls
back the complete asynchronous run, with no partially committed steps.

Two found defects were corrected and regression-tested:

- Nondividing time steps use a whole-step floor, so a nominal five-second budget
  cannot round above its bound.
- Server-restart Refresh now adopts a new server generation atomically. A delayed
  old state or geometry bundle cannot replace it, and stale commands cannot become
  valid merely because the new server is also at revision zero.

The reviewed generation implementation and delayed-response tests were separately
read back. Final local research tests: 41 Python and 23 Node passed. The release's
aggregate acceptance provides its own completion timestamp.

This is a bounded research audit, not hardware calibration. Nominal D5 transient
qualification, excluded rail/wire/jaw interfaces and actual browser visual QA
remain outside this result. V1 absolute-only failures are not rewritten by V2.
