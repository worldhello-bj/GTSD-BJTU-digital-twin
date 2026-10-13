# Complete 476-state reviewed source delivery

Run from `source/cad-source-console`:

```sh
python prepare_assets.py --source-review-only
python prepare_assets.py --source-review-only --check-only
python prepare_assets.py --source-review-only --list-assets
```

These commands restore, read-only check, or list all 432 newly bundled source
JSON resources. The original 44 source resources retain their existing paths
and identities. Normal `prepare_assets.py`, `launch.py` and `doctor.py` include
source restoration after the original strict legacy asset step. `--only` and
plain `--list-assets` continue to refer to the original 57 runtime assets.
No manual concatenation, downloads, Blender, conversion or solver is involved.

## Two fixed, independently verifiable containers

- First: 424 original reviewed additions, 42,236,149 raw bytes, 6,662,492 packed
  bytes in 23 pieces. All piece paths/bytes/hashes and the first source manifest
  are preserved exactly.
- Additional: 8 reviewed comparisons, 1,762,187 raw bytes, one 252,660-byte piece.
- Combined: 432 JSON files, 43,998,336 raw bytes, 6,915,152 packed bytes, 24 pieces.
  Each piece is at most 300,000 bytes.

Both use canonical path-ordered raw-byte concatenation in an XZ preset-6 stream
with SHA-256 checksum. Each explicit index contains exact file offsets, lengths
and SHA-256 identities, plus per-piece hashes. `SOURCE_REVIEW_MANIFEST.json` and
`SOURCE_REVIEW_EXTRA_MANIFEST.json` are individually SHA-pinned in the standard
library loader. There are no embedded archive filenames or arbitrary extraction
paths. The second namespace is `payloads/source-review-extra-v1/`.

Both indices and every output file are validated before the first publication.
The loader rejects duplicate paths within/across containers, traversal, catalog
replacement, boundary gaps/overlaps, chunk reorder, tampering, excess memory or
size, malformed UTF-8/JSON and trailing/concatenated XZ streams. It checks all
existing files first and never overwrites changed files or follows symlinks.
Same-directory temporary files are fsynced and linked atomically with no replace.
No-follow directory-relative operations require Linux/macOS or WSL. The existing
57-asset decoder and ASSET_MANIFEST are unchanged.

An interruption can leave a subset of complete verified files, which a retry
checks and preserves before filling missing paths. Caught interruptions clean
staging files. A hard-killed process may leave an ignored `.gtsd-source-review-*`
file; it is not treated as completed output and does not block a retry. Atomicity
is per file, not a transaction over the full set. Hashes assume the published
loader/index identities are trusted; they are not a cryptographic signature.

## Source meaning and accounting

All 476 review states are now included. These are not 476 independent physical
parts. The additional eight are selected CAD-entity source excerpts/derived
coordinates and comparisons with existing geometry, including uncertainty and
zero-thickness source-reference views. Every original byte and source SHA is
retained. They add no physics, collision, mass or whole-vehicle placement, and do
not turn incomplete semantic evidence into a confirmed manufactured component.
Their source/evidence/preview filenames are provenance identifiers, not a promise
that those underlying evidence files are publicly downloadable.

The original 19 catalog metadata fields are retained from the 476-state source
catalog. Accounting covers 3,160 records across all 20 categories, with 107 roots,
594 associated definitions, 312 regions and zero unread association pointers.
The overlapping 3,050 conditional-upgrade/missing-field handles are not a count
of unreviewed or missing physical parts. These denominators must not be added.

## Checkout and test behavior

The complete-checkout regression classifies both pinned source containers as
transport without requiring generated JSON to exist first. Missing chunks remain
an explicit blocker, and present chunks/restored files get exact identity checks.
Missing manifest/loader/source metadata is never hidden as a binary transport
skip. Without source manifests the original legacy classification is unchanged.

```sh
python -B -m unittest discover -s tests -v
node --test integration/tests/test_source_library_page.js
```

Test totals come from the executed suites. `run_acceptance.py` preserves the same
steps and obtains its Node total from the actual TAP report, rather than a frozen
number. Source-only success is not full runtime or remote acceptance. The report
`verification/remote_fresh_4401c7b.json` is historical evidence for commit 4401c7b;
it does not certify the later 476-state source increment. That increment still
needs its own fresh-checkout verification after publication.
