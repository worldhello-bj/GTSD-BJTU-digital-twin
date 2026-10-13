# Restore an explicitly selected asset

Run these commands from `source/cad-source-console` after installing the updated
`prepare_assets.py` beside the existing `ASSET_MANIFEST.json` and `payloads/`.
Python 3.12 and its standard library are sufficient; this loader never downloads
anything or starts Blender, a solver, or the web console.

List the manifest's exact asset paths without reading payloads or writing files:

```sh
python prepare_assets.py --list-assets
```

Restore only the public native Blender assembly, even if unrelated GLB or mapping
payloads have not arrived yet:

```sh
python prepare_assets.py --only integration/source_assembly_native/GTSD_Source_Assembly_Revision.blend
```

The selected asset still requires all of its own manifest chunks. It is restored
to its original manifest path, with every chunk, the packed payload, and the
restored file checked against their recorded lengths and SHA-256 hashes.
No chunk concatenation or manual decompression is needed.

Repeat `--only` for another independent asset; use forward slashes and the complete
`asset.path` from the manifest. Basenames, directories, wildcards, unsafe paths,
and unknown names are rejected. Repeated identical selectors are checked once.

```sh
python prepare_assets.py --only integration/source_assembly_native/GTSD_Source_Assembly_Revision.blend --only full_source_audit/bearing_envelopes/bearing_dimension_envelopes.blend
python prepare_assets.py --check-only --only integration/source_assembly_native/GTSD_Source_Assembly_Revision.blend
```

A selected success reports `scope: "selected"`, the selected names, and the full
manifest count. It does not certify that the whole release is present or that the
console can start. `--list-assets` only lists manifest names, not available files.
`--list-assets` and `--check-only` cannot be combined.

Without `--only`, behavior remains strict for every asset:

```sh
python prepare_assets.py
python prepare_assets.py --check-only
python launch.py --port 8778
python doctor.py
```

`launch.py` and `doctor.py` continue calling `ensure_assets()` without selectors.
They will fail on missing or corrupt full-release payloads. They do not inherit a
previous selection, silently skip assets, or change their physical models.

Existing modified files and target symlinks are never overwritten. A verified
temporary file is published atomically through a same-directory hard link, so a
concurrent writer creating the target also prevents publication. The filesystem
must support hard links; if it does not, restoration fails without overwriting a
target or falling back to a partially visible copy. Temporary files are cleaned
on failure. This is per-file atomic restoration, not a transaction across several
selected assets: if a later payload fails, earlier successful assets can remain.

API: `ensure_assets(root, check_only=False, only=["exact/asset.path"])`.
`only=None` means all assets. An empty selection is rejected.

Run the original six safety tests and the new selection tests:

```sh
python -m unittest discover -s tests -v
```

## Integrating this overlay

Replace only `prepare_assets.py`; add `tests/test_selective_assets.py` and this
usage document. `tests/test_prepare_assets.py` in the overlay is an unchanged
copy of the original six tests. Update the three changed/added entries in
`PUBLIC_MANIFEST.json` to the bytes and SHA-256 values recorded in the overlay's
`PUBLIC_MANIFEST_UPDATES.json`. That JSON is an integration aid, not a replacement
for the complete release manifest. No asset manifest or payload changes are
needed. Keep all unrelated entries, especially the existing small-chunk manifest
and payload identities. Update a repository-level manifest too if it indexes
these source files; do not change frozen baseline manifests or old evidence.
