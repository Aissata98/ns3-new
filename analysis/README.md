# Offline summary reproduction after moving the archive

This adapter reads the frozen inputs, completed run receipts and retained results. It does not launch simulations, select candidates, call Docker or contact a network. The seven analytical source files in `frozen/` are byte-identical snapshots; `frozen/CODE_SHA256.json` identifies them.

## Explicit relocation, unchanged evidence

Supply one or more non-overlapping absolute mappings as `--relocate ORIGINAL=ACTUAL`. `ORIGINAL` is the root recorded in the experimental evidence, and `ACTUAL` is the location of the same directory tree in your extracted archive. Paths in JSON files are **not rewritten**. The adapter changes filesystem lookup only and preserves original logical names when checking context, job and additive-completion hashes. Missing mapped evidence fails immediately, even if a file still exists in the original workspace. Unmapped reads and symlinks escaping the archive are rejected.

Campaign and selection arguments may use either the original absolute names or their corresponding relocated absolute names. Output must be a new file outside all mapped evidence roots and outside the frozen-code directory, in an existing directory. Use a canonical physical directory name for `ACTUAL` rather than a symlink alias.

For the confirmation phase, replace the uppercase paths and hash below with the archived values:

```sh
python3 reproduce_summary.py \
  --relocate /ORIGINAL/FINALIZATION=/ACTUAL/ARCHIVE/FINALIZATION \
  --campaign /ACTUAL/ARCHIVE/FINALIZATION/confirmation-v6 \
  --binding-sha256 CONFIRMATION_BINDING_SHA256 \
  --selection /ACTUAL/ARCHIVE/FINALIZATION/SELECTION_V6.json \
  --output /ACTUAL/OUTPUT/new-confirmation-summary.json
```

Repeat `--campaign` and `--binding-sha256` in corresponding order to include additional completed phases. For the interrupted transfer, supply **only `transfer-v6-resume`** as that phase. Its pinned original `transfer-v6` directory must also be included in the archive: the additive receipt verifies 30 retained observations plus 30 newly completed observations, with original identities and no duplicate sampling. The original zero-launch disk-guard failure is retained but is not a scientific observation. A partial or absent continuation is rejected.

The archive must contain all evidence read by the frozen validators, including the phase directories, the selection, the scope decision, and the original plan/binding/receipt referenced by a continuation. The wrapper provides no alternate data, substitutes no missing evidence and changes no scientific criterion. A public archive that redacts immutable files needs its own documented provenance chain; silently editing their paths invalidates their original hashes.

## Interpretation and checks

The calculations retain the fixed 10,000-resample bootstrap and the original seed-pairing rule. Running at another time changes `created_at_utc`; the analytical script hash identifies the frozen revision actually used. Original logical paths are intentionally retained as provenance. Numerical equivalence is checked across the complete groups, paired effects and confidence intervals, rather than just rounded table entries. Offline summary completion is not a physical deployment qualification and does not authorize an experimental launch.

Run the software-only adapter checks from this directory:

```sh
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest test_reproduce_summary.py -v
```

The wrapper rejects process creation while the analytical modules run and permits only exclusive creation of the requested summary. It is an adapter for the listed trusted, hash-checked scripts, not a sandbox for arbitrary untrusted Python programs. No active workspace script or original experiment receipt is modified by this package.

The recorded check in `RELOCATION_CHECK_V2.json` used the actual 20-run confirmation archive copied temporarily outside the original workspace. All 440 evidence files retained their hashes, and every statistic, interval and logical provenance value matched the reference summary after excluding its timestamp and analytical-script-version field. The temporary copy was removed after the check. Twelve adapter tests passed, including missing-evidence/no-fallback, path-escape, write-protection and process-launch rejection. The transfer-union path rules are software-tested; this check does not claim that a relocated full transfer archive was also exercised.
