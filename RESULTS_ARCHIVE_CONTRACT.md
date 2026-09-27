# Separate immutable results archive

The source ZIP contains this `artifact/` tree, including model, launchers, analysis, small configuration/provenance and existing license notices. Do not put the result traces in that ZIP. The separate results tar preserves original filenames and bytes. Its final inventory and SHA-256 must be published separately after all authorized phases are closed; this document is a layout contract, not a claim that packaging or every phase is complete.

Extract results to one `results/` tree mirroring the original scope:

```text
results/
  sensors-20260920-closure/
    finalization/
      SELECTION_V6.json
      POST_SELECTION_SCOPE_V6.json
      BUILD_INTEGRATED_V6.json
      ... frozen contracts, bindings, phase/union evidence and summaries ...
      tuning-v6/
      confirmation-v6/
      transfer-v6/              # preserve original stopped receipt
      transfer-v6-resume/       # completed union references the first 30 jobs
      ... later authorized phase directories under their actual names ...
    ... referenced frozen protocol, manifests and ledger snapshots ...
  ... any other explicitly referenced original evidence roots ...
```

For each phase preserve `binding.json`, `plan.json`, `phase-receipt.json`, `manifest.json`, `protocol.json`, scope decision and any completion-union receipt. Preserve every job's small host receipts (`job.json`, `audit.json`, `host-execution.json`, `run-result.json`) and `remote/` with workload, artifact index, execution evidence, energy and all indexed compressed traces. Failed/interrupted attempts remain distinguishable from completed jobs; never merge them into success counts. Include every file referenced by a frozen binding/union/analysis recipe, retaining hash-checked identity. Original absolute references are resolved by explicit mappings with the offline analysis wrapper, not by rewriting JSON.

Avoid duplicate scientific trace storage: retain the canonical `remote/*.gz` members; do not additionally export decompressed CSV copies or raw container copies. If multiple original paths genuinely contain identical bytes and both paths are required, a tar hardlink member may preserve each path while storing the content once. This is an archive-writing option only; no workspace hardlinks or original file deletions are required. Validate extracted relative paths, reject traversal/symlink escape, and check hashes before analysis. Do not collapse interrupted and resumed job records solely because their workload is identical.

The single-job fixture is any completed `.../<job-id>/remote/`, checked with `replay.py verify-archive --deep`. Whole-campaign analysis additionally requires the surrounding phase/union evidence. A bundle with only selected good runs cannot reproduce the selection or full cohort denominator. Keep H-A tuning failures and H-S service-infeasible diagnostics visible. No new run is required to verify archive integrity.
