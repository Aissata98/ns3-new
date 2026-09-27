# Reviewed V6: sources, offline analysis and single-job replay

This source-only package rebuilds the **reviewed V6 model**, not the older paper v2 simulator. It does not bundle the large campaign traces and does not publish anything. Supply the `remote/` directory of a completed archived job, containing `job.json`, `workload.csv`, `artifacts.json` and the recorded outputs. Preserve original archives; all replay outputs go to a new directory.

## Current verification status

Source hashes and offline launcher tests can be checked without Docker or a simulation. The Dockerfile and reconstructed network binary have **not yet been built or exercised by this packaging step**. The historical V6 build and campaign receipts document the original runs, not this new Dockerfile. A source-compatible rebuild may have a different binary hash or floating-point behavior on another architecture. It must not be described as the same binary or an independently confirmed scientific result.

Requirements: Python 3.10+ for offline tools; Linux Docker engine or Docker Desktop on macOS for build/replay. The host launcher uses POSIX user IDs. A full image build downloads dependencies and needs substantially more space than this source package; arrange its resource budget separately. No network download occurs during offline verification, planning or replay (`--pull=never`, container `--network=none`).

From this directory:

```sh
python3 -B replay.py verify-model
python3 -B -m unittest -v test_replay.py
python3 -B replay.py verify-archive --archive /absolute/path/to/job/remote --deep
python3 -B replay.py build --dry-run
python3 -B replay.py plan --archive /absolute/path/to/job/remote --output /absolute/path/to/new-replay
```

`verify-archive` checks local hashes, the canonical job identity, matching V6 sources, input bytes, execution linkage and every indexed output. `--deep` additionally streams decompression to verify original content hashes and sizes. This is an integrity check against the supplied manifest, not cryptographic authentication of its publisher, a service-performance test or a hardware certificate. Check the externally supplied package/archive checksum as well.

Only when ready to spend resources, explicitly build and replay one job:

```sh
python3 -B replay.py build --image edc-reviewed-v6:local
python3 -B replay.py replay --archive /absolute/path/to/job/remote --output /absolute/path/to/new-replay --image edc-reviewed-v6:local
```

The build fixes ns-3.40 and Aqua-Sim commit `0fbc166b8b12cf711f14379cb159393d8061e3a6`, verifies the packaged source hashes and builds `edc-reviewed` plus the PHY test target. It generates `BUILD_REPRODUCED.json` with the new binary hash, compiler, architecture and dependency versions. Dependency downloads/apt versions are not fully content-pinned; bit-identical images are not claimed. The build does not execute a network simulation.

The replay loads **all scientific arguments directly from the archived job**. Only the workload location and seven output locations change; no timeout, workload, topology, policy, deadline, speed, buffer or energy parameter is tuned. Output directories must be new and outside the immutable archive. Execution uses two CPUs, 4 GiB (including swap), a read-only image, a finite wall deadline from the job and a 1.5-GB disk floor. This user-approved execution-resource limit does not change the modeled node/vehicle storage; original campaign receipts retain their historical limits. It has no automatic restart and no macOS-specific thermal dependency. Users remain responsible for host temperature and free space; these resource limits are not a temperature guarantee.

`REPLAY_INTENT.json` and `REPLAY_RECEIPT.json` retain the historical job/binary identities separately from the rebuilt binary identity, actual relocated command, input hash, resource limits, return status and new output hashes. They never replace historical campaign receipts and do not automatically extend the original campaign ledger. An execution returning zero is not a scientific-validity or acceptable-performance verdict. Use the accompanying campaign audit/statistical tools for cohort-level analysis; original HIGH/MED units are records, LOW units are complete acquisition windows, and missing complete windows remain in the mature denominator.

Component checks may be run explicitly after a successful image build, without launching a scientific workload:

```sh
docker run --rm --pull=never --network=none --cpus=2 --memory=4g --memory-swap=4g --entrypoint /opt/ns-allinone-3.40/ns-3.40/build/scratch/ns3.40-edc-reviewed-optimized edc-reviewed-v6:local --reviewedSelfTest=1
docker run --rm --pull=never --network=none --cpus=2 --memory=4g --memory-swap=4g --entrypoint /opt/ns-allinone-3.40/ns-3.40/build/scratch/ns3.40-edc-reviewed-optimized edc-reviewed-v6:local --reviewedOpticalSelfTest=1
docker run --rm --pull=never --network=none --cpus=2 --memory=4g --memory-swap=4g --entrypoint /opt/ns-allinone-3.40/ns-3.40/build/scratch/ns3.40-edc-reviewed-optimized edc-reviewed-v6:local --reviewedIngressSelfTest=1
```

## Optional bounded local rebuild

The already available `edc-ns3:3.40` image can avoid downloading/rebuilding dependencies. `LOCAL_BASE_IDENTITY.json` records its observed arm64/Linux image ID and selected dependency-source fingerprints. This option is local-only: it refuses a missing or different base, instead of downloading one. **The cached recipe has not yet been built.** Its dependency checks must actually pass inside the new container before it can be described as verified.

```sh
python3 -B -m unittest -v test_local_build.py
python3 -B local_build.py plan
python3 -B local_build.py inspect
# Explicit build only after other campaign runs finish and space is available:
python3 -B local_build.py build --image edc-reviewed-v6:cached --receipt-dir /absolute/path/to/new-build-receipts
```

`plan` starts no processes. `inspect` only reads the existing image identity. `build` creates a **new** container from the exact checked image ID with two CPUs, 4 GiB total memory+swap, no network and a 900-second compilation deadline. It checks the cgroup, ns-3 version/fingerprints and clean tracked/untracked Aqua-Sim source, copies the exact V6 sources, then compiles as a real non-root user with two compiler tasks. It commits only this new container to a new local image tag with the replay entrypoint. It never alters the campaign container. Build logs and new binary/dependency receipts remain distinct from historical evidence. The new container is stopped and retained on both success and failure for inspection; it is not silently deleted. A failed build needs diagnosis, not automatic retry. Confirm available disk space before launching; bounded RAM is not a thermal or disk-space guarantee.

`Dockerfile.local` documents a conventional cached-image alternative. Direct `docker build` does not impose the above runtime cgroup cap or enforce the mutable base tag; do not use that alternative under the current four-GiB build budget. The fresh `Dockerfile` also remains untested by this packaging step.

## Offline analysis and separate results

`config/` contains exact small copies of the frozen protocol, hardware contracts, experiment manifest, selected candidates and post-selection scope. `provenance/` retains the original V6 build/kernel checks and selection receipt. These historical records can contain original absolute paths as provenance, not instructions to read another machine.

`analysis/reproduce_summary.py` and its frozen dependencies handle explicit archive relocation without rewriting historical evidence. Consult its `--help` and the analysis-specific checks supplied with the package. All result directories belong in the **separate results archive** described by [RESULTS_ARCHIVE_CONTRACT.md](RESULTS_ARCHIVE_CONTRACT.md). The source ZIP includes no scientific traces, job workloads, manuscript PDFs or reviewer correspondence. [fixtures/README.md](fixtures/README.md) identifies the real-data fixture convention; synthetic launcher tests are not scientific results.

## Source and redistribution scope

`MODEL_MANIFEST.json` pins the copied V6 study source and headers; its hashes derive from `BUILD_INTEGRATED_V6.json`. The model is unchanged. The separate artifact/storage audit describes persistent archive, staging RAM, AUV data reservations and their limitations. These are finite study allocations, not measured total embedded-device memory.

This local preparation is **not yet a public release**. The existing authors' MIT notice is retained verbatim in [LICENSE](LICENSE); [NOTICE.md](NOTICE.md) preserves the separate dependency-license scope. That notice does not relicense ns-3 or Aqua-Sim. ns-3 uses [GPLv2-compatible licensing](https://www.nsnam.org/docs/contributing/html/general.html); retain upstream license and source-distribution obligations when distributing built images or binaries. No credentials or Git history are included. The large result archive and its external checksum are supplied separately, not duplicated by this launcher.
