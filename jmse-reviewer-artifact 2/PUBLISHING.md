# Publish this artifact with the paper

Suggested location in `Aissata98/ns3-new`: `reviewer-artifact/`.
Copy the contents of this directory there without generated reruns or Git
metadata. Keep `results/`, `SHA256SUMS` and the manuscript evidence snapshot.

Before publishing:

1. Run `python3 verify_artifact.py` and validate `SHA256SUMS`.
2. Review the repository diff and include only this release's files. Existing
   unrelated local work must not be included automatically.
3. Publish a commit and, ideally, attach both the artifact ZIP and Overleaf ZIP
   to an immutable tagged GitHub release. Record that commit/tag in the journal
   submission. A suitable proposed tag is `jmse-artifact-2026-09-03`; it has not
   been created by preparing this package.
4. Open the published link without signing in to confirm reviewer access.
5. If an archive service later issues a DOI, add the actual DOI to the paper
   and release description. Never substitute a pending or invented identifier.

The development-repository link in the paper is not an immutable artifact
identifier. Until the release is published, provide this ZIP directly as
supplementary material. The package does not assert that GitHub publication or
DOI registration has already occurred.
