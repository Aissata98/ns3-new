# Execution environment

| Component | Specified version |
|---|---|
| ns-3 | 3.40, official `ns-allinone-3.40.tar.bz2` release |
| AquaSim-NG fork | `https://github.com/rmartin5/aqua-sim-ng.git` |
| AquaSim-NG commit | `0fbc166b8b12cf711f14379cb159393d8061e3a6` |
| Ubuntu base | `sha256:4fbb8e6a8395de5a7550b33509421a2bafbc0aab6c06ba2cef9ebffbc7092d90` |
| ns-3 build profile | optimized; examples enabled; Python bindings disabled |
| Analysis | Python 3.9+, standard library only |
| Host launcher | Bash and Docker Linux containers |

The complete build recipe is `docker/Dockerfile`. Upstream compiler and package
versions are resolved by Ubuntu repositories when the image is built. The
ns-3 archive is selected by release version; the recipe does not pin its
checksum. Consequently the recipe fixes core simulator versions but is not a
byte-reproducible OCI image specification. Preserve the resulting image ID and
the output of `docker version` with a new campaign when stronger provenance is
needed. Cross-platform floating-point differences may require numerical
tolerances rather than byte comparison of regenerated CSV files.

The artifact manifest hashes the submitted simulator, drivers, reference
outputs and manuscript files. Historical experiment logs do not contain an
embedded source hash, so the release manifest identifies this source snapshot;
it does not retroactively prove the build provenance of each historical run.

No external Python library is required. Docker downloads its dependencies
during image creation. The image itself is not embedded in the ZIP.
