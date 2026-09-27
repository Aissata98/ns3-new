# Fixture paths, not copied results

Launcher tests create small synthetic fixtures in temporary directories. They make no performance or hardware claim. Real scientific fixtures live only in the separate immutable results archive.

One historically checked real fixture is:

`results/sensors-20260920-closure/finalization/tuning-v6/040-nominal-H-S-r0-a120-q030-s301/remote/`

Its job SHA-256 is `3b357c96ca57eccf9847a9d430ea37c8b194c6c549de9078955229d12011987f`; original archive-index SHA-256 is `b8befee7848efa2440d5e418df1c75ce47108d37ad9e5984cfe854ec60827902`. The earlier offline packaging check deeply verified its 12 indexed members. It is an integrity fixture, not a selected feasible hybrid or a new replay result. Other complete jobs may be checked in the same manner.

No workload, trace, fake result, binary or Docker image is duplicated in this directory. See `../RESULTS_ARCHIVE_CONTRACT.md` for whole-campaign evidence requirements.
