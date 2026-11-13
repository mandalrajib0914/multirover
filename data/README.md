# Recorded ROS dataset

[Download the ROS bags](https://github.com/mandalrajib0914/multirover/releases/download/v1.0.0/multirover-rosbags-v1.0.0.tar)
and [archive checksum](https://github.com/mandalrajib0914/multirover/releases/download/v1.0.0/SHA256SUMS).

The archive is about 1.4 GB and contains 175 compressed MCAP recordings plus
175 `metadata.yaml` files. The recordings are already compressed with zstd, so
the enclosing tar archive is uncompressed.

- 110 primary ArUco campaign trials: `data/s8/`.
- 40 complementary YOLO trials: `data/s8_yolo/`.
- 25 superseded YOLO trials: `data/excluded/s8_yolo_doll_in_wall/`.
  These are preserved for transparency and must not be included in the reported
  campaign statistics. See [exclusion reasons](excluded/README.md).

## Download and verify

Run from the repository root. The original archive uses a `results/` prefix.
The extraction command below translates that prefix to `data/`, matching this
repository layout without changing the recordings or their checksums.

```bash
curl -fL --retry 3 -C - -O https://github.com/mandalrajib0914/multirover/releases/download/v1.0.0/multirover-rosbags-v1.0.0.tar
curl -fL -O https://github.com/mandalrajib0914/multirover/releases/download/v1.0.0/SHA256SUMS
sha256sum -c SHA256SUMS
tar -xf multirover-rosbags-v1.0.0.tar --transform='s,^results/,data/,'
sha256sum -c data/rosbags.sha256
```

Allow approximately 3 GB of free space for both the archive and extracted bags,
plus space for any decompressed files produced during analysis.

## Inspect a recording

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag info data/s8/n1_greedy_t00_A_V1/bag
```

Use rosbag2 with the MCAP storage plugin and zstd compression support. Each bag's
metadata lists recorded topics, message types, timing, and compression details.
These are simulation recordings; their time stamps use the simulation clock.
Recorded topics are a subset of the full simulation, not a snapshot of every
sensor or process.

The checksum manifest identifies the exact bytes distributed with `v1.0.0`.
Original measurements and bag contents have not been edited.

Some archived summaries contain original absolute `saved_map` paths as provenance.
Analysis uses the map beside each summary rather than those historical paths.
