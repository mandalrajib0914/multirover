# Excluded runs

Kept for transparency, not used in any table or figure.

- `20260926_212133_g2a` (2 robots, greedy, "42.0 s"): coverage was already 72.7 % at t = 3 s,
  while every valid run starts at 7-12 % (what the robots see from the spawn). A merged `/map`
  from an earlier test run was still being published by a leftover process, so the coordinator
  started on an almost-finished map. It ran before the stale-map guard (maps stamped in the
  future are ignored) and before the per-run process cleanup were added.
  It also predates the "no two robots on one frontier" change.

- `s8_prefix/n2_nearest_t05_B_V3` ("stalled", 88.4 %): hit a coordinator deadlock (frontiers
  left but none reachable, so no goal and no "done"), fixed from trial 61 on
  (`unreachable_timeout`). Re-run with the fix; the re-run is in data/s8/.

- `s8_yolo_doll_in_wall/` (the first 25 YOLO trials, 2026-09-27): the doll sat at the ArUco
  box's centre points, but it is 0.41 m long, so at V2/V3 its back half was 8 cm inside the
  wall. The camera saw a shifted, partly hidden doll while the "truth" was a buried centre
  (errors: V1 max 14 cm, V2 28 cm, V3 18 cm). From then on run_campaign.py moves the doll 0.105 m
  forward (its back where the box's back was), and the YOLO campaign was re-run from scratch.
