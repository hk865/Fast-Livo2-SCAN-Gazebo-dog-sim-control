# Excluded ALIGN translation candidate

This candidate is frozen for a separately authorized component trial. It does
not change the production workspace, original `f592194d`/`101912f6` candidate,
PID, gait, map, SCAN algorithm, region or safety thresholds.

The only additional motion is body-x anchor feedback during ALIGN. The raw
position and raw rotation come from the same freshly accepted odometry Header.
An immutable request/goal/trajectory/reference identity binds the anchor.
Duplicates cannot update the estimator, quaternion, age or count.

`vx = clip(0.8 * deadband(anchor_error_x, .01) - 0.4 * deadband(measured_vx, .015), -.10, .10)`.
There is no integral, GT input or predicted Go2 response. Measured velocity is
a least-squares fit over 0.4 s of raw positions, minimum span 0.35 s and four
samples. Original raw gap/age limits remain 0.2/0.25 s; position RMS must be
≤0.02 m. The fit has roughly 0.2 s of effective lag. The helper slew is at most
0.2 m/s²; the original emitted-command limiter further limits translation to
0.15 m/s². Requested helper values are recorded separately from emitted values.

When vx is nonzero, emitted yaw remains at the existing mixed-motion cap 0.08
rad/s. A preceding pure-turn rate of 0.12 must first decay to that cap while
vx stays zero. Lateral velocity is always zero. Pure-turn drift protection now
uses ALIGN stage, so compensation cannot escape its original 0.15 m / 0.2 s /
three fresh observations condition. Preturn and settle retain exact zero and
their original one-second durations. STOP, stale sensors, protection and
pending handoff never produce compensation.

Additional translation is checked against the real, self-filtered cloud in
the signed current body-x corridor, with the original 0.95 m / 0.42 m / three
points rules. The exact cloud stamp must match the accepted cloud stamp;
frame, count, finite nonempty geometry, native/wall age and original ≤0.15 s
filter-body association are verified. Negative compensation therefore cannot
borrow the forward SCAN corridor as evidence of rear clearance. A failed
candidate corridor or estimator continuity invokes exact zero and the original
post-stop idle/one-second-zero/new checked-SCAN handoff. Original route obstacle
protection remains in place.

The three frozen sources are `nav_align_controller.py`, `turn_drift.py` and
`align_translation.py`. The entry contains the entire prior drift class plus a
subclass; it does not import another excluded controller. Explicit environment
`DEMO_TEST_ROOT=<multifloor_demo>` and
`DEMO_TEST_ALIGN_TRANSLATION_ENABLED=1` are required, along with the original
explicit ROS scenario path. Mode 0 calls the original execution methods; the
prior drift class AST is identical. Enabled compensation is schema2-only.

`ready_receipt.json` records hashes. Twenty-two pure/actual-method checks and
twenty-two actual ROS77 DDS checks pass with those same source bytes. The DDS
fixture runs actual NAV subscribers and emits actual Twist messages, with
synthetic clock, poses, cloud, IMU, metadata and Bridge/Adapter protocol inputs.
Its controlled ACK is not a real hardware ACK, nor a physical/SLAM/SCAN PASS.
Owned nodes and the test process are fully cleaned up. The independent review
also checks actual emitted mixed velocity/yaw and original handoff boundaries.

The old-turn replay evaluates only the helper on immutable historical
measurements. It does not apply candidate commands to a plant or predict
success, avoided tilt or future arrival. Its unavailable wall timestamps and
retained pose gap are explicitly disclosed. Physical verification remains a
separate decision owned by the parent.
