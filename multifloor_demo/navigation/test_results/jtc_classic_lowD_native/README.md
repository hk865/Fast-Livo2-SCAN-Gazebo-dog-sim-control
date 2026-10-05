# Native low-D contract

New excluded test ID; production and all prior results are unchanged. The actual installed JTC fixture CPP is byte-for-byte the reviewed classic19 fixture. The sole control configuration change is D3.360637 →1.0 on all 12 joints. P220.982919/I.2/clamp2.5/FF0, interpolate_from_desired_state=true and closed-loop effort PID remain.

Actual run `actual_20261002_051349`: 19/19 checks pass. 441 position-only/header0/horizon16,666,666ns real DDS callbacks on the 5ms integer producer grid; 550 successful synchronous JTC updates report 4ms. All-joint GetParameters and actual PID gain getters agree. Five original stages include repeated nominal, changed targets, producer-anchored return, idle and restart. Initial command and actual reference effort FF remain zero. Reconstructed PID+FF maximum residual is 4.44e-16; all outputs are finite (maximum command-interface magnitude 3.51682).

PID3037594 exits0, group is clean, domain77 released. All269 production fingerprints match before/after. `prior_comparison.json` also proves identical old/new callback inputs, measured-memory states, reference stream and loaded three DSOs. The YAML-only difference is exactly the twelve D fields.

This bare installed-controller test does not instantiate CM/ResourceManager or enforce the URDF total clamp. Command-interface values are not measured applied forces. The prescribed q/v stream is an interface contract, not a physical robot trajectory; neither the native test nor the no-contact mass model establishes Go2 stability. No new gain was adopted and no physics was run.

Reproduce locally after reserving ROS77: build with the saved CMake file against /opt/ros/jazzy, then run_fixture.py and analyze.py. The fixture always creates a new timestamped evidence directory; no older FAIL/native result is overwritten.
