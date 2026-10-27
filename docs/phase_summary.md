# Navigation phase and daily project

The current route is F1→F3 with actual sensor SLAM, map-assisted localization, persistent SCAN and the existing CPU Student45/GRU policy. It is a simulation result, not a real-robot or repeated-run reliability claim.

## Daily operation and explicit testing

`start.py` starts normal nodes from the current project configuration and assets. The experiment launcher/owner/verifier, source witnesses, frozen version lists, source/model/binary SHA approval, archive completion barriers and final PASS aggregation are outside the daily execution chain. Eighty-five retired experiment/helper files were moved out of this formal project. Current request IDs, coordinate generations, packet sequences, finite-value checks and native IPC CRC remain real protocol/control checks; an old run or archived evidence file cannot grant permission to navigate.

The functional causal IMU buffer, current-source freshness, real obstacle/tilt protection, command bounds and native deadlines remain. A relocation edit accidentally disabled the IMU buffer; an actual run exposed repeated pair-missing zero holds, and the original buffer was restored. No pairing/freshness threshold was enlarged.

`tools/validation/run_navigation_check.py` explicitly runs the normal entry and a separate observer. The observer commands only `moving_obstacle`, reads its actual Gazebo model observations, and reads SLAM/IMU/control. Robot truth is not navigation or arrival authority. Every observed stop must have zero hold commands, measured stopping, at least ten actual clear evaluations over one second and measured recovery. The parking test uses the first actual zero-input qualification time, causal 10Hz positions/yaws, five immutable seconds, 5cm/0.1rad drift and 300ms source coverage. High-frequency velocity is not an extra parking veto under the existing pose-only rule.

## Recorded outcomes

An earlier experiment-entry run at 12:37–12:44 UTC completed the route, one dynamic stop/recovery and fixed parking. It is separate from validation of this later daily entry.

Daily assembly runs 15–18 exposed and retained failures: a terminal-heading geometry contract, a removed route SHA still being consumed, an archive transaction interface, and an experiment protocol ledger. Those mechanisms were removed rather than repinned. The real IMU pairing regression was also corrected.

Daily run 19 reached F3 and completed one dynamic stop/recovery. Its original observer reported FAIL because that observer mistakenly added a velocity veto from a different parking mode. The original result is preserved. The observer was corrected against the existing pose-only rule; it now uses the first actual qualification time rather than a later observer-selected start. Final-layout run 20 failed at import because real parking pairing still imported a retired acceptance helper. That import was removed; all twelve normal node imports then passed in the actual ROS/DDS environment.

Final-layout run 21 completed at 14:52:20–14:59:36 UTC with entry exit code 0. Actual SLAM route progress was 34.945m. One measured obstacle stop/recovery passed; clear release used eleven actual evaluations over 1.000s and measured recovery was 1.038m. This run predates subsequent Python changes and does not establish the final submitted source state. A fresh complete final-source run is required. The original first qualification was 424.715sim s; its immutable five-second window used 51 causal samples. Rechecking against the original qualification anchor gave maximum XY drift 0.018480m and yaw drift 0.064646rad, with maximum source age 35ms. Earlier failures are not relabelled PASS.

The qualification position met the original goal bounds (XY error 0.017797m, Z error 0.032189m, yaw error 0.024357rad). During the hold the SLAM goal-height error reached 0.115630m; Z drift relative to its qualification anchor reached 0.085899m. The existing hold rule checks XY/yaw; this result is not a claim of validated three-dimensional height hold. This is an observed corrected-SLAM Z variation, not an established localization fault: it has not been aligned against ground truth for the same physical reference point. Body motion, coordinate corrections and estimation effects have not yet been separated. The run/journals are temporarily protected for that comparison.

A subsequent short normal-entry loading observation saw eighteen owned processes and the project-local patched DDS library, with no project path outside the formal directory. Shutdown contains a retained native FAST-LIVO2 error when its ROS context is already invalid at stop. Process absence and lock release are measured separately; an outer launch exit of zero is not claimed to make every native child exit zero. Own Python nodes were subsequently adjusted to handle expected SIGINT without publishing into an already stopped ROS context. No navigation math or acceptance threshold changed in that shutdown-only adjustment.

## Retention and unresolved work

Closed bugs retain concise root cause, fix description, normal Git changes and measured outcomes. Proposed deletion of original diagnostics means those reports cannot promise later raw replay. Two earlier CPU resource aborts remain unresolved and are listed separately from closed bugs. Historical data and old checkouts have not been permanently deleted by this implementation phase. Cleanup requires a separately confirmed exact path manifest.

The native SLAM/SCAN/actuator binaries and DDS fix were relocated without rebuilding algorithms. Required project assets/installations remain local. A clean build from the public source snapshot has not been validated; shared ROS/Gazebo/PyTorch installations are outside project cleanup scope.

## Repository identity and final-source rerun

The current candidate repository has local `main` at `5237b886`; its working development branch was `f9e5086` before this documentation/test update. The separate old publication checkout remains at `990f1c98`; that is not this repository’s current main. A read-only check saw remote main at `bc9d393`. Publication is paused pending specific destination/content approval and asset rights; no private development ancestry may be pushed. Any future integration must preserve the actual latest remote changes without force.

The final-source complete rerun is performed after committing the prepared tree and saving its full tracked-source snapshot outside runtime. That snapshot is only for independent review, never a startup input. Its result is reported in the local completion receipt.

## Final submitted source: complete run 26

The frozen source commit `baa5d06` and its full tracked-source archive were saved before the final complete run. Run 26 used the normal entry from 15:46:55 to 15:54:28 UTC and exited 0; neither application nor test source was edited while it ran. It advanced 34.88259m along the route and passed one measured obstacle stop/clear/recovery. Clear release used twelve actual observations over 1.000s; measured recovery was 0.93804m.

The immutable first qualification was 441.645sim s. The original five-second window contained 51 causal 10Hz pose samples, with maximum XY/yaw drift 0.030527m/0.012019rad and maximum source age 100ms. Forty-four actual statuses received within the window were zero holds; the maximum boundary/internal gap was 165ms. No pre-T0 status was carried into the window. Fifteen focused tests passed. Eighteen owned processes were observed loading project-local dependencies and the patched DDS, without old project or temporary-worktree paths.

Run 25's original observer FAIL remains unchanged. Its false rejection came from carrying a pre-T0 deceleration status into the hold window. The pre-existing rule checks only statuses actually received inside the fixed interval and allows up to 300ms boundary/internal gaps. The correction is confined to explicit development tests; navigation code, T0 and thresholds did not change. Negative tests reject late/missing statuses, nonzero commands and changed coordinate sources.

Run 26's Z drift from its qualification anchor was 0.027125m and maximum target-height error was 0.014661m. This one result does not explain the larger run-21 Z variation and does not establish repeated-run reliability. The run-21 numbers are not confirmed localization drift; the earlier CPU aborts separately remain unexplained. Required runtime installations and shared system dependencies remain as described in `restoration.md`; the 349MB retention set is not an independent package. Publication and permanent cleanup remain unapproved. A report-only commit may follow this run; Git comparison must show all executable/configuration/asset/test paths unchanged from the tested source.

### Z field provenance

The reported Z comes from actual `/demo/slam/body_odom` position, frame `camera_init`, child `demo_slam_body`. The publisher applies `T_navigation_world @ C_world_odom` to the raw SLAM-derived body-origin transform. The two numbers compare this corrected body-position Z with the first qualification anchor or the configured F3 goal; neither is automatically a ground-truth localization error.

Saved global-localization profile parameters match between the earlier experiment run 14 and daily runs 21/26: 1s matching interval, 3s match age, 300ms input freshness, 0.04m/0.025rad correction trigger, 0.04m/s/0.02rad/s smoothing and the same quality settings. The earlier and current implementations both initiate new correction transactions only while navigation state is `running`; matching itself continues after arrival. This is not a newly added post-arrival switch.

Input assembly did change during decoupling: the older source join supplied selected raw/adapted LiDAR points plus mounting translation; the daily snapshot forms body points by rigidly transforming the actual post-LIO registered cloud with its same-stamp body pose. Matcher/profile equality is therefore not a proof that its input arrays are identical. No new algorithm repair or failure diagnosis is claimed from the Z observation.

### Clarified Z comparison boundary

Read-only review found that run 21 kept exactly the same correction matrix throughout its five-second window (generation 138, planner epoch 9, transaction count 8); its last correction completed before the window. The configured target Z=2.402696328m was the initial body estimate plus the 2.4m floor increment, not ground truth. The body-origin Z first fell and then rose; first-to-last change was +0.082297m and peak-to-peak range 0.169341m.

Comparable earlier successful records also show nonzero Z changes: the earlier bcb2 record had maximum anchor/target offsets 0.087239m/0.050729m; run 14 had 0.009253m/0.134060m; run 21 had 0.085899m/0.115630m. These values do not establish a newly introduced Z localization defect. Run 21 has no same-window ground-truth trajectory aligned to the same physical body reference point; its later snapshot is outside the window. No Z absolute trajectory error was computed. The earlier wording treating this as a new unresolved localization drift is withdrawn, without claiming that the estimate is exact.
