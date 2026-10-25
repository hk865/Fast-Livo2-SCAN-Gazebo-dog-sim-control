# Published SCAN private rebuild

Compile the 28 exported `src/plan_manage` inputs into `.local_build/` and `.local_install/`. The package-local `build`/`install` links target only those same new private directories; manifest keys use canonical resolved paths. No old protocol_sender/SCAN executable is copied or used as fallback.

This is an explicit mixed source/frozen-dependency rebuild, not a fully source-rebuilt distribution. Four planning static libraries (`bspline_opt`, `path_searching`, `plan_env`, `traj_utils`) plus eight `scan_planner_msgs` generated libraries are copied into `.local_dependencies/` only after exact SHA comparison to the original formal22 build receipt. Their accompanying installed headers/CMake/Python text are copied and recorded; the top SCAN executable is newly compiled from this release. Do not silently replace these dependencies with an arbitrary upstream checkout. Shared ROS Jazzy/system C++ libraries remain declared external dependencies.

The same installed message SDK also supplies these three newly explicit Python extensions:

- `scan_planner_msgs_s__rosidl_typesupport_c.so`: `773a00fc3c228df53f119230c0cdf35253d1f8ca3020c0cd3674a6c56fc0e44a`
- `scan_planner_msgs_s__rosidl_typesupport_fastrtps_c.so`: `c3a808331a5baba7baeb0869a028e7b119504c64478771bc0cce794fcb8b6776`
- `scan_planner_msgs_s__rosidl_typesupport_introspection_c.so`: `3f49a0630da84b04b0715d742765c48ec1da829e269a1b0c9695d273de6c735d`

Their hashes/paths were not found in the original22 source/scope/runtime witnesses. They are honestly recorded as newly explicit dependencies from that same installed message SDK, not falsely claimed as originally bound. Native SCAN C++ compilation does not by itself prove PID Python imports or runtime delivery; runtime setup must include this new private message SDK prefix.

`private_build_receipts/SCAN_BUILD_RECEIPT.json` records exact sources, original flags with path relocations, new ELF/config, compiler inputs, private dependencies, `readelf` and `ldd`. Build is single-job nice10 and compile-only. Generated artifacts/receipts are Git ignored. Binding/actual process loading and full simulation require the parent owner's separate fresh admission; no historical PASS is inherited.
