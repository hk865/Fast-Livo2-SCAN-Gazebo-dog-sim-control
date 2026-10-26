# Reproduce and bind the private DDS runtime

The demo uses a private Fast DDS 2.14.6 with the reviewed two-sided SHM heartbeat
backport. It does not replace `/opt/ros` or a system library. The creation witness
is preloaded into owned roles; the existing bounded owner/ACK/send/SHM observer is
additionally preloaded into native SLAM only. Message/QoS hooks, polling cadence,
retention, binary schemas and the original 300ms source-age gate are unchanged.

The committed `candidate/PRIVATE_DDS_CONTRACT.json` is a locator. Actual absolute
paths, file SHA256 values and GNU BuildIDs are generated into
`.local/private_dds/PRIVATE_DDS_CONTRACT.json`. Missing bindings fail before
launch; there is no silent fallback to system DDS. An explicitly selected
`PRIVATE_DDS_RUNTIME_CONTRACT` may point to another locally generated contract;
the choice is fixed for that process. Never commit that generated contract,
descriptors, compiled libraries, debug companions or local build receipts.

## Requirements

The supported observer ABI is Linux x86_64, little endian, Fast DDS 2.14.6. The
validated environment is Ubuntu 24.04/ROS 2 Jazzy with the existing C++ toolchain,
Fast-CDR, foonathan-memory, Boost, OpenSSL, TinyXML2 and SQLite development files.
The tools also require Python 3.12 or later, CMake, a C++17 compiler, GNU patch,
binutils (`readelf`, `nm`, `objdump`, `addr2line`, `objcopy`) and GDB with Python.
Tests use a local Googletest source tree (default `/usr/src/googletest`), or an
explicit existing GTest CMake directory. The tools install no packages and fetch
no source or dependencies. Supply those dependencies before building.

## Build from fixed local sources

Obtain the two archives whose URLs, commits and SHA256 values are recorded in
`third_party/fastdds/SOURCES.json`, or copy previously verified archives locally.
Run from the repository root; change the archive paths to their actual locations.
The first command validates source hashes and prints the full plan without
building or extracting anything:

```bash
python3 -B tools/private_dds/build_private_dds.py \
  --fastdds-archive /path/to/Fast-DDS-2.14.6-source.tar.gz \
  --asio-archive /path/to/Asio-pinned-source.tar.gz
```

Add `--execute` to build. The default fresh locations are
`.local_build/private_dds` and `.local/private_dds/install`. The tool rejects
existing build/install directories and system/ROS prefixes. It extracts the fixed
archives, applies the production and two test patches, builds the local GTest
dependency if needed, and configures a new CMake cache with the validated flags:

```text
RelWithDebInfo; THIRDPARTY=OFF; THIRDPARTY_UPDATE=OFF;
SECURITY=ON; FASTDDS_STATISTICS=ON; SQLITE3_SUPPORT=ON; LOG_NO_INFO=ON;
SHM_TRANSPORT_DEFAULT=ON; COMPILE_EXAMPLES=OFF; COMPILE_TOOLS=ON;
EPROSIMA_BUILD_TESTS=ON; SYSTEM_TESTS=OFF; PERFORMANCE_TESTS=OFF
```

It builds `fastrtps`, `SharedMemTests` and `fast-discovery-server`, then installs
only into the selected private prefix. It separates debug data with GNU objcopy
and verifies that every allocated ELF section remains identical. The small
runtime library and its `.debug` companion must stay together for later
DWARF/observer regeneration. `BUILD_PLAN.json`, step logs and
`BUILD_IDENTITY.json` remain in the private work directory. These are local build
identities, not portable historical pass receipts. The same build flags and
source pins are reproducible; compiler/dependency differences may produce a
different ELF and must generate a different exact identity.

The build does not execute tests. To run only the bounded SHM regression cases
after reviewing the plan:

```bash
.local_build/private_dds/build/test/unittest/transport/SharedMemTests \
  --gtest_filter='SHMTransportTests.reviewed_heartbeat_*'
```

`--baseline` builds the unpatched production library with the same regression
fixtures. Give it distinct `--work-dir` and `--prefix` locations. Latency checks
are expected to fail for that negative control; never deploy the baseline as the
patched runtime.

## Build the observers against that exact ELF

The observer build is self-contained and does not depend on old experiment
directories or an old build-command receipt:

```bash
python3 -B tools/private_dds/build_observers.py \
  --library .local/private_dds/install/lib/libfastrtps.so.2.14.6 \
  --fastdds-source .local_build/private_dds/Fast-DDS-f052f239538ebc0df56835fdb57f7628628010bc \
  --output-dir .local/private_dds/observers
```

Without `--execute`, this prints the compile plan only. With `--execute`, it uses
readelf/nm/objdump/addr2line and file-only GDB (no inferior) to inspect the exact
ELF, its GNU BuildID, exported method offsets, native/private-member layouts and
the source-proven `healthy_check` nanosleep return PC. It rejects an unknown
layout, ambiguous/missing call site or missing debug data. Only then does it
generate `probe_identity.hpp` and compile the observer. It never reuses another
build's offset constants. It also compiles the creation witness against private
public headers and rejects any witness DDS `DT_NEEDED` dependency. No library is
preloaded and no DDS/ROS process is started by this tool.

The generated observer still checks the exact canonical DDS path, GNU BuildID,
function offsets and DSO base before its bounded sampling starts. Moving the
private library to a new path requires rebuilding the observer and generating a
new binding; editing a descriptor alone cannot bypass that guard.

## Generate the local binding

```bash
python3 -B tools/private_dds/bind_runtime.py \
  --bundle .local/private_dds/observers/OBSERVER_BUILD.json
```

This writes a new descriptor, the four-artifact runtime contract, and a binding
receipt under `.local/private_dds/`. It refuses to overwrite an existing
descriptor/contract. For an already independently reviewed tuple on the same
machine, use `--from-contract /path/to/validated/PRIVATE_DDS_CONTRACT.json`
instead. That mode rechecks every existing artifact hash, actual library BuildID,
descriptor, witness dependencies and observer path/BuildID guard before writing
a new local binding. It does not turn an arbitrary binary into a reviewed build.
It preserves the existing artifact locations; do not remove those inputs later.

For a nondefault output directory, select its generated contract explicitly
before preparing the demo:

```bash
export PRIVATE_DDS_RUNTIME_CONTRACT=/absolute/path/to/PRIVATE_DDS_CONTRACT.json
```

The runtime retains the existing `binding`, `configure`, `native_environment`,
`verify_environment` and `ENV_KEYS` interfaces. It hashes the complete artifact
tuple before launching owned children, puts the private library first after all
ROS overlays, rejects unknown/duplicate preloads, and preserves native-only
owner-probe scope. Preparation freezes the generated binding and execution
requires the same one. Directory ownership/freshness rules remain unchanged.

## Validation boundaries

Building/binding does not grant navigation or physical acceptance. At runtime,
the creation-only witness records real factory returns with process identity,
GUID/domain and loaded-library identity. Arguments, QoS and return behavior pass
through. Reconcile records against every initialized and late owned participant,
including regenerated SCAN children. Missing records, partial output, cap errors
or mixed library mappings are coverage failures. A mapped DDS library alone does
not prove a participant was created or intercepted. Administrative domain-query
records are not navigation-participant evidence.

The native observer is diagnostic: retained phase spans, 20ms snapshots and exact
sleep-call samples have explicit coverage limits. Health-count zero does not
prove no wait, and component delivery counts do not prove losslessness. Source
patch regression tests and local build guards do not replace actual loading,
source/ACK freshness, Actor archives or the original navigation/closure gates.
