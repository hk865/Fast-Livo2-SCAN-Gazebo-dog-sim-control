"""One owned ROS77 installed-JTC profile contract. No Gazebo or body command."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

frozen = json.loads((ROOT / "test_results/full18_freeze/source_manifest.json").read_text())["sha256"]
before = {name:sha(ROOT / name) for name in frozen}
attempt = HERE / ("actual_" + time.strftime("%Y%m%d_%H%M%S"))
attempt.mkdir()
binary = HERE / "build/jtc_classic_profile_native"
for name in ["fixture.cpp", "parameters.yaml", "CMakeLists.txt", "prepare.py", "preparation.json", "run_fixture.py", "analyze.py"]:
    shutil.copyfile(HERE / name, attempt / name)
shutil.copyfile(binary, attempt / binary.name)
env = os.environ.copy()
env["ROS_DOMAIN_ID"] = "77"
env["ROS_LOCALHOST_ONLY"] = "1"
output = attempt / "true_batch1.jsonl"
with (attempt / "true_batch1.log").open("w") as log:
    start = time.monotonic()
    process = subprocess.Popen([str(attempt / binary.name), str(attempt / "parameters.yaml"),
        str(output), "1", "1"], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    timeout = False
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        timeout = True
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
owned = subprocess.run(["ps", "-g", str(process.pid), "-o", "pid=,args="], capture_output=True, text=True)
after = {name:sha(ROOT / name) for name in frozen}
receipt = dict(scope=__doc__, domain=77, batch=1, selected_flag=True,
    execution=dict(pid=process.pid, exit_code=process.returncode, timeout=timeout,
        owned_group_clean=not owned.stdout.strip(), wall_duration_s=time.monotonic() - start),
    production_source_count=len(frozen), production_fingerprint_match_before=before == frozen,
    production_fingerprint_match_after=after == frozen,
    production_changes=[name for name in frozen if before[name] != after[name]],
    source_sha256={name:sha(attempt / name) for name in ["fixture.cpp", "parameters.yaml", "CMakeLists.txt",
        "prepare.py", "preparation.json", "run_fixture.py", "analyze.py", binary.name]},
    external_CM_enforce_not_instantiated=True)
(attempt / "execution_receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
(HERE / "latest_attempt.txt").write_text(str(attempt) + "\n")
print(json.dumps(dict(attempt=str(attempt), **receipt["execution"]), indent=2))
raise SystemExit(not (process.returncode == 0 and not timeout and not owned.stdout.strip()))
