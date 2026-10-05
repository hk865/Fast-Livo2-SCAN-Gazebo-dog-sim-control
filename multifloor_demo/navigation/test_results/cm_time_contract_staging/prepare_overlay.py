#!/usr/bin/env python3
"""Prepare a requested isolated CM workspace; never overwrites an unowned package."""
import argparse
import hashlib
import io
import json
import tarfile
from pathlib import Path

HERE=Path(__file__).resolve().parent
ARCHIVE_SHA="6faa6cd12e84cc273afc7d0eebe050e8d272fc20dd7fb593d3e028e8000b93c7"
BASE_CPP_SHA="abb0f3eaf7f6deec65ad16749f421ffda831341eb527eb13241b1568941c5b54"
CANDIDATE_CPP_SHA="8aff7a7a24c8267e9227e98988050df367a9d52ef5307978a15aa4b95a073645"


def sha(data): return hashlib.sha256(data).hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--workspace",required=True,type=Path,
                        help="Explicit isolated workspace. Production use requires the root's adoption decision.")
    args=parser.parse_args()
    workspace=args.workspace.resolve()
    package=workspace/"src/controller_manager"
    marker=workspace/".demo_cm_overlay_provenance.json"
    if package.exists() and not marker.exists():
        raise RuntimeError("Refusing to overwrite a package without this overlay's provenance marker")
    if marker.exists():
        previous=json.loads(marker.read_text())
        if previous.get("candidate_cpp_sha256")!=CANDIDATE_CPP_SHA:
            raise RuntimeError("Existing overlay has a different patch identity")
        current=package/"src/controller_manager.cpp"
        if current.exists() and sha(current.read_bytes())!=CANDIDATE_CPP_SHA:
            raise RuntimeError("Existing overlay source was modified; refusing to overwrite")
    archive=(HERE/"ros2_control-4.45.2.tar.gz").read_bytes()
    if sha(archive)!=ARCHIVE_SHA: raise RuntimeError("Upstream archive identity mismatch")
    upstream_license=None
    with tarfile.open(fileobj=io.BytesIO(archive),mode="r:gz") as source:
        for member in source.getmembers():
            parts=Path(member.name).parts
            if len(parts)==2 and parts[1] in ("LICENSE","LICENSE.txt") and member.isfile():
                upstream_license=source.extractfile(member).read()
            if len(parts)<2 or parts[1]!="controller_manager": continue
            relative=Path(*parts[2:])
            if ".." in relative.parts: raise RuntimeError("Unsafe archive path")
            destination=package/relative
            if member.isdir(): destination.mkdir(parents=True,exist_ok=True)
            elif member.isfile():
                destination.parent.mkdir(parents=True,exist_ok=True)
                data=source.extractfile(member).read()
                if str(relative)=="src/controller_manager.cpp":
                    if sha(data)!=BASE_CPP_SHA: raise RuntimeError("Baseline CPP identity mismatch")
                    text=data.decode()
                    old_time="const rclcpp::Time current_time = get_clock()->started() ? get_trigger_clock()->now() : time;"
                    new_time="const rclcpp::Time current_time =\n        use_sim_time_ ? time : (get_clock()->started() ? get_trigger_clock()->now() : time);"
                    old_trigger="loaded_controller.c->trigger_update(this->now(), controller_actual_period);"
                    new_trigger="loaded_controller.c->trigger_update(use_sim_time_ ? time : this->now(), controller_actual_period);"
                    if text.count(old_time)!=1 or text.count(old_trigger)!=1:
                        raise RuntimeError("Expected source contract not found uniquely")
                    data=text.replace(old_time,new_time).replace(old_trigger,new_trigger).encode()
                    if sha(data)!=CANDIDATE_CPP_SHA: raise RuntimeError("Candidate CPP identity mismatch")
                destination.write_bytes(data)
            else: raise RuntimeError("Unexpected archive link/device member")
    if upstream_license is None: raise RuntimeError("Missing upstream repository license")
    license_path=package/"LICENSE"
    if license_path.exists() and license_path.read_bytes()!=upstream_license:
        raise RuntimeError("Package LICENSE differs from the official archive; refusing to replace it")
    license_path.write_bytes(upstream_license)
    provenance={"upstream_url":"https://github.com/ros-controls/ros2_control/tree/4.45.2/controller_manager",
                "upstream_archive_sha256":ARCHIVE_SHA,"upstream_package_version":"4.45.2",
                "baseline_cpp_sha256":BASE_CPP_SHA,"candidate_cpp_sha256":CANDIDATE_CPP_SHA,
                "license":"Apache-2.0; upstream source headers and repository license preserved",
                "scope":"use_sim_time true CM caller-time contract only; non-sim branch unchanged",
                "changed_cpp_files":["src/controller_manager.cpp"],
                "physical_reset_supported":False,"workspace":str(workspace)}
    marker.write_text(json.dumps(provenance,indent=2)+"\n")
    (package/"DEMO_PATCH_PROVENANCE.json").write_text(json.dumps(provenance,indent=2)+"\n")
    print(json.dumps(provenance,indent=2))


if __name__=="__main__": main()
