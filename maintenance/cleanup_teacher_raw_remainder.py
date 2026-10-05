#!/usr/bin/env python3
"""Second explicit plan for small Isaac/raw-cloud/video payloads missed by large-log patterns."""
from pathlib import Path
import argparse
import cleanup_teacher_raw as purge
purge.OUT=purge.OUT/'remainder'
def raw_remainder(rel):
 if len(rel.parts)<2 or 'sources' in rel.parts:return False
 n=rel.name
 return n.endswith('_trace.json')or n=='frame_replay.gif'or(n.endswith('.npz')and(n.startswith('raw_lidar_')or n.startswith('cloud_')))
purge.candidate=raw_remainder
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute-plan-sha');p.add_argument('--preserved-commit');p.add_argument('--evidence-manifest',type=Path);a=p.parse_args()
 if a.execute_plan_sha:
  if not a.preserved_commit or not a.evidence_manifest:p.error('preserved remote commit and evidence manifest required')
  purge.execute(a.execute_plan_sha,a.preserved_commit,a.evidence_manifest)
 else:purge.plan()
