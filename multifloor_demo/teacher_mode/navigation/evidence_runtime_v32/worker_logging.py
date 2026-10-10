"""Log projection only: no command validation, action, response, or guard changes.

Mission46's synchronous telemetry remains synchronous and retains every outer
native safety field. Full original command bytes are recorded separately once
per consecutive (sequence, raw SHA) change through a bounded optional writer.
"""
import json
from pathlib import Path
from .bounded_binary import BoundedBinaryWriter

DEFAULT_WINDOWS=((3.,4.5),(10.,12.),(208.5,215.))

def compact_envelope(value):
    if not isinstance(value,dict):return None
    out={k:v for k,v in value.items() if k!='acceptance'}
    acceptance=value.get('acceptance')
    if isinstance(acceptance,dict):
        out['acceptance']={k:acceptance[k] for k in ('sha256','mode','allowed') if k in acceptance}
        out['acceptance_recording_projection']=True
    return out

class RecordingContext:
    def __init__(self, run, *, windows=DEFAULT_WINDOWS, maximum_command_bytes=768*1024*1024,
                 maximum_height_bytes=64*1024*1024):
        self.run=Path(run);self.windows=tuple((float(a),float(b)) for a,b in windows)
        if any(a<0 or b<a for a,b in self.windows):raise ValueError('Invalid evidence time window')
        self.directory=self.run/'bounded_worker_evidence';self.directory.mkdir(exist_ok=True)
        self.command_writer=BoundedBinaryWriter(self.directory/'original_commands.bin',maximum_file_bytes=maximum_command_bytes)
        self.height_writer=BoundedBinaryWriter(self.directory/'height_sources.bin',maximum_file_bytes=maximum_height_bytes)
        self.last_key=None;self.last_submission=None;self.height_index=0
        self.command_changes=0;self.command_original_unavailable=0

    def project_evidence(self, evidence):
        if not isinstance(evidence,dict):return evidence
        if evidence.get('recording_schema')=='actual_read_compact_evidence/v1':return dict(evidence)
        attempt=evidence.get('actual_read_attempt') or {}
        envelope=evidence.get('actual_original_envelope') or attempt.get('decoded_envelope')
        sequence=envelope.get('sequence') if isinstance(envelope,dict) else None
        raw_sha=attempt.get('raw_bytes_sha256');key=(sequence,raw_sha)
        if key!=self.last_key:
            self.command_changes+=1
            raw=attempt.get('raw_utf8')
            metadata=dict(schema='actual_original_command_read/v1',command_sequence=sequence,
                read_clock_ns=attempt.get('read_clock_ns'),read_monotonic_wall=attempt.get('read_monotonic_wall'),
                native_physics_clock_ns=evidence.get('physics_clock_ns'),raw_bytes_sha256=raw_sha,
                expected_body_sha256=raw_sha,source='unchanged actual command read before validation',
                navigation_ground_truth_used=False)
            submitted=isinstance(raw,str) and isinstance(raw_sha,str) and self.command_writer.submit(metadata,raw)
            if not isinstance(raw,str) or not isinstance(raw_sha,str):self.command_original_unavailable+=1
            self.last_key=key;self.last_submission=dict(sequence=sequence,raw_bytes_sha256=raw_sha,
                full_original_submitted=bool(submitted),optional_capture='bounded_worker_evidence/original_commands.bin',
                complete_capture_requires_final_writer_stats=True)
        out={k:v for k,v in evidence.items() if k not in ('actual_read_attempt','actual_original_envelope')}
        out['actual_read_attempt']={k:v for k,v in attempt.items() if k not in ('raw_utf8','decoded_envelope')}
        out['actual_read_attempt'].update(raw_utf8_recorded_inline=False,decoded_envelope_recorded_inline=False)
        out['command_envelope_projection']=compact_envelope(envelope)
        out['original_command_recording']=dict(self.last_submission or {})
        out['recording_schema']='actual_read_compact_evidence/v1'
        return out

    def project_telemetry(self, row):
        if not isinstance(row,dict):return row
        out=dict(row)
        if isinstance(out.get('closed_loop_read_evidence'),dict):
            out['closed_loop_read_evidence']=self.project_evidence(out['closed_loop_read_evidence'])
            out['recording_projection']='native telemetry fields unchanged; duplicated command bytes external'
        return out

    def project_height(self, row):
        if not isinstance(row,dict) or 'height_scan_raw' not in row:return row
        t=float(row['sim_time']);detail=any(a<=t<=b for a,b in self.windows)
        out=dict(row)
        if not detail:
            for key in ('height_scan_raw','height_scan_hit_z','height_scan_valid_mask','height_scan_hit_models'):
                out.pop(key,None)
        out['detailed_height_source_recorded']=detail
        out['recording_windows_elapsed_sim_s']=[list(w) for w in self.windows]
        return out

    def height_sink(self):
        context=self
        class Sink:
            closed=False
            def write(self,text):
                if self.closed:raise ValueError('Closed optional height sink')
                # Small summaries outside windows; whole unchanged detailed row
                # in windows. Never used by Actor or Mission46 safety consumers.
                context.height_index+=1
                context.height_writer.submit(dict(schema='actor_height_source_record/v1',
                    record_index=context.height_index,navigation_ground_truth_used=False),text)
                return len(text)
            def flush(self):pass
            def close(self):self.closed=True
        return Sink()

    def install_height_path(self, legacy):
        """Process-local override of only height_scan_sources.jsonl open('w')."""
        base=type(Path());context=self
        class EvidencePath(base):
            def open(self,*args,**kwargs):
                mode=args[0] if args else kwargs.get('mode','r')
                if self.name=='height_scan_sources.jsonl' and mode=='w' and self.parent.resolve()==context.run.resolve():
                    return context.height_sink()
                return super().open(*args,**kwargs)
        legacy.Path=EvidencePath

    def close(self):
        result=dict(schema='teacher_recording_only_log_projection/v1',
            actor_or_navigation_algorithm_changed=False,synchronous_native_telemetry_preserved=True,
            command_validation_response_ack_guards_and_deadlines_unchanged=True,
            height_detail_elapsed_sim_windows=[list(x) for x in self.windows],
            consecutive_command_sequence_hash_changes=self.command_changes,
            original_command_unavailable_on_changes=self.command_original_unavailable,
            original_commands=self.command_writer.close(),height_sources=self.height_writer.close(),
            truth_source='native telemetry offline only; no truth navigation publisher')
        (self.directory/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
        return result
