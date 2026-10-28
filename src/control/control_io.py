"""Current live control I/O. Developer archive codecs are separate tools."""
from common.runtime_telemetry import RuntimeTelemetry
from common.runtime_io import LatestCommandWriter

def make_evidence_writer(run,profile):
    return RuntimeTelemetry()

def make_command_writer(writer,history,configuration):
    return LatestCommandWriter(writer,None)
