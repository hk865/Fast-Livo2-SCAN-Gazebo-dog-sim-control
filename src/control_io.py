"""Current live control I/O. Developer archive codecs are separate tools."""
from runtime_telemetry import RuntimeTelemetry
from runtime_io import LatestCommandWriter

def make_evidence_writer(run,profile):
    return RuntimeTelemetry()

def make_command_writer(writer,history,configuration):
    return LatestCommandWriter(writer,None)
