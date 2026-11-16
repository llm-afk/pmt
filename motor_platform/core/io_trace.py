"""Optional process-wide hardware capture. Producers enqueue, never write files."""
_sink = None


def install(sink):
    global _sink
    previous, _sink = _sink, sink
    return previous


def remove(sink):
    global _sink
    if _sink is sink:
        _sink = None


def record(event, *, device='', interface='', direction='', raw=b'', **values):
    sink = _sink
    if sink is not None:
        sink.record(event, device=device, interface=interface, direction=direction,
                    raw=raw, **values)
