"""PCD reader shared by map diagnostics; no ROS node initialization."""
import io
import numpy as np


def map_points(data):
    """Read XYZ from the ASCII/binary PCD formats used by the saved-map workflow."""
    stream = io.BytesIO(data)
    header = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError('PCD has no DATA header')
        words = line.decode('ascii').strip().split()
        if not words or words[0].startswith('#'):
            continue
        header[words[0]] = words[1:]
        if words[0] == 'DATA':
            break
    fields = header['FIELDS']
    counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
    if not all(k in fields and counts[fields.index(k)] == 1 for k in ('x', 'y', 'z')):
        raise ValueError('PCD requires scalar x/y/z fields')
    encoding = header['DATA'][0]
    if encoding == 'binary':
        types = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype = np.dtype([(name, '<' + types[kind] + size, (count,))
                          if count != 1 else (name, '<' + types[kind] + size)
                          for name, size, kind, count in
                          zip(fields, header['SIZE'], header['TYPE'], counts)])
        count = int(header['POINTS'][0]) if 'POINTS' in header else int(header['WIDTH'][0]) * int(header['HEIGHT'][0])
        points = np.frombuffer(stream.read(), dtype=dtype, count=count)
        xyz = np.column_stack([points[k] for k in ('x', 'y', 'z')])
    elif encoding == 'ascii':
        offsets = np.cumsum([0] + counts[:-1])
        xyz = np.loadtxt(stream, usecols=[offsets[fields.index(k)] for k in ('x', 'y', 'z')], ndmin=2)
    else:
        raise ValueError('Consistency check requires an ASCII or binary PCD; binary_compressed is unsupported')
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < 100:
        raise ValueError('Map must contain at least 100 finite points')
    return xyz
