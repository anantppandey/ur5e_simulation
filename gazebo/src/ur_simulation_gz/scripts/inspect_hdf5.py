#!/usr/bin/env python3
"""
Inspect HDF5 file and dump a textual summary of groups/datasets, shapes and dtypes.
Usage:
  python3 inspect_hdf5.py /path/to/file.h5 /path/to/output.txt
"""
import sys
import h5py


def format_dataset(name, ds):
    try:
        shape = ds.shape
        dtype = ds.dtype
        size = ds.size
        return f"{name} : dataset shape={shape} dtype={dtype} size={size}\n"
    except Exception as e:
        return f"{name} : dataset (error reading metadata: {e})\n"


def recurse(h5obj, path='/', out_lines=None):
    if out_lines is None:
        out_lines = []
    for key in sorted(h5obj.keys()):
        item = h5obj[key]
        full = path + key
        if isinstance(item, h5py.Dataset):
            out_lines.append(format_dataset(full, item))
        elif isinstance(item, h5py.Group):
            out_lines.append(f"{full}/ : group\n")
            recurse(item, full + '/', out_lines)
        else:
            out_lines.append(f"{full} : unknown HDF5 node\n")
    return out_lines


def inspect_file(h5path):
    out = []
    with h5py.File(h5path, 'r') as f:
        out.append(f"File: {h5path}\n")
        # attributes of root
        if f.attrs:
            out.append("Root attributes:\n")
            for k, v in f.attrs.items():
                out.append(f"  @{k} = {v}\n")
        out.extend(recurse(f, '/', []))
    return out


def main():
    if len(sys.argv) < 3:
        print('Usage: inspect_hdf5.py <input.h5> <output.txt>')
        sys.exit(1)
    h5path = sys.argv[1]
    outpath = sys.argv[2]
    lines = inspect_file(h5path)
    with open(outpath, 'w') as f:
        f.writelines(lines)
    print(f'Wrote summary to {outpath}')

if __name__ == '__main__':
    main()
