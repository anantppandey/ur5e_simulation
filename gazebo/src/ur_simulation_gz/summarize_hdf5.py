#!/usr/bin/env python3
"""
summarize_hdf5.py

Inspect an HDF5 file and produce:
 - a human-readable report (Markdown/text)
 - an optional machine-readable JSON summary
 - optional saved sample images for RGB-like datasets

Usage:
    python summarize_hdf5.py INPUT_H5 -o report.md --json summary.json --images samples/ --samples 5

Author: ChatGPT (adapt & extend as needed)
"""

import argparse
import h5py
import numpy as np
import json
import os
import traceback
from datetime import datetime

try:
    from PIL import Image
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False

# ----------------------
# Utilities / heuristics
# ----------------------
def is_probably_image(ds, shape, dtype):
    # Heuristic: last dimension 3 or 4 and dtype uint8 or last two dims consistent with HxW
    if dtype.kind == 'u' and dtype.itemsize == 1:
        if len(shape) >= 3 and shape[-1] in (3, 4):
            return True
        if len(shape) == 2 and shape[0] <= 4096 and shape[1] <= 4096:
            # could be single-channel image
            return True
    return False

def decode_bytes_array(arr):
    # Convert numpy array of bytes (S) to python strings
    try:
        return [x.decode('utf-8', errors='replace') if isinstance(x, (bytes, bytearray)) else str(x) for x in arr]
    except Exception:
        return [str(x) for x in arr]

def safe_read_slice(ds, slc):
    # read slice from dataset, handle exceptions for weird dtypes
    try:
        return ds[slc]
    except Exception:
        # try coercing to numpy array (may load entire dataset; risk for very large arrays)
        try:
            return np.array(ds)[slc]
        except Exception:
            return None

def sample_indices_for_shape(shape, n_samples):
    if len(shape) == 0:
        return [()]  # scalar dataset
    # assume first axis is frames when >1 dim
    if shape[0] == 0:
        return []
    if shape[0] == 1:
        return [0]
    if shape[0] <= n_samples:
        return list(range(shape[0]))
    # else choose evenly spaced indices across axis 0
    idxs = np.linspace(0, shape[0]-1, num=n_samples, dtype=int).tolist()
    return idxs

def summarize_dataset(ds, path, options):
    """
    Return a dict summary for dataset 'ds'. Does not read entire dataset unless small.
    """
    summary = {}
    try:
        shape = tuple(ds.shape)
    except Exception:
        # Some datasets may not expose shape straightforwardly
        try:
            shape = tuple(np.array(ds).shape)
        except Exception:
            shape = ()
    summary['path'] = path
    summary['shape'] = shape
    summary['maxshape'] = tuple(ds.maxshape) if getattr(ds, 'maxshape', None) is not None else None
    summary['dtype'] = str(ds.dtype)
    summary['chunks'] = tuple(ds.chunks) if getattr(ds, 'chunks', None) is not None else None
    summary['compression'] = getattr(ds, 'compression', None)
    summary['compression_opts'] = getattr(ds, 'compression_opts', None)
    # attributes
    attrs = {}
    for k, v in ds.attrs.items():
        # don't force reading huge attributes; convert to JSON-friendly
        try:
            if isinstance(v, (np.ndarray, list, tuple)):
                attrs[k] = np.array(v).tolist()
            elif isinstance(v, bytes):
                attrs[k] = v.decode('utf-8', errors='replace')
            else:
                attrs[k] = v
        except Exception:
            attrs[k] = str(v)
    summary['attributes'] = attrs

    # quick size estimate (number of elements)
    try:
        n_elem = int(np.prod(shape)) if len(shape) > 0 else 1
    except Exception:
        n_elem = None
    summary['n_elements'] = n_elem

    # Decide sampling strategy
    sample_n = options.samples
    sample_info = {}
    try:
        # if dataset small, read whole; else read first few along axis 0 or small random sample
        if n_elem is not None and n_elem <= options.full_stat_max_elements:
            arr = np.array(ds)
            sample_info['sampled_full'] = True
            sample_arr = arr
        else:
            sample_info['sampled_full'] = False
            # sample along first axis if possible
            if len(shape) >= 1 and shape[0] > 0:
                idxs = sample_indices_for_shape(shape, sample_n)
                # build slice tuple to fetch first element for each index
                samples = []
                for i in idxs:
                    slc = [slice(None)] * len(shape)
                    slc[0] = i
                    val = safe_read_slice(ds, tuple(slc))
                    samples.append(val)
                sample_arr = samples
            else:
                # scalar or empty or no axis 0 -> read first element
                val = safe_read_slice(ds, ())
                sample_arr = [val]
        # store sample values converted to small repr
        # careful with bytes dtype
        def repr_val(v):
            try:
                if v is None:
                    return None
                a = np.array(v)
                if a.size == 0:
                    return "[]"
                if a.dtype.kind in ('S', 'b'):  # byte strings
                    flat = [x.decode('utf-8', errors='replace') if isinstance(x, (bytes, bytearray)) else str(x) for x in a.flatten()[:10]]
                    return flat
                if a.dtype.kind in ('U', 'O'):
                    flat = [str(x) for x in a.flatten()[:10]]
                    return flat
                # numeric
                flat = a.flatten()
                # limit how much we return
                if flat.size > 40:
                    return flat[:40].tolist()
                return flat.tolist()
            except Exception:
                return str(v)
        # add sample representation
        if isinstance(sample_arr, list):
            sample_vals = [repr_val(x) for x in sample_arr]
        else:
            sample_vals = repr_val(sample_arr)
        summary['sample_values'] = sample_vals

        # numeric stats if numeric dtype: compute on sample only
        if ds.dtype.kind in ('i', 'u', 'f') and sample_arr is not None:
            try:
                # convert sample_arr to flat numeric array
                if isinstance(sample_arr, list):
                    flat = np.concatenate([np.array(x).ravel() for x in sample_arr if x is not None and np.array(x).size > 0]) \
                           if any(x is not None and np.array(x).size > 0 for x in sample_arr) else np.array([])
                else:
                    flat = np.array(sample_arr).ravel()
                if flat.size > 0:
                    sample_info['min'] = float(np.nanmin(flat))
                    sample_info['max'] = float(np.nanmax(flat))
                    sample_info['mean'] = float(np.nanmean(flat))
                    sample_info['std'] = float(np.nanstd(flat))
                    sample_info['sample_count'] = int(flat.size)
                else:
                    sample_info['note'] = "sample contained no values"
            except Exception:
                sample_info['stats_error'] = traceback.format_exc()
        else:
            sample_info['note'] = 'non-numeric or stats skipped'
    except Exception:
        sample_info['sample_error'] = traceback.format_exc()

    summary['sample_info'] = sample_info

    # Heuristic: is image-like?
    try:
        if is_probably_image(ds, shape, ds.dtype):
            summary['heuristic_type'] = 'image_like'
        else:
            summary['heuristic_type'] = None
    except Exception:
        summary['heuristic_type'] = None

    # Add recommended saving format (heuristic)
    rec = {}
    name = os.path.basename(path).lower()
    if 'rgb' in name or 'image' in name or 'color' in name:
        rec['save_as'] = 'uint8 HxWx3 (RGB), dtype=uint8, compression: gzip (optional)'
        rec['notes'] = 'store as bytes (0-255). If multiple cameras, use group per-camera.'
    elif 'depth' in name:
        rec['save_as'] = 'float32 HxW, units = meters (use NaN/-1 for invalid)'
        rec['notes'] = 'store per-pixel depth in meters; include camera intrinsics as metadata.'
    elif 'seg' in name or 'label' in name or 'mask' in name:
        rec['save_as'] = 'int32 HxW (semantic class ids) or uint16 if many classes'
    elif 'pose' in name or 'cam_pose' in name or 'extrinsic' in name:
        rec['save_as'] = 'N x 7 array: [x,y,z,qx,qy,qz,qw], dtype=float32; units=m and quaternion order x,y,z,w'
    elif 'joint' in name or 'joint_positions' in name or 'joint_state' in name:
        rec['save_as'] = 'N x n_joints float32 (radians/meters), store joint_names in root attributes'
    elif ds.dtype.kind in ('S', 'O', 'U'):
        rec['save_as'] = 'string dataset (utf-8)'
    else:
        rec['save_as'] = f'keep same dtype {ds.dtype}; consider chunking on frame axis and gzip compression'
    summary['recommended_save'] = rec

    return summary

    # end summarize_dataset

def walk_hdf5_and_summarize(h5path, options):
    out = {
        'file': os.path.abspath(h5path),
        'inspected_on': datetime.utcnow().isoformat() + 'Z',
        'datasets': [],
        'groups': [],
        'root_attributes': {},
    }
    with h5py.File(h5path, 'r') as f:
        # root attributes
        for k, v in f.attrs.items():
            try:
                if isinstance(v, (np.ndarray, list, tuple)):
                    out['root_attributes'][k] = np.array(v).tolist()
                elif isinstance(v, bytes):
                    out['root_attributes'][k] = v.decode('utf-8', errors='replace')
                else:
                    out['root_attributes'][k] = v
            except Exception:
                out['root_attributes'][k] = str(v)

        # walk
        def visitor(name, obj):
            if isinstance(obj, h5py.Dataset):
                try:
                    s = summarize_dataset(obj, name, options)
                    out['datasets'].append(s)
                except Exception:
                    out['datasets'].append({'path': name, 'error': traceback.format_exc()})
            elif isinstance(obj, h5py.Group):
                # add group info (list children)
                try:
                    children = list(obj.keys())
                except Exception:
                    children = []
                out['groups'].append({'path': name, 'children': children})
        f.visititems(visitor)
    return out

def write_markdown_report(summary, out_path, options):
    lines = []
    lines.append(f"# HDF5 inspection report")
    lines.append(f"**File**: `{summary['file']}`")
    lines.append(f"**Inspected on (UTC)**: {summary['inspected_on']}")
    lines.append("")
    if summary['root_attributes']:
        lines.append("## Root attributes")
        for k, v in summary['root_attributes'].items():
            lines.append(f"- **{k}**: `{v}`")
        lines.append("")

    lines.append("## Groups (top-level + nested)")
    for g in summary['groups']:
        lines.append(f"- `{g['path']}` (children: {g['children']})")
    lines.append("")

    lines.append("## Datasets")
    for ds in summary['datasets']:
        lines.append(f"### `{ds.get('path')}`")
        if ds.get('dtype'):
            lines.append(f"- shape: `{ds.get('shape')}`")
            lines.append(f"- maxshape: `{ds.get('maxshape')}`")
            lines.append(f"- dtype: `{ds.get('dtype')}`")
            lines.append(f"- chunks: `{ds.get('chunks')}`")
            lines.append(f"- compression: `{ds.get('compression')}` (opts: `{ds.get('compression_opts')}`)")
        if ds.get('attributes'):
            lines.append(f"- attributes:")
            for k, v in ds['attributes'].items():
                lines.append(f"  - `{k}`: `{v}`")
        # sample values
        sv = ds.get('sample_values')
        if sv is not None:
            lines.append("- sample values / first few entries:")
            lines.append("```")
            lines.append(str(sv))
            lines.append("```")
        # sample stats
        si = ds.get('sample_info', {})
        if si:
            lines.append("- sample statistics / notes:")
            for kk, vv in si.items():
                lines.append(f"  - {kk}: {vv}")
        # heuristic
        if ds.get('heuristic_type'):
            lines.append(f"- heuristic_type: `{ds.get('heuristic_type')}`")
        # recommendation
        rec = ds.get('recommended_save')
        if rec:
            lines.append("- recommended save format:")
            for k, v in rec.items():
                lines.append(f"  - {k}: {v}")
        lines.append("")

    # write file
    with open(out_path, 'w', encoding='utf-8') as fo:
        fo.write('\n'.join(lines))
    return out_path

def save_sample_images(h5path, summary, images_outdir, options):
    if not PIL_AVAILABLE:
        print("PIL not available; skipping image saves.")
        return []
    os.makedirs(images_outdir, exist_ok=True)
    saved = []
    with h5py.File(h5path, 'r') as f:
        for ds in summary['datasets']:
            path = ds.get('path')
            if ds.get('heuristic_type') == 'image_like':
                try:
                    dset = f[path]
                    shape = dset.shape
                    # choose sample indices
                    idxs = sample_indices_for_shape(shape, options.samples)
                    for ii, i in enumerate(idxs):
                        # create slicing tuple
                        if len(shape) == 3:
                            # (H,W,3) no frame axis
                            arr = safe_read_slice(dset, ())
                        else:
                            # assume axis0 is frames
                            slc = [slice(None)] * len(shape)
                            slc[0] = i
                            arr = safe_read_slice(dset, tuple(slc))
                        if arr is None:
                            continue
                        arr_np = np.array(arr)
                        # if shape is (H,W) -> grayscale; if (H,W,3) -> rgb
                        if arr_np.dtype != np.uint8:
                            # try to cast if values in [0,255]
                            try:
                                arr_np = np.clip(arr_np, 0, 255).astype(np.uint8)
                            except Exception:
                                # skip saving non-uint8 images
                                continue
                        # ensure H,W,3 ordering
                        if arr_np.ndim == 2:
                            mode = 'L'
                        elif arr_np.ndim == 3 and arr_np.shape[2] == 3:
                            mode = 'RGB'
                        elif arr_np.ndim == 3 and arr_np.shape[2] == 4:
                            mode = 'RGBA'
                        else:
                            # unexpected shape
                            continue
                        img = Image.fromarray(arr_np, mode=mode)
                        safe_name = path.replace('/', '__').strip('_')
                        outfn = os.path.join(images_outdir, f"{safe_name}_sample{ii}.png")
                        img.save(outfn)
                        saved.append(outfn)
                except Exception:
                    # continue on error - we don't want to abort whole process
                    continue
    return saved

# ----------------------
# CLI and main
# ----------------------
def parse_args():
    p = argparse.ArgumentParser(description="Inspect HDF5 and write a comprehensive report + JSON summary.")
    p.add_argument("h5file", help="Path to input HDF5 file")
    p.add_argument("-o", "--out", help="Path to output Markdown report (default: report.md)", default="report.md")
    p.add_argument("--json", help="Optional JSON summary output (machine readable)")
    p.add_argument("--images", help="Optional directory to save sample images (PNG) for RGB-like datasets")
    p.add_argument("--samples", type=int, default=5, help="Number of frame samples to show per dataset (default 5)")
    p.add_argument("--full-stat-max-elements", type=int, default=2000000, help="If dataset has <= this many elements, compute full stats (default 2e6)")
    return p.parse_args()

def main():
    args = parse_args()
    class Options:
        samples = args.samples
        full_stat_max_elements = args.full_stat_max_elements
    options = Options()

    print(f"Inspecting {args.h5file} ...")
    summary = walk_hdf5_and_summarize(args.h5file, options)
    # write markdown report
    out_md = args.out
    write_markdown_report(summary, out_md, options)
    print(f"Wrote report: {out_md}")
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fj:
            json.dump(summary, fj, indent=2, default=str)
        print(f"Wrote JSON summary: {args.json}")
    if args.images:
        saved = save_sample_images(args.h5file, summary, args.images, options)
        if saved:
            print(f"Saved sample images: {len(saved)} files to {args.images}")
        else:
            print("No sample images saved (none found or PIL missing).")
    print("Done.")

if __name__ == "__main__":
    main()
