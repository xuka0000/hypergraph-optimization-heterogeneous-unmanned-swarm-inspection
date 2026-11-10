"""Bounded-memory native LP evidence, independent of Windows Unicode fopen.

Statistics survive artifact failures. Native files are staged at a verified
ASCII absolute path and removed only after a compressed, hashed readback.
"""
from pathlib import Path
import datetime
import gzip
import hashlib
import heapq
import json
import os
import shutil
import tempfile


def write_json(path, value):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), 'utf-8')
    os.replace(temp, path)


def coefficient_statistics(model):
    rows = model.getLPRowsData()
    lo, hi, nnz, worst = float('inf'), 0., 0, []
    for row in rows:
        rlo, rhi = float('inf'), 0.
        for value in row.getVals():
            a = abs(float(value))
            if a:
                lo = min(lo, a); hi = max(hi, a); nnz += 1
                rlo = min(rlo, a); rhi = max(rhi, a)
        if rhi:
            item = (rhi / rlo, rhi, row.name, rlo)
            if len(worst) < 32: heapq.heappush(worst, item)
            elif item > worst[0]: heapq.heapreplace(worst, item)
    return dict(available=True, rows=len(rows), nnz=nnz,
                minimum_nonzero=lo if nnz else None, maximum=hi if nnz else None,
                largest_row_spreads=[dict(name=z[2], minimum=z[3], maximum=z[1], ratio=z[0])
                                     for z in sorted(worst, reverse=True)])


def export_native_lp(model, output, name, nnz=0, live_reservations=()):
    output = Path(output).resolve()
    if Path(name).name != name or not name.endswith('.lp'):
        raise ValueError('An owned LP basename is required')
    staging = Path(tempfile.gettempdir()).resolve()
    if not str(staging).isascii():
        raise OSError('Native LP staging path is not ASCII: ' + str(staging))
    # Count simultaneous raw and gzip writes, other active reservations, and
    # the existing runner output allowance. Never rely on compression ratios.
    estimate = max(64 * 2**20, int(nnz) * 512 + model.getNVars() * 512)
    need = {}
    for p, amount in ((staging, estimate), (output, estimate + 256 * 2**20)):
        drive = p.anchor.upper(); need[drive] = need.get(drive, 0) + amount
    for row in live_reservations:
        if row.get('status') not in ('released', 'cancelled', 'completed'):
            drive = str(row['drive']).rstrip(':/\\').upper() + ':\\'
            need[drive] = need.get(drive, 0) + float(row['peak_gib']) * 2**30
    storage = {}
    for drive, amount in need.items():
        free = shutil.disk_usage(drive).free
        floor = (10 if drive.startswith('C:') else 30) * 2**30
        storage[drive] = dict(free_bytes=free, peak_reserved_bytes=amount, floor_bytes=floor)
        if free - amount < floor:
            raise OSError('LP artifact peak writes do not fit storage floor: ' + json.dumps(storage))
    fd, raw_name = tempfile.mkstemp(prefix='paper11_lp_', suffix='.lp', dir=staging)
    os.close(fd)
    raw = Path(raw_name); packed = output / (name + '.gz'); temp = packed.with_suffix('.gz.tmp')
    if packed.exists() or temp.exists():
        raw.unlink()  # Only this call's empty tempfile.
        raise FileExistsError('Never overwrite earlier native evidence: ' + str(packed))
    try:
        model.writeLP(str(raw))
        if raw.stat().st_size == 0:
            raise OSError('Native writer returned without an LP')
        if raw.stat().st_size > estimate:
            raise OSError('LP exceeded reserved size; raw evidence retained')
        with raw.open('rb') as src, gzip.open(temp, 'wb') as dst:
            h = hashlib.sha256()
            while block := src.read(1024 * 1024):
                h.update(block); dst.write(block)
        with gzip.open(temp, 'rb') as src:
            verified = hashlib.file_digest(src, 'sha256').hexdigest()
        if verified != h.hexdigest():
            raise OSError('Native LP gzip readback hash mismatch')
        os.replace(temp, packed)
        result = dict(path=packed.name, uncompressed_sha256=h.hexdigest(),
                      bytes=raw.stat().st_size, compressed_bytes=packed.stat().st_size,
                      native_ascii_path=str(raw), storage_admission=storage,
                      verified_decompressed_hash=True)
        raw.unlink()
        result['owned_staging_removed'] = not raw.exists()
        return result
    except Exception as exc:
        # Report exact retained files; never erase the sole diagnostic copy.
        raise OSError(str(exc) + '; retained_native_path=' + str(raw)
                      + '; partial_gzip_path=' + str(temp)) from exc


def capture_lp(model, output, name, *, export=True, live_reservations=()):
    output = Path(output)
    receipt = output / (name + '.json')
    result = dict(observed_at=datetime.datetime.now().astimezone().isoformat(), available=False)
    try:
        result.update(coefficient_statistics(model))
    except Exception as exc:
        result['statistics_error'] = repr(exc)
    # Durable statistics precede the fallible native export.
    write_json(receipt, result)
    if export:
        try:
            result['artifact'] = export_native_lp(model, output, name, result.get('nnz', 0), live_reservations)
        except Exception as exc:
            result['artifact_error'] = repr(exc)
        write_json(receipt, result)
    return result
