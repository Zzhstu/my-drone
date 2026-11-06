#!/usr/bin/env python3
"""Lossless .bag archival; SHA-256 verification precedes source removal.

Uses the system libzstd without installing packages. Historical experiment
manifests remain unchanged: restore the exact .bag before existing ROS tools.
Only explicitly listed regular files in a saved plan can be archived.
"""
import argparse
import concurrent.futures
import ctypes as C
import ctypes.util
import hashlib
import json
import os
from pathlib import Path
import stat
import tempfile
import time

CHUNK = 4 * 1024 * 1024


class Buffer(C.Structure):
    _fields_ = [('ptr', C.c_void_p), ('size', C.c_size_t), ('pos', C.c_size_t)]


class Zstd:
    def __init__(self):
        name = ctypes.util.find_library('zstd')
        if not name:
            raise RuntimeError('System libzstd is required')
        self.lib = C.CDLL(name)
        for kind in ('C', 'D'):
            f = getattr(self.lib, 'ZSTD_create' + kind + 'Stream')
            f.argtypes = []; f.restype = C.c_void_p
            f = getattr(self.lib, 'ZSTD_free' + kind + 'Stream')
            f.argtypes = [C.c_void_p]; f.restype = C.c_size_t
        self.lib.ZSTD_initCStream.argtypes = [C.c_void_p, C.c_int]
        self.lib.ZSTD_initDStream.argtypes = [C.c_void_p]
        self.lib.ZSTD_compressStream.argtypes = [C.c_void_p, C.POINTER(Buffer), C.POINTER(Buffer)]
        self.lib.ZSTD_decompressStream.argtypes = [C.c_void_p, C.POINTER(Buffer), C.POINTER(Buffer)]
        self.lib.ZSTD_endStream.argtypes = [C.c_void_p, C.POINTER(Buffer)]
        for name in ('initCStream', 'initDStream', 'compressStream', 'decompressStream', 'endStream'):
            getattr(self.lib, 'ZSTD_' + name).restype = C.c_size_t
        self.lib.ZSTD_isError.argtypes = [C.c_size_t]; self.lib.ZSTD_isError.restype = C.c_uint
        self.lib.ZSTD_getErrorName.argtypes = [C.c_size_t]; self.lib.ZSTD_getErrorName.restype = C.c_char_p

    def check(self, value):
        if self.lib.ZSTD_isError(value):
            raise ValueError(self.lib.ZSTD_getErrorName(value).decode())
        return value

    def stream(self, source, destination=None, decompress=False, level=1):
        kind = 'D' if decompress else 'C'
        context = getattr(self.lib, 'ZSTD_create' + kind + 'Stream')()
        if not context:
            raise MemoryError('zstd context allocation')
        input_hash = hashlib.sha256(); output_hash = hashlib.sha256()
        input_bytes = output_bytes = 0
        storage = C.create_string_buffer(CHUNK)

        def emit(out):
            nonlocal output_bytes
            data = storage.raw[:out.pos]
            output_hash.update(data); output_bytes += len(data)
            if destination is not None:
                destination.write(data)
        try:
            init = self.lib.ZSTD_initDStream(context) if decompress else self.lib.ZSTD_initCStream(context, level)
            self.check(init); last = init
            operation = self.lib.ZSTD_decompressStream if decompress else self.lib.ZSTD_compressStream
            while True:
                data = source.read(CHUNK)
                if not data:
                    break
                input_hash.update(data); input_bytes += len(data)
                backing = C.create_string_buffer(data)
                inp = Buffer(C.addressof(backing), len(data), 0)
                # A full output buffer may still leave decoded bytes buffered.
                while True:
                    before = inp.pos
                    out = Buffer(C.addressof(storage), CHUNK, 0)
                    last = self.check(operation(context, C.byref(out), C.byref(inp)))
                    emit(out)
                    if inp.pos == inp.size and (not decompress or out.pos < CHUNK or last == 0):
                        break
                    if before == inp.pos and out.pos == 0:
                        raise ValueError('zstd made no progress')
            if decompress:
                if not input_bytes or last != 0:
                    raise ValueError('Truncated or empty zstd archive')
            else:
                while True:
                    out = Buffer(C.addressof(storage), CHUNK, 0)
                    left = self.check(self.lib.ZSTD_endStream(context, C.byref(out)))
                    emit(out)
                    if left == 0:
                        break
            return dict(input_bytes=input_bytes, output_bytes=output_bytes,
                        input_sha256=input_hash.hexdigest(), output_sha256=output_hash.hexdigest())
        finally:
            getattr(self.lib, 'ZSTD_free' + kind + 'Stream')(context)


def atomic_json(path, value):
    path = Path(path)
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(value, f, indent=2, ensure_ascii=False); f.write('\n')
            f.flush(); os.fsync(f.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def identity(s):
    return s.st_dev, s.st_ino, s.st_size, s.st_mtime_ns, s.st_ctime_ns


def sync_directory(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def archive_one(item):
    source = Path(item['path']); archive = Path(str(source) + '.zst')
    meta_path = Path(str(archive) + '.archive.json')
    if not source.exists():
        if archive.exists() and meta_path.exists():
            return dict(path=str(source), status='already_archived')
        raise FileNotFoundError(source)
    initial = source.lstat()
    if not stat.S_ISREG(initial.st_mode) or initial.st_nlink != 1:
        raise ValueError('Expected single-link regular source file: ' + str(source))
    if identity(initial) != tuple(item['identity']):
        raise ValueError('Source changed since plan: ' + str(source))
    if archive.exists() or meta_path.exists():
        raise FileExistsError('Archive or catalog already exists: ' + str(archive))
    started = time.monotonic(); zstd = Zstd()
    fd, temporary = tempfile.mkstemp(prefix=archive.name + '.', suffix='.tmp', dir=str(source.parent))
    try:
        with source.open('rb') as src, os.fdopen(fd, 'wb') as dst:
            compressed = zstd.stream(src, dst)
            dst.flush(); os.fsync(dst.fileno())
        if item.get('manifest_sha256') and compressed['input_sha256'] != item['manifest_sha256']:
            raise ValueError('Original bag differs from experiment manifest: ' + str(source))
        with open(temporary, 'rb') as src:
            verified = zstd.stream(src, decompress=True)
        if (verified['output_sha256'] != compressed['input_sha256'] or
                verified['output_bytes'] != compressed['input_bytes'] or
                verified['input_sha256'] != compressed['output_sha256']):
            raise ValueError('Full decompression SHA-256 verification failed')
        if identity(source.lstat()) != identity(initial):
            raise ValueError('Source changed while archiving')
        if compressed['output_bytes'] >= compressed['input_bytes']:
            return dict(path=str(source), status='not_smaller_source_preserved')
        # No-clobber publication; this never overwrites an existing archive.
        os.link(temporary, archive); os.unlink(temporary)
        os.utime(archive, ns=(initial.st_atime_ns, initial.st_mtime_ns))
        metadata = dict(version=1, original_path=str(source), archive_path=str(archive),
                        original_bytes=compressed['input_bytes'], archive_bytes=compressed['output_bytes'],
                        original_sha256=compressed['input_sha256'], archive_sha256=compressed['output_sha256'],
                        original_mode=stat.S_IMODE(initial.st_mode), original_mtime_ns=initial.st_mtime_ns,
                        status='verified_original_present', full_decompression_verified=True,
                        scope='Byte-exact lossless archive. Original experiment manifest unchanged; restore .bag before ROS replay.')
        atomic_json(meta_path, metadata)
        sync_directory(source.parent)
        # All original bytes now exist in a fully verified, cataloged archive.
        if identity(source.lstat()) != identity(initial):
            raise ValueError('Source changed before removal; original preserved')
        source.unlink()
        metadata['status'] = 'archived_original_removed'
        metadata['released_bytes'] = metadata['original_bytes']-metadata['archive_bytes']
        metadata['wall_seconds'] = time.monotonic()-started
        atomic_json(meta_path, metadata)
        return metadata
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def restore(archive):
    archive = Path(archive).resolve()
    meta = json.loads(Path(str(archive)+'.archive.json').read_text())
    destination = Path(meta['original_path'])
    if archive != Path(meta['archive_path']) or destination.parent != archive.parent:
        raise ValueError('Archive catalog path mismatch')
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(destination)
    fd, temporary = tempfile.mkstemp(prefix=destination.name+'.', suffix='.restoring', dir=str(destination.parent))
    try:
        with archive.open('rb') as src, os.fdopen(fd, 'wb') as dst:
            result = Zstd().stream(src, dst, decompress=True)
            dst.flush(); os.fsync(dst.fileno())
        if (result['output_sha256'] != meta['original_sha256'] or result['output_bytes'] != meta['original_bytes'] or
                result['input_sha256'] != meta['archive_sha256']):
            raise ValueError('Archive restore hash mismatch; no .bag published')
        os.chmod(temporary, meta['original_mode'])
        os.utime(temporary, ns=(meta['original_mtime_ns'], meta['original_mtime_ns']))
        os.link(temporary, destination); os.unlink(temporary)
        return dict(restored=str(destination), sha256=result['output_sha256'], archive_preserved=True)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    a = sub.add_parser('archive'); a.add_argument('plan', type=Path); a.add_argument('--jobs', type=int, default=2)
    r = sub.add_parser('restore'); r.add_argument('archive', type=Path)
    args = parser.parse_args()
    if args.action == 'restore':
        print(json.dumps(restore(args.archive), ensure_ascii=False)); return
    plan = json.loads(args.plan.read_text())
    if not 1 <= args.jobs <= 2:
        parser.error('Use 1 or 2 workers')
    items = [i for i in plan['bags'] if i['action'] == 'lossless_archive']
    failures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        pending = {pool.submit(archive_one, i): i for i in items}
        for future in concurrent.futures.as_completed(pending):
            try:
                result = future.result()
            except Exception as error:
                result = dict(path=pending[future]['path'], error=str(error)); failures.append(result)
            print(json.dumps(result, ensure_ascii=False), flush=True)
    if failures:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
