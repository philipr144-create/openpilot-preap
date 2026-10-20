"""Offline unit tests; no NAP hardware, network, or vehicle required."""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile
import unittest
from unittest.mock import patch

import nap_package as n


def sha(b):
    return hashlib.sha256(b).hexdigest()


def create_tar(base: Path, filename: str, content: dict[str, bytes]):
    p = base / filename
    with tarfile.open(p, 'w:gz') as tf:
        for name, data in content.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o600
            tf.addfile(info, io.BytesIO(data))
    Path(str(p) + '.sha256').write_text(sha(p.read_bytes()) + '  ' + p.name + '\n')
    return p


def sample_archives(base: Path):
    private_files = {
        'continue.sh': b'#!/bin/sh\n',
        'nap_settings/nap_tailscale_public_url': b'https://secret.example/\n',
        'nap_settings/v9e_predictive_grade.json': b'{"enabled":true}',
        'params/d/Secret': b'some-private-token',
    }
    manifest = {'files': {k: {'sha256': sha(v), 'size': len(v)} for k, v in private_files.items()}}
    private_files['NAP_MANIFEST.json'] = json.dumps(manifest).encode()
    private = create_tar(base, 'private.tar.gz', private_files)
    runtime_files = {f'nap-runtime/bin/{name}': name.encode() for name in ('tailscale', 'tailscaled', 'ffmpeg')}
    sums = ''.join(f'{sha(name.encode())}  bin/{name}\n' for name in ('tailscale', 'tailscaled', 'ffmpeg'))
    runtime_files['nap-runtime/SHA256SUMS'] = sums.encode()
    runtime_files['nap-runtime/README.txt'] = b'testing\n'
    runtime = create_tar(base, 'runtime.tar.gz', runtime_files)
    hashes = {name: sha(name.encode()) for name in ('tailscale', 'tailscaled', 'ffmpeg')}
    return private, runtime, hashes


def cmd(*args, cwd=None):
    return subprocess.check_output(args, cwd=cwd, text=True, stderr=subprocess.DEVNULL).strip()


def repo(base: Path, label: str, files: dict, gitlink=None):
    p = base / f'remote-{label}'
    cmd('git', 'init', '-q', '-b', 'main', str(p))
    cmd('git', 'config', 'user.email', 'test@example.com', cwd=p)
    cmd('git', 'config', 'user.name', 'Test', cwd=p)
    for key, val in files.items():
        dst = p / key
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_text(val)
    cmd('git', 'add', '.', cwd=p)
    if gitlink:
        cmd('git', 'update-index', '--add', '--cacheinfo', f'160000,{gitlink},opendbc_repo', cwd=p)
    cmd('git', 'commit', '-qm', 'test fixture', cwd=p)
    digest = cmd('git', 'rev-parse', 'HEAD', cwd=p)
    tag = {'openpilot':'NAP-v1.0.2', 'opendbc':'NAP-opendbc-v1.0.0', 'NAP-Dash':'NAP-Dash-v1.0.1'}[label]
    cmd('git', 'tag', tag, cwd=p)
    return p, tag, digest


class PackageTests(unittest.TestCase):
    def test_private_and_runtime_archive_validation(self):
        with tempfile.TemporaryDirectory() as td:
            private, runtime, hashes = sample_archives(Path(td))
            with patch.dict(n.RUNTIME_SHA, hashes):
                mf, entries = n.validate_private(private)
                self.assertEqual(len(mf['files']), 4)
                self.assertIn('params/d/Secret', entries)
                n.validate_runtime(runtime)
                # A valid archive whose contents change no longer matches its sidecar.
                with private.open('ab') as f:
                    f.write(b'changed')
                with self.assertRaises(n.PackageError):
                    n.validate_private(private)

    def test_sidecar_from_original_comma_absolute_path(self):
        # Real runtime bundles created on the comma use an absolute pathname
        # in their sha256sum sidecar; copying to Mac changes the directory.
        with tempfile.TemporaryDirectory() as td:
            base = Path(td)
            archive = create_tar(base, 'runtime.tar.gz', {'data': b'contents'})
            sidecar = Path(str(archive) + '.sha256')
            sidecar.write_text(sha(archive.read_bytes()) +
                               '  /data/nap_recovery/runtime.tar.gz\n')
            n.archive_sidecar(archive)
            sidecar.write_text(sha(archive.read_bytes()) +
                               '  /data/nap_recovery/other.tar.gz\n')
            with self.assertRaises(n.PackageError):
                n.archive_sidecar(archive)

    def test_archive_path_traversal_and_symlink_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            bad = create_tar(root, 'bad.tar.gz', {'../outside': b'bad'})
            with self.assertRaises(n.PackageError):
                n.read_members(bad)
            evil = root/'evil.tar.gz'
            with tarfile.open(evil,'w:gz') as tf:
                info = tarfile.TarInfo('link')
                info.type = tarfile.SYMTYPE
                info.linkname = '/etc/passwd'
                tf.addfile(info)
            with self.assertRaises(n.PackageError):
                n.read_members(evil)

    def test_protected_destinations(self):
        for d in ('/', '/data', '/data/openpilot/new', '/data/server', '/data/params/any', '/usr/local'):
            with self.subTest(path=d), self.assertRaises(n.PackageError):
                n.refuse_unsafe_destination(Path(d))

    def test_check_and_complete_isolated_preview_with_local_test_repos(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            private, runtime, hashes = sample_archives(root)
            odb = repo(root, 'opendbc', {'README':'custom opendbc\n'})
            dashfiles = {
                'server_v21.py': 'pass\n',
                'tailscaled_daemon.py': 'from pathlib import Path\nPUBLIC_URL = _read_private_public_url()\n',
                'phone_navigation/route_tracking.py':'pass\n',
                'phone_navigation/desire_bridge.py':'pass\n',
            }
            dash = repo(root, 'NAP-Dash', dashfiles)
            opfiles = {
                'system/manager/process_config.py':'# server.server_v21 server.tailscaled_daemon\n',
                'selfdrive/nap/grade_toggle_ui.py':'pass\n',
                'selfdrive/controls/lib/predictive_grade_assist.py':'pass\n',
                'launch_openpilot.sh':'#!/bin/sh\n',
                '.lfsconfig':'[lfs]\n url = https://github.com/philipr144-create/openpilot-preap.git/info/lfs\n',
            }
            op = repo(root, 'openpilot', opfiles, gitlink=odb[2])
            releases = {'opendbc': (str(odb[0]), odb[1], odb[2]), 'NAP-Dash': (str(dash[0]), dash[1], dash[2]), 'openpilot': (str(op[0]), op[1], op[2])}
            args = type('Args', (), {'private': str(private), 'runtime': str(runtime), 'destination': str(root/'preview')})()
            with patch.dict(n.RELEASES, releases), patch.dict(n.RUNTIME_SHA, hashes), patch.object(n,'validate_lfs', return_value=249), patch.object(n.shutil, 'which', return_value='/usr/bin/git'):
                n.prepare(args)
            stage = root/'preview'
            self.assertTrue((stage/'data/openpilot/opendbc_repo/README').is_file())
            self.assertTrue((stage/'data/server/tailscaled_daemon.py').is_file())
            self.assertTrue((stage/'PRIVATE_REVIEW/params/d/Secret').is_file())
            self.assertFalse((stage/'data/params').exists())
            self.assertTrue((stage/'data/tailscale/bin/tailscaled').is_file())
            self.assertEqual((stage/'data/ffmpeg-7.0.2-arm64-static/ffmpeg').read_bytes(), b'ffmpeg')
            report=json.loads((stage/'REHEARSAL_REPORT.json').read_text())
            self.assertEqual(report['lfs_files_verified'], 249)
            with self.assertRaises(n.PackageError):
                n.refuse_unsafe_destination(stage)


if __name__ == '__main__':
    unittest.main()
