"""Safe tests for the NAP maintenance toolkit using temporary fake /data roots."""
import argparse
import subprocess
import json
from pathlib import Path
import sqlite3
import tarfile
import tempfile
import unittest
from unittest.mock import patch
from nap_recovery import backup, verify, stage, repo_manifest, OPENDBC_SHA

class RecoveryTests(unittest.TestCase):
    def test_backup_verify_manifest_and_live_wal(self):
        with tempfile.TemporaryDirectory() as name:
            base = Path(name)
            root = base / 'data'
            root.mkdir()
            settings = root / 'nap_settings'
            settings.mkdir()
            (settings / 'v9e_predictive_grade.json').write_text('{"enabled": true}')
            bms = root / 'bms/nap_dash'
            bms.mkdir(parents=True)
            db = bms / 'nap_history.sqlite3'
            con = sqlite3.connect(db)
            con.execute('PRAGMA journal_mode=WAL')
            con.execute('CREATE TABLE sample (id integer, value text)')
            con.execute('INSERT INTO sample VALUES (1, "test")')
            con.commit()
            (bms / 'battery_snapshots.jsonl').write_text('{"a":1}\n')
            (root / 'continue.sh').write_text('# dummy\n')
            args = argparse.Namespace(data_root=str(root), output=str(base/'backups'), include_params=False)
            backup(args)
            archives = list((base/'backups').glob('*.tar.gz'))
            self.assertEqual(len(archives), 1)
            verify(argparse.Namespace(archive=str(archives[0])))
            with tarfile.open(archives[0], 'r:gz') as t:
                m = json.load(t.extractfile('NAP_MANIFEST.json'))
                self.assertEqual(m['files']['nap_settings/v9e_predictive_grade.json']['size'], 17)
                self.assertIn('bms/nap_dash/nap_history.sqlite3', m['files'])
                raw = t.extractfile('bms/nap_dash/nap_history.sqlite3').read()
                tmp = base/'copied.db'
                tmp.write_bytes(raw)
                chk = sqlite3.connect(tmp)
                self.assertEqual(chk.execute('SELECT value FROM sample').fetchone()[0], 'test')
                chk.close()
            con.close()
    def test_stage_refuses_live_path(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            with self.assertRaises(RuntimeError):
                stage(argparse.Namespace(data_root=str(root), destination=str(root/'openpilot')))
    def test_stage_exact_versions_with_local_git_repos(self):
        with tempfile.TemporaryDirectory() as name:
            base = Path(name)
            refs = {}
            def run(*a, cwd=None):
                return subprocess.check_output([*a], cwd=cwd, text=True, stderr=subprocess.STDOUT).strip()
            for label in ('opendbc', 'NAP-Dash', 'openpilot'):
                p = base / f'remote-{label}'
                p.mkdir()
                run('git', 'init', '-q', '-b', 'main', str(p))
                run('git', 'config', 'user.email', 'test@example.com', cwd=p)
                run('git', 'config', 'user.name', 'Test', cwd=p)
                (p / 'file.txt').write_text(label)
                run('git', 'add', '.', cwd=p)
                if label == 'openpilot':
                    run('git', 'update-index', '--add', '--cacheinfo', f'160000,{refs["opendbc"][1]},opendbc_repo', cwd=p)
                run('git', 'commit', '-qm', 'test', cwd=p)
                commit = run('git', 'rev-parse', 'HEAD', cwd=p)
                tag = 'NAP-v1.0.0' if label == 'openpilot' else ('NAP-Dash-v1.0.0' if label == 'NAP-Dash' else 'main')
                if tag != 'main': run('git', 'tag', tag, cwd=p)
                refs[label] = (str(p), commit, tag)
            fake_repos = {label: ('unused', v[1], v[0], v[2]) for label, v in refs.items()}
            with patch('nap_recovery.REPOS', fake_repos), patch('nap_recovery.OPENDBC_SHA', refs['opendbc'][1]):
                target = base / 'staged'
                stage(argparse.Namespace(data_root=str(base/'live'), destination=str(target)))
                self.assertTrue((target/'openpilot/file.txt').is_file())
                self.assertEqual((target/'NAP-Dash/file.txt').read_text(), 'NAP-Dash')
                self.assertTrue((target/'STAGED_README.txt').exists())

    def test_release_manifest_missing_repos(self):
        with tempfile.TemporaryDirectory() as name:
            m = repo_manifest(Path(name))
            self.assertFalse(m['submodule_matches_release'])
            self.assertFalse(m['openpilot']['present'])

if __name__ == '__main__': unittest.main()
