#!/usr/bin/env python3
"""NAP production maintenance toolkit (openpilot v1.0.1).

Audit, private runtime backup, verification, and release source staging only.
NEVER activates vehicle controls, changes Params, alters active git checkouts,
or starts/stops services. Python 3.10+ standard library only.
"""
from __future__ import annotations
import argparse
import datetime as dt
from contextlib import closing
import hashlib
import json
import os
from pathlib import Path
import shutil
import sqlite3
import subprocess
import sys
import tarfile
import tempfile

OPENPILOT_SHA = "fb41fc71b4266e0a92a4550c51c2bfb1ef2a7e91"
OPENDBC_SHA = "31da4137f2ce06702bc595a05689d537f046bb70"
DASH_SHA = "a9eb509e96a2555ecf979149d8121ad72430695e"
REPOS = {
    "openpilot": ("openpilot", OPENPILOT_SHA, "https://github.com/philipr144-create/openpilot-preap.git", "NAP-v1.0.2"),
    "opendbc": ("openpilot/opendbc_repo", OPENDBC_SHA, "https://github.com/philipr144-create/opendbc-preap.git", "NAP-opendbc-v1.0.0"),
    "NAP-Dash": ("server", DASH_SHA, "https://github.com/philipr144-create/NAP-Dash.git", "NAP-Dash-v1.0.1"),
}
# All user-specific data in backups is PRIVATE. Never commit the output to GitHub.
CONFIG_PATHS = [
    "nap_settings", "nap_params.json", "nap_settings.json", "nap_turn_settings.json",
    "nap_destination_presets.json", "nap_osm_cache.json", "continue.sh",
    "server/phone_navigation-deployment.json",
    "server/tailscaled_daemon.py", "server/cloudflared_tunnel.py",
    "server/.gitignore", "openpilot/.gitmodules",
]
BMS_PATH = "bms/nap_dash"
BMS_LOG_SUFFIXES = (".jsonl", ".json")


def shell(args, cwd=None, *, timeout=90):
    try:
        p = subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return (1, "", str(exc))
    return (p.returncode, p.stdout.strip(), p.stderr.strip())


def git_value(repo: Path, *args):
    code, out, _ = shell(["git", "-C", str(repo), *args])
    return out if code == 0 else None


def repo_manifest(root: Path):
    d = {}
    for label, (rel, expected, url, tag) in REPOS.items():
        path = root / rel
        if not (path / ".git").exists():
            # A submodule .git can be a file rather than a directory.
            if not (path / ".git").is_file():
                d[label] = {"present": False, "expected_commit": expected, "repository": url}
                continue
        commit = git_value(path, "rev-parse", "HEAD")
        status = git_value(path, "status", "--porcelain=v1", "--untracked-files=normal")
        d[label] = {
            "present": True, "commit": commit, "expected_commit": expected,
            "matches_release": commit == expected, "branch": git_value(path, "branch", "--show-current"),
            "dirty": bool(status), "status_lines": (status or "").splitlines()[:50],
            "origin": git_value(path, "remote", "get-url", "origin"),
            "repository": url, "release_ref": tag,
        }
    op = root / "openpilot"
    pin = git_value(op, "ls-tree", "HEAD", "opendbc_repo")
    d["submodule_recorded"] = pin.split()[2] if pin and len(pin.split()) > 2 else None
    d["submodule_matches_release"] = d["submodule_recorded"] == OPENDBC_SHA
    return d


def audit(args):
    root = Path(args.data_root).resolve()
    data = repo_manifest(root)
    print("NAP release check (openpilot v1.0.1, read-only)")
    for label in REPOS:
        r = data[label]
        if not r["present"]:
            print(f"  {label}: NOT PRESENT")
        else:
            print(f"  {label}: {str(r['commit'])[:12]}  release={'yes' if r['matches_release'] else 'NO'}  dirty={'YES' if r['dirty'] else 'no'}  branch={r['branch']}")
    print(f"  openpilot opendbc gitlink: {data['submodule_recorded']}  expected={'yes' if data['submodule_matches_release'] else 'NO'}")
    for p in (root / "nap_settings", root / "bms/nap_dash/nap_history.sqlite3", root / "continue.sh", root / "server/phone_navigation"):
        print(f"  runtime {p}: {'present' if p.exists() else 'missing'}")
    print("  Note: this does not verify vehicle readiness, service autostart or runtime control behavior.")
    if args.remote:
        print("Remote GitHub branch/tag checks (read-only):")
        for label, (_, expected, url, ref) in REPOS.items():
            code, out, err = shell(["git", "ls-remote", url, f"refs/heads/main", f"refs/tags/{ref}^{{}}", f"refs/tags/{ref}"], timeout=45)
            hits = [line.split() for line in out.splitlines() if line.strip()]
            present = any(hash_ == expected for hash_, _ in hits)
            print(f"  {label}: release commit visible={'yes' if present else 'NO'} ({'network error' if code else 'remote reached'})")
            if code: print(f"    remote check unavailable: {err[:120]}")


def safe_copy_file(source: Path, target: Path):
    if source.is_symlink() or not source.is_file():
        return False
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target, follow_symlinks=False)
    os.chmod(target, 0o600)
    return True


def safe_copy_tree(source: Path, target: Path, *, max_file_bytes=100_000_000):
    if not source.is_dir() or source.is_symlink():
        return 0
    count = 0
    for p in source.rglob("*"):
        if p.is_symlink() or not p.is_file():
            continue
        if p.stat().st_size > max_file_bytes:
            print(f"SKIP oversized file: {p.relative_to(source)}", file=sys.stderr)
            continue
        if safe_copy_file(p, target / p.relative_to(source)):
            count += 1
    return count


def sha256(path: Path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def backup(args):
    root = Path(args.data_root).resolve()
    outdir = Path(args.output).expanduser().resolve()
    # Never save backups inside source repos and risk accidentally committing private data.
    if any(outdir == root / rel or (root / rel) in outdir.parents for rel, *_ in REPOS.values()):
        raise RuntimeError("Backup destination must be outside all source Git repositories")
    old_umask = os.umask(0o077)
    try:
        outdir.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(outdir, 0o700)
        stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        archive = outdir / f"nap-private-recovery-{stamp}.tar.gz"
        if archive.exists():
            raise RuntimeError(f"Refusing to overwrite: {archive}")
        with tempfile.TemporaryDirectory(prefix=".nap-build-", dir=outdir) as tmp:
            temp = Path(tmp)
            content = temp / "payload"
            content.mkdir()
            count = 0
            for rel in CONFIG_PATHS:
                p = root / rel
                if p.is_file() and safe_copy_file(p, content / rel): count += 1
                elif p.is_dir(): count += safe_copy_tree(p, content / rel, max_file_bytes=10_000_000)
            if args.include_params:
                count += safe_copy_tree(root / "params", content / "params", max_file_bytes=20_000_000)
            else:
                print("Note: /data/params not included; use --include-params for a PRIVATE parameter snapshot.")
            bms = root / BMS_PATH
            db = bms / "nap_history.sqlite3"
            if db.exists() and not db.is_symlink():
                target = content / BMS_PATH / db.name
                target.parent.mkdir(parents=True, exist_ok=True)
                # Online SQLite backup incorporates committed WAL state coherently.
                uri = "file:" + str(db) + "?mode=ro"
                with closing(sqlite3.connect(uri, uri=True, timeout=15)) as source:
                    with closing(sqlite3.connect(str(target), timeout=15)) as dest:
                        source.backup(dest, pages=256, sleep=0.05)
                os.chmod(target, 0o600)
                count += 1
                for p in bms.iterdir():
                    if p.is_file() and p.suffix in BMS_LOG_SUFFIXES:
                        if safe_copy_file(p, content / BMS_PATH / p.name): count += 1
            manifest = {
                "format": "NAP private recovery 1.0", "created_utc": stamp,
                "repos": repo_manifest(root), "include_params": args.include_params,
                "warning": "Contains private vehicle settings/history. Never publish or commit to GitHub. Restore manually after review.",
                "files": {},
            }
            for p in sorted(content.rglob("*")):
                if p.is_file():
                    manifest["files"][str(p.relative_to(content))] = {"sha256": sha256(p), "size": p.stat().st_size}
            (content / "NAP_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n")
            staging_archive = temp / archive.name
            with tarfile.open(staging_archive, "w:gz", format=tarfile.PAX_FORMAT) as tar:
                for p in content.rglob("*"):
                    if p.is_file(): tar.add(p, arcname=str(p.relative_to(content)), recursive=False)
            os.chmod(staging_archive, 0o600)
            staging_archive.replace(archive)
        digest = sha256(archive)
        sums = Path(str(archive) + ".sha256")
        sums.write_text(f"{digest}  {archive.name}\n")
        os.chmod(sums, 0o600)
        print(f"PRIVATE RECOVERY ARCHIVE: {archive}")
        print(f"SHA256 FILE: {sums}")
        print(f"DATA FILES: {count}; verified in-archive manifest will be checked by verify")
        print("IMPORTANT: copy this archive OFF the comma to encrypted storage. An on-device backup does not survive device failure.")
    finally:
        os.umask(old_umask)


def verify(args):
    archive = Path(args.archive).expanduser().resolve()
    if not archive.is_file(): raise RuntimeError(f"Missing archive: {archive}")
    check = Path(str(archive) + ".sha256")
    if check.exists():
        expected = check.read_text().strip().split()[0]
        if sha256(archive) != expected: raise RuntimeError("ARCHIVE SHA256 MISMATCH")
    with tarfile.open(archive, "r:gz") as tar:
        members = tar.getmembers()
        names = {m.name for m in members}
        for m in members:
            if not m.isfile() or m.name.startswith("/") or ".." in Path(m.name).parts:
                raise RuntimeError(f"Unexpected archive member: {m.name}")
        mf = tar.extractfile("NAP_MANIFEST.json")
        if mf is None: raise RuntimeError("Manifest not found")
        manifest = json.load(mf)
        for name, meta in manifest["files"].items():
            if name not in names: raise RuntimeError(f"File missing: {name}")
            h = hashlib.sha256()
            stream = tar.extractfile(name)
            assert stream is not None
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                h.update(chunk)
            if h.hexdigest() != meta["sha256"]: raise RuntimeError(f"File checksum mismatch: {name}")
    print(f"VERIFY OK: {archive.name}; {len(manifest['files'])} payload files")
    print("No files extracted or restored. This archive is private.")


def stage(args):
    dest = Path(args.destination).expanduser().resolve()
    root = Path(args.data_root).resolve()
    # Guard against touching the known live tree.
    live_dirs = [root / "openpilot", root / "server", root / "openpilot/opendbc_repo"]
    if any(dest == p or p in dest.parents or dest in p.parents for p in live_dirs):
        raise RuntimeError("Stage destination overlaps the live Git repositories")
    if dest.exists():
        raise RuntimeError(f"Destination already exists, refusing to replace: {dest}")
    if shutil.which("git") is None: raise RuntimeError("git is not installed")
    # Stage sources in a new location, with exact known commits and no activation.
    print("This downloads source only. No build, installation, startup or vehicle settings are changed.")
    dest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".nap-stage-", dir=dest.parent) as temp_name:
        temp = Path(temp_name)
        for label, (_, expected, url, ref) in REPOS.items():
            path = temp / label
            selected_ref = ref
            cmd = ["git", "clone", "--quiet", "--depth", "1", "--branch", selected_ref, "--", url, str(path)]
            print(f"Cloning {label} at {selected_ref} (source only)...", flush=True)
            code, _, err = shell(cmd, timeout=450)
            if code: raise RuntimeError(f"Clone failed for {label}: {err[-500:]}")
            commit = git_value(path, "rev-parse", "HEAD")
            if commit != expected:
                raise RuntimeError(f"{label} version mismatch: {commit} (wanted {expected}); STOP and inspect release refs")
        pin = git_value(temp / "openpilot", "ls-tree", "HEAD", "opendbc_repo")
        if not pin or OPENDBC_SHA not in pin:
            raise RuntimeError("Openpilot Git submodule pointer does not match the expected opendbc release")
        # Source-only: do NOT submodule update or replace /data/openpilot.
        (temp / "STAGED_README.txt").write_text(
            "NAP staged source ONLY. NOT an installed system.\n"
            "Do not run launch scripts from this directory or copy into live /data/openpilot.\n"
            "Other submodules, Python dependencies, safety validation, service startup,\n"
            "hardware-specific settings and calibration still need independent verification.\n"
        )
        temp.rename(dest)
    print(f"STAGE VERIFIED: {dest}")
    print("Release hashes match. Live driving software and MCU2 server untouched.")



def startup(args):
    """Read-only inventory of deployment dependencies and launch entrypoints."""
    root = Path(args.data_root).resolve()
    print("NAP startup / external dependency inventory (read-only)")
    for rel in ["continue.sh", "openpilot/launch_openpilot.sh", "openpilot/launch_chffrplus.sh",
                "server/server_v21.py", "server/phone_navigation/__init__.py",
                "server/tailscaled_daemon.py", "server/cloudflared_tunnel.py",
                "server/ffmpeg_full", "server/phone_navigation-deployment.json",
                "nap_settings/v9e_predictive_grade.json", "params/d"]:
        p = root / rel
        print(f"  {rel}: {'present' if p.exists() else 'MISSING'}")
    keywords = ["server_v21", "mpp_writer", "cloudflared", "tailscale", "phone_navigation", "continue.sh"]
    print("Startup-file keyword locations (contents intentionally hidden):")
    for rel in ["continue.sh", "openpilot/launch_openpilot.sh", "openpilot/launch_chffrplus.sh"]:
        p = root / rel
        if p.is_file() and not p.is_symlink():
            try:
                lines = p.read_text(errors="replace").splitlines()
                for i, line in enumerate(lines, 1):
                    matches = [k for k in keywords if k in line.lower()]
                    if matches:
                        print(f"  {rel}:{i}: {', '.join(matches)}")
            except OSError:
                print(f"  {rel}: cannot read")
    print("Related service unit names (not their private configuration):")
    unit_dirs = [Path('/etc/systemd/system'), Path('/lib/systemd/system')]
    for d in unit_dirs:
        if d.is_dir():
            for p in sorted(d.glob('*.service')):
                if any(k in p.name.lower() for k in ('nap', 'mpp', 'server', 'cloudflared', 'tailscale', 'navigation')):
                    print(f"  {p.name}")
    print("External dashboard services may need environment-specific restoration.")
    print("No service was started, stopped, enabled or modified.")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data-root", default="/data", help="Runtime root, default /data")
    sub = p.add_subparsers(dest="operation", required=True)
    a = sub.add_parser("audit", help="Read-only local release + runtime inventory")
    a.add_argument("--remote", action="store_true", help="Also check GitHub (network)")
    sub.add_parser("startup", help="Read-only startup/service dependency inventory")
    b = sub.add_parser("backup", help="Create user-only private settings/BMS recovery archive")
    b.add_argument("--output", default="/data/nap_recovery", help="Outside Git repos")
    b.add_argument("--include-params", action="store_true", help="Also include private /data/params (may contain tokens)")
    v = sub.add_parser("verify", help="Verify private archive without extracting")
    v.add_argument("archive")
    s = sub.add_parser("stage", help="Stage pinned GitHub release source; never activate")
    s.add_argument("--destination", default="/data/nap_staging/NAP-v1.0.2")
    args = p.parse_args()
    try:
        return {"audit": audit, "startup": startup, "backup": backup, "verify": verify, "stage": stage}[args.operation](args) or 0
    except (OSError, ValueError, RuntimeError, sqlite3.Error, tarfile.TarError) as e:
        print(f"STOP: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
