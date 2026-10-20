#!/usr/bin/env python3
"""NAP v1 recovery package validator and SAFE, ISOLATED restore rehearsal.

No live installs, overwrites, service changes, Tailscale login, code execution from
archives, or vehicle-control activation. Standard-library Python >= 3.10.
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile

RELEASES = {
    "openpilot": ("https://github.com/philipr144-create/openpilot-preap.git", "NAP-v1.0.2", "fb41fc71b4266e0a92a4550c51c2bfb1ef2a7e91"),
    "opendbc": ("https://github.com/philipr144-create/opendbc-preap.git", "NAP-opendbc-v1.0.0", "31da4137f2ce06702bc595a05689d537f046bb70"),
    "NAP-Dash": ("https://github.com/philipr144-create/NAP-Dash.git", "NAP-Dash-v1.0.1", "a9eb509e96a2555ecf979149d8121ad72430695e"),
}
RUNTIME_SHA = {
    "tailscale": "a14b94589c2630eb68ba7f7651ede226d2976708760ef3460556a00cf1aa4bab",
    "tailscaled": "dda710b5bed9fbf87efc0126b614ed8f0e9f4a43b2265486bc6ad7eb0570f226",
    "ffmpeg": "6bb182d0d75d23028db82e9e4f723ca69b853d055698486e6984ddb2c06fb8ce",
}
RUNTIME_PATH = {
    "tailscale": "data/tailscale/bin/tailscale",
    "tailscaled": "data/tailscale/bin/tailscaled",
    "ffmpeg": "data/ffmpeg-7.0.2-arm64-static/ffmpeg",
}
EXTRA_REQUIRED = {
    "openpilot": [
        "system/manager/process_config.py", "selfdrive/nap/grade_toggle_ui.py",
        "selfdrive/controls/lib/predictive_grade_assist.py", "launch_openpilot.sh",
    ],
    "NAP-Dash": [
        "server_v21.py", "tailscaled_daemon.py",
        "phone_navigation/route_tracking.py", "phone_navigation/desire_bridge.py",
    ],
}
MAX_ARCHIVE_FILE = 300 * 1024 * 1024
MAX_ARCHIVE_TOTAL = 2 * 1024 * 1024 * 1024


class PackageError(Exception):
    pass


def sha256_file(p: Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda: f.read(1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def archive_sidecar(p: Path) -> None:
    if p.is_symlink() or not p.is_file():
        raise PackageError(f"Missing or symlinked archive: {p}")
    sidecar = Path(str(p) + ".sha256")
    if sidecar.is_symlink() or not sidecar.is_file():
        raise PackageError(f"Missing or symlinked SHA256 sidecar for {p.name}")
    # GNU sha256sum records the path used when computing the digest.  The
    # on-device runtime packager records an absolute /data/nap_recovery path,
    # while the Mac restores the same archive under ~/NAP-Recovery.  Compare
    # the archive basename, not its original device-specific directory.
    # Never use the recorded path to open or extract any file.
    fields = sidecar.read_text(encoding="utf-8").strip().split()
    if (len(fields) != 2 or not re.fullmatch(r"[0-9a-f]{64}", fields[0])
            or Path(fields[1].lstrip("*")).name != p.name):
        raise PackageError(f"Malformed or filename-mismatched SHA256 sidecar for {p.name}")
    if sha256_file(p) != fields[0]:
        raise PackageError(f"Archive digest does not match sidecar: {p.name}")


def valid_member_name(name: str) -> str:
    # PAX/tar paths must be relative, non-conflicting, and safe on any OS.
    path = PurePosixPath(name)
    if (not name or "\\" in name or "\x00" in name or name.startswith("/")
        or any(p in ("", ".", "..") for p in name.split("/"))):
        raise PackageError("Unsafe tar member name")
    return str(path)


def read_members(archive: Path) -> dict[str, tarfile.TarInfo]:
    files = {}
    total = 0
    with tarfile.open(archive, "r:gz") as tar:
        for m in tar.getmembers():
            # Directory members are accepted if their names are safe.
            name = valid_member_name(m.name.rstrip("/"))
            if m.isdir():
                continue
            if not m.isfile() or name in files or m.size < 0 or m.size > MAX_ARCHIVE_FILE:
                raise PackageError("Unsafe, repeated, or oversized archive entry")
            total += m.size
            if total > MAX_ARCHIVE_TOTAL:
                raise PackageError("Archive exceeds allowed unpacked size")
            files[name] = m
    return files


def digest_member(tar: tarfile.TarFile, member: tarfile.TarInfo) -> str:
    h = hashlib.sha256()
    stream = tar.extractfile(member)
    if stream is None:
        raise PackageError("Cannot read archive member")
    with stream:
        for b in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(b)
    return h.hexdigest()


def validate_private(archive: Path) -> tuple[dict, dict]:
    archive_sidecar(archive)
    members = read_members(archive)
    if "NAP_MANIFEST.json" not in members:
        raise PackageError("Private archive lacks NAP_MANIFEST.json")
    with tarfile.open(archive, "r:gz") as tar:
        with tar.extractfile(members["NAP_MANIFEST.json"]) as stream:
            manifest = json.load(stream)
        entries = manifest.get("files")
        if not isinstance(entries, dict) or set(members) != set(entries) | {"NAP_MANIFEST.json"}:
            raise PackageError("Private manifest/file membership mismatch")
        for name, meta in entries.items():
            if valid_member_name(name) != name or not isinstance(meta, dict):
                raise PackageError("Invalid private manifest entry")
            if meta.get("size") != members[name].size or digest_member(tar, members[name]) != meta.get("sha256"):
                raise PackageError("Private file size/digest mismatch")
    return manifest, members


def validate_runtime(archive: Path) -> dict:
    archive_sidecar(archive)
    members = read_members(archive)
    required = {f"nap-runtime/bin/{name}" for name in RUNTIME_SHA}
    if not required.issubset(members):
        raise PackageError("Runtime bundle lacks a required executable")
    if set(members) - (required | {"nap-runtime/SHA256SUMS", "nap-runtime/README.txt"}):
        raise PackageError("Unexpected runtime bundle files")
    with tarfile.open(archive, "r:gz") as tar:
        for name, digest in RUNTIME_SHA.items():
            if digest_member(tar, members[f"nap-runtime/bin/{name}"]) != digest:
                raise PackageError(f"Runtime checksum mismatch: {name}")
        mf = members.get("nap-runtime/SHA256SUMS")
        if mf is None:
            raise PackageError("Runtime internal SHA256SUMS missing")
        lines = tar.extractfile(mf).read().decode("utf-8").splitlines()
        sums = {}
        for line in lines:
            fields = line.split()
            if len(fields) != 2 or not re.fullmatch(r"[a-f0-9]{64}", fields[0]):
                raise PackageError("Malformed internal runtime checksums")
            sums[fields[1].lstrip("*")] = fields[0]
        if sums != {f"bin/{k}": v for k, v in RUNTIME_SHA.items()}:
            raise PackageError("Internal runtime checksum list does not match binaries")
    return members


def copy_archive_files(archive: Path, files: dict, dest: Path, rename: dict[str, str] | None = None) -> None:
    rename = rename or {name: name for name in files}
    with tarfile.open(archive, "r:gz") as tar:
        for name, dst_name in rename.items():
            member = files[name]
            target = dest / dst_name
            # No archive-sourced links, and the destination is a newly created tree.
            target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            stream = tar.extractfile(member)
            if stream is None:
                raise PackageError("Archive member could not be extracted")
            with stream, target.open("xb") as out:
                shutil.copyfileobj(stream, out, 1024 * 1024)
            target.chmod(0o600)


def run(cmd: list[str], *, cwd: Path | None = None, timeout=900, env=None) -> str:
    try:
        cp = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout, env=env)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise PackageError(f"Required command failed: {cmd[0]} ({type(exc).__name__})") from exc
    if cp.returncode:
        # Do not display full stdout/stderr that might contain credentials.
        raise PackageError(f"Command failed ({cmd[0]}, exit {cp.returncode}); see terminal or check network/authentication")
    return cp.stdout.strip()


def clone_repo(name: str, dest: Path) -> None:
    url, tag, expected = RELEASES[name]
    print(f"Cloning {name} [{tag}]...", flush=True)
    env = dict(os.environ)
    env.pop("GIT_LFS_SKIP_SMUDGE", None)
    run(["git", "clone", "--quiet", "--depth", "1", "--branch", tag, "--", url, str(dest)], env=env)
    current = run(["git", "-C", str(dest), "rev-parse", "HEAD"])
    if current != expected:
        raise PackageError(f"{name} commit mismatch; release has changed")


def validate_code_layout(data: Path) -> dict:
    op = data / "openpilot"
    db = op / "opendbc_repo"
    dashboard = data / "server"
    for label, root in (("openpilot", op), ("opendbc", db), ("NAP-Dash", dashboard)):
        expected = RELEASES[label][2]
        if run(["git", "-C", str(root), "rev-parse", "HEAD"]) != expected:
            raise PackageError(f"{label}: incorrect commit")
    pinned = run(["git", "-C", str(op), "ls-tree", "HEAD", "opendbc_repo"])
    if not re.match(r"^160000 commit " + RELEASES["opendbc"][2] + r"\s+opendbc_repo$", pinned):
        raise PackageError("Openpilot gitlink does not match opendbc release")
    for label, paths in EXTRA_REQUIRED.items():
        root = op if label == "openpilot" else dashboard
        for path in paths:
            p = root / path
            if not p.is_file():
                raise PackageError(f"Missing essential {label} file: {path}")
            if path.endswith(".py"):
                import ast
                ast.parse(p.read_text(encoding="utf-8"), filename=str(p))
    manager = (op / "system/manager/process_config.py").read_text()
    for module in ("server.server_v21", "server.tailscaled_daemon"):
        if module not in manager:
            raise PackageError("Missing required openpilot manager module registration")
    tail = (dashboard / "tailscaled_daemon.py").read_text()
    if "PUBLIC_URL = _read_private_public_url()" not in tail or "ts.net" in tail:
        raise PackageError("Tailscale private-hostname protection not confirmed")
    if "github.com/philipr144-create/openpilot-preap.git/info/lfs" not in (op / ".lfsconfig").read_text():
        raise PackageError("GitHub LFS endpoint missing from pinned openpilot")
    return {"manager_modules": ["server.server_v21", "server.tailscaled_daemon"], "gitlink": RELEASES["opendbc"][2]}


def validate_lfs(repo: Path) -> int:
    out = run(["git", "-C", str(repo), "lfs", "ls-files", "--long"])
    count = 0
    for line in out.splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})\s+[-*]\s+(.+)", line)
        if match is None:
            raise PackageError("Unrecognized git-lfs inventory entry")
        digest, rel = match.groups()
        p = (repo / rel).resolve()
        if not p.is_relative_to(repo.resolve()) or not p.is_file() or sha256_file(p) != digest:
            raise PackageError("One or more LFS files are missing/corrupt")
        count += 1
    if count != 251:
        raise PackageError(f"Unexpected NAP-v1.0.2 LFS file count: {count}")
    return count


def refuse_unsafe_destination(dest: Path) -> None:
    dest = dest.expanduser().resolve()
    if dest == Path("/") or dest == Path("/data"):
        raise PackageError("Destination must be a fresh isolated directory")
    # Never write within active comma production trees, even if a user mistypes.
    for live in ("/data/openpilot", "/data/server", "/data/params", "/data/nap_settings", "/data/tailscale", "/usr", "/etc"):
        root = Path(live)
        if dest == root or root in dest.parents or dest in root.parents:
            raise PackageError("Destination overlaps protected production paths")
    if dest.exists() or dest.is_symlink():
        raise PackageError("Destination exists; refusing to overwrite")


def check(args) -> None:
    private = Path(args.private).expanduser().resolve()
    runtime = Path(args.runtime).expanduser().resolve()
    pm, _ = validate_private(private)
    validate_runtime(runtime)
    print(f"PASS: Private archive and manifest verified ({len(pm['files'])} payload files)")
    print("PASS: ARM64 runtime bundle and SHA256 manifest verified (3 executables)")
    print("No files extracted; no services or configuration changed.")


def prepare(args) -> None:
    private = Path(args.private).expanduser().resolve()
    runtime = Path(args.runtime).expanduser().resolve()
    dest = Path(args.destination).expanduser().resolve()
    refuse_unsafe_destination(dest)
    check(args)
    for cmd in ("git", "git-lfs"):
        if not shutil.which(cmd):
            raise PackageError(f"{cmd} not installed; cannot perform fresh-source rehearsal")
    dest.parent.mkdir(parents=True, exist_ok=True)
    old_umask = os.umask(0o077)
    try:
        with tempfile.TemporaryDirectory(prefix=".nap-preview-", dir=dest.parent) as tmp:
            preview = Path(tmp)
            data = preview / "data"
            data.mkdir(mode=0o700)
            clone_repo("openpilot", data / "openpilot")
            clone_repo("opendbc", data / "openpilot" / "opendbc_repo")
            clone_repo("NAP-Dash", data / "server")
            validated = validate_code_layout(data)
            print("Checking all 251 Git LFS file checksums...", flush=True)
            lfs_count = validate_lfs(data / "openpilot")
            manifest, private_files = validate_private(private)
            print("Extracting private data into review-only folder (NOT live /data)...", flush=True)
            copy_archive_files(private, private_files, preview / "PRIVATE_REVIEW")
            runtime_files = validate_runtime(runtime)
            renames = {f"nap-runtime/bin/{k}": v for k, v in RUNTIME_PATH.items()}
            copy_archive_files(runtime, runtime_files, preview, renames)
            for rel in RUNTIME_PATH.values():
                (preview / rel).chmod(0o755)
            for k, path in RUNTIME_PATH.items():
                if sha256_file(preview / path) != RUNTIME_SHA[k]:
                    raise PackageError("Copied runtime binary mismatch")
            status = {
                "format": "NAP guarded installation preview 1.0",
                "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
                "sources": {k: {"release": v[1], "sha": v[2]} for k, v in RELEASES.items()},
                "lfs_files_verified": lfs_count,
                "private_payload_files": len(manifest["files"]),
                "runtime_sha256": RUNTIME_SHA,
                "source_layout": validated,
                "warning": "Not installed or commissioned; private values held under PRIVATE_REVIEW. Vehicle control NOT enabled.",
            }
            (preview / "REHEARSAL_REPORT.json").write_text(json.dumps(status, indent=2) + "\n")
            (preview / "READ_BEFORE_ANY_INSTALL.txt").write_text(
                "NAP v1.0.1 isolated restoration rehearsal. NOT an installed or road-validated system.\n"
                "data/openpilot: exact public release source; opendbc_repo: pinned custom fork.\n"
                "data/server: NAP-Dash; data/tailscale/bin: CLI and daemon; data/ffmpeg-*: binary.\n"
                "PRIVATE_REVIEW contains secrets, parameters and settings. Never publish it.\n"
                "Do NOT copy PRIVATE_REVIEW/params into a live vehicle without evaluating\n"
                "calibration, hardware, and authorization for that device.\n"
                "Missing/unverified: other submodules and build artifacts, Python dependencies,\n"
                "OS/bootstrap compatibility, FFmpeg PATH link, Tailscale reauthentication,\n"
                "device-specific safety checks and first-run validation. No auto-activation.\n"
            )
            # TemporaryDirectory would otherwise delete our promoted tree. We move
            # the freshly verified contents to a new destination atomically.
            preview.rename(dest)
            # Renamed dir no longer exists at tmp, cleanup is safe.
        print(f"PASS: Isolated NAP recovery preview ready: {dest}")
        print("PASS: 3 source hashes, submodule gitlink, 251 LFS assets, 3 runtime binaries verified")
        print("PRIVATE_REVIEW stays private and separated from executable deployment layout")
        print("No live installation, reboot, service changes, or vehicle control activation.")
    finally:
        os.umask(old_umask)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for cmd in ("check", "prepare"):
        p = sub.add_parser(cmd)
        p.add_argument("--private", required=True, help="Exact private NAP tar.gz archive")
        p.add_argument("--runtime", required=True, help="Exact ARM64 runtime tar.gz archive")
        if cmd == "prepare":
            p.add_argument("--destination", required=True, help="NEW isolated preview directory, never /data")
    args = parser.parse_args(argv)
    try:
        {"check": check, "prepare": prepare}[args.command](args)
        return 0
    except (PackageError, OSError, ValueError, KeyError, json.JSONDecodeError, tarfile.TarError, SyntaxError) as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
