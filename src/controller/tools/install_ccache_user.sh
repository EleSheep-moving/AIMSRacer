#!/usr/bin/env bash
# Optional Ubuntu fallback when sudo is unavailable. No Python environment.
set -euo pipefail
tools_prefix="$HOME/.local/lib/aims-mpcc-tools"
tools_bin="$HOME/.local/bin"
if command -v ccache >/dev/null 2>&1; then
    ccache --version
    exit 0
fi
if [[ -x "$tools_bin/ccache" ]]; then
    "$tools_bin/ccache" --version
    exit 0
fi
download_dir=$(mktemp -d)
trap 'rm -rf "$download_dir"' EXIT
cd "$download_dir"
apt download ccache libhiredis0.14
mkdir -p "$tools_prefix" "$tools_bin"
for package in ./*.deb; do
    dpkg-deb -x "$package" "$tools_prefix"
done
arch_triplet=$(dpkg-architecture -qDEB_HOST_MULTIARCH)
/usr/bin/python3 - "$tools_prefix" "$tools_bin" "$arch_triplet" <<'PY'
from pathlib import Path
import shlex
import sys
prefix, bin_path, arch = sys.argv[1:]
wrapper = Path(bin_path) / 'ccache'
with wrapper.open('x') as output:
    output.write('#!/bin/sh\nexport LD_LIBRARY_PATH=' +
                 shlex.quote(str(Path(prefix) / 'usr/lib' / arch)) +
                 '${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\nexec ' +
                 shlex.quote(str(Path(prefix) / 'usr/bin/ccache')) + ' "$@"\n')
wrapper.chmod(0o755)
PY
"$tools_bin/ccache" --version
