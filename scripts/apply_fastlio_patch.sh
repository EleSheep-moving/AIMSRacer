#!/usr/bin/env bash
# Preserve the upstream gitlink; apply the versioned AIMSRacer TF change locally.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
fastlio_dir="$repo_root/src/FASTLIO2_ROS2"
patch_file="$repo_root/patches/fastlio2-publish-tf.patch"
expected_revision=f516daac08bc46e50e814a2e7d6c8352ed8141bb
if [[ ! -f "$fastlio_dir/fastlio2/src/lio_node.cpp" ]]; then
    echo "Initialize first: git submodule update --init src/FASTLIO2_ROS2" >&2
    exit 1
fi
actual_revision="$(git -C "$fastlio_dir" rev-parse HEAD)"
if [[ "$actual_revision" != "$expected_revision" ]]; then
    echo "FAST-LIO revision differs from the reviewed base: $actual_revision" >&2
    exit 1
fi
if git -C "$fastlio_dir" apply --reverse --check "$patch_file" 2>/dev/null; then
    echo "FAST-LIO publish_tf patch is already applied."
elif git -C "$fastlio_dir" apply --check "$patch_file"; then
    git -C "$fastlio_dir" apply "$patch_file"
    echo "Applied FAST-LIO publish_tf patch; rebuild fastlio2 before bringup."
else
    echo "Patch conflicts with local changes; no files were changed." >&2
    exit 1
fi
