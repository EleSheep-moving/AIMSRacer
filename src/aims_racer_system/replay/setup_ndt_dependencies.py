#!/usr/bin/env python3
"""Prepare exact NDT sources and audited patches in an isolated dependency directory."""
import argparse
import hashlib
import subprocess
from pathlib import Path
import yaml


def run(*args,cwd=None,capture=False):
    return subprocess.run(args,cwd=cwd,check=True,text=True,
                          stdout=subprocess.PIPE if capture else None).stdout


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    here=Path(__file__).resolve().parent
    manifest=yaml.safe_load((here/'dependencies.yaml').read_text())
    args.directory.mkdir(parents=True,exist_ok=True)
    for name,spec in manifest.items():
        target=args.directory/name
        if not target.exists():
            run('git','clone',spec['url'],str(target))
            run('git','checkout','--detach',spec['commit'],cwd=target)
        observed=run('git','rev-parse','HEAD',cwd=target,capture=True).strip()
        if observed!=spec['commit']:
            raise SystemExit(f'{target}: revision differs; refusing to replace existing source')
        patch=here/spec['patch']
        if hashlib.sha256(patch.read_bytes()).hexdigest()!=spec['patch_sha256']:
            raise SystemExit(f'{name}: patch checksum differs')
        check=subprocess.run(['git','apply','--check',str(patch)],cwd=target,capture_output=True)
        if check.returncode==0:
            if run('git','status','--porcelain',cwd=target,capture=True).strip():
                raise SystemExit(f'{target}: dirty before patch; refusing to modify')
            run('git','apply',str(patch),cwd=target)
        else:
            run('git','apply','--reverse','--check',str(patch),cwd=target)
        # Mark new patch files for diff only, then verify the COMPLETE source delta.
        # Intent-to-add does not commit dependency changes or stage their contents.
        import re
        added=re.findall(r'^diff --git a/(.+) b/\1\nnew file mode',patch.read_text(),re.MULTILINE)
        if added:
            run('git','add','-N','--',*added,cwd=target)
        observed_patch=run('git','diff','HEAD','--binary',cwd=target,capture=True)
        if observed_patch!=patch.read_text():
            raise SystemExit(f'{target}: source delta does not match audited patch')
        if run('git','ls-files','--others','--exclude-standard',cwd=target,capture=True).strip():
            raise SystemExit(f'{target}: unexpected untracked source')
        print(f'{name}: {observed} + verified {patch.name}')


if __name__=='__main__':
    main()
