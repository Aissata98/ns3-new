#!/usr/bin/env python3
"""Reproduce frozen scientific summaries offline after relocating their archive.

Mappings change filesystem lookup only. Immutable JSON, logical job paths,
hashes, selection rules and statistical calculations are never rewritten.
No simulator, shell, Docker, live gate or network operation is allowed.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import os
from pathlib import Path as NativePath
import subprocess
import sys


MODULES = ("closure_protocol", "hardware_preflight", "closure_campaign_worker",
           "closure_campaign_audit", "run_closure_campaign",
           "select_campaign_representatives", "summarize_final_campaign")
PATH_BASE = type(NativePath())


def digest(path):
    result = hashlib.sha256()
    with NativePath(path).open("rb") as stream:
        for block in iter(lambda:stream.read(1024*1024), b""):
            result.update(block)
    return result.hexdigest()


def contained(path, root):
    return path == root or root in path.parents


def absolute(path):
    path = NativePath(path)
    if not path.is_absolute() or ".." in path.parts:
        raise ValueError("Explicit absolute paths without '..' are required")
    return NativePath(os.path.abspath(path))


class Relocation:
    def __init__(self, pairs, code_root, output):
        self.pairs = []
        self.code_root = absolute(code_root).resolve(strict=True)
        self.output = absolute(output)
        self.lookups = {"archive_reads":0, "code_reads":0, "output_accesses":0}
        for value in pairs:
            if value.count("=") != 1:
                raise ValueError("Each relocation must be ORIGINAL=ACTUAL")
            original, actual = (absolute(part) for part in value.split("=",1))
            if len(original.parts) < 3 or len(actual.parts) < 3 or not actual.is_dir():
                raise ValueError("Relocation roots must be scoped directories")
            if actual.resolve(strict=True) != actual:
                raise ValueError("Archive root must be an actual directory, not a symlink alias")
            for old, new in self.pairs:
                if any((contained(original,old),contained(old,original),contained(actual,new),contained(new,actual))):
                    raise ValueError("Overlapping or ambiguous relocation roots")
            if contained(self.output,original) or contained(self.output,actual):
                raise ValueError("Output must be outside all immutable archive roots")
            if contained(self.code_root,actual) or contained(actual,self.code_root):
                raise ValueError("Frozen analytical code must be separate from relocated evidence")
            self.pairs.append((original,actual))
        if not self.pairs or self.output.exists() or not self.output.parent.is_dir():
            raise ValueError("Provide mappings and a new output in an existing directory")
        if contained(self.output,self.code_root):
            raise ValueError("Output cannot modify frozen analysis code")

    def logical(self, value):
        path = NativePath(value)
        if path.is_absolute():
            for original,actual in self.pairs:
                if contained(path,actual):
                    return original/path.relative_to(actual)
        return path

    def lookup(self, value, *, writing=False):
        path = absolute(value)
        if writing:
            if path != self.output:
                raise PermissionError("Offline analysis may write only its new declared summary")
            self.lookups["output_accesses"] += 1
            return self.output
        for original,actual in self.pairs:
            if contained(path,original):
                target = actual/path.relative_to(original)
                if not contained(target.resolve(strict=False),actual):
                    raise PermissionError("Archive path resolves outside its explicit mapping")
                self.lookups["archive_reads"] += 1
                return target  # Missing mapped evidence fails; never consult original.
        if contained(path,self.code_root):
            if not contained(path.resolve(strict=False),self.code_root):
                raise PermissionError("Analytical source escapes the frozen code directory")
            self.lookups["code_reads"] += 1
            return path
        if path == self.output:
            self.lookups["output_accesses"] += 1
            return path
        raise PermissionError("Unmapped filesystem lookup refused: " + str(path))

    def path_class(self):
        relocation = self

        class LookupPath(PATH_BASE):
            def __new__(cls, *parts):
                path = relocation.logical(NativePath(*parts))
                return super().__new__(cls, path)

            def resolve(self, strict=False):
                # Resolve the logical name, not the physical relocation. The
                # logical name is part of original union/provenance evidence.
                result = type(self)(os.path.abspath(str(self)))
                if strict:
                    relocation.lookup(result).stat()
                return result

            def open(self, mode="r", *args, **kwargs):
                writing = any(flag in mode for flag in "wax+")
                if writing and ("x" not in mode or any(flag in mode for flag in "wa+")):
                    raise PermissionError("Only exclusive creation of the declared output is permitted")
                return relocation.lookup(self,writing=writing).open(mode,*args,**kwargs)

            def stat(self, *args, **kwargs):
                return relocation.lookup(self).stat(*args,**kwargs)

            def lstat(self, *args, **kwargs):
                return relocation.lookup(self).lstat(*args,**kwargs)

            def is_symlink(self):
                return relocation.lookup(self).is_symlink()

            def write_text(self, *args, **kwargs):
                raise PermissionError("Archive rewriting is forbidden")

            def write_bytes(self, *args, **kwargs):
                raise PermissionError("Archive rewriting is forbidden")

        return LookupPath


def verified_code(code_root):
    code_root = absolute(code_root).resolve(strict=True)
    manifest = json.loads((code_root/"CODE_SHA256.json").read_text())
    expected = {"sim/"+name+".py" for name in MODULES[:5]} | {
        "finalization/"+name+".py" for name in MODULES[5:]}
    if manifest.get("schema") != "closure-offline-code-v1" or set(manifest.get("files",{})) != expected:
        raise ValueError("Incomplete frozen offline analysis package")
    for relative, expected_hash in manifest["files"].items():
        path=code_root/relative
        if path.is_symlink() or digest(path) != expected_hash:
            raise ValueError("Frozen analytical source changed: "+relative)
    return manifest


@contextmanager
def offline_modules(code_root, relocation):
    """Patch only this offline process; never edit or import active host scripts."""
    saved_modules = {name:sys.modules.pop(name,None) for name in MODULES}
    saved_path = list(sys.path)
    saved_bytecode = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    code_root = NativePath(code_root)
    sys.path[:0] = [str(code_root/"finalization"),str(code_root/"sim")]
    patched=[]
    original_popen,original_run=subprocess.Popen,subprocess.run
    try:
        spec=importlib.util.spec_from_file_location("summarize_final_campaign",code_root/"finalization/summarize_final_campaign.py")
        module=importlib.util.module_from_spec(spec)
        sys.modules[spec.name]=module
        spec.loader.exec_module(module)
        lookup_path=relocation.path_class()
        for name in MODULES:
            imported=sys.modules[name]
            if hasattr(imported,"Path"):
                patched.append((imported,imported.Path))
                imported.Path=lookup_path
        def no_process(*args,**kwargs):
            raise PermissionError("Offline reproduction cannot start processes or simulations")
        subprocess.Popen=subprocess.run=no_process
        yield module
    finally:
        subprocess.Popen,subprocess.run=original_popen,original_run
        for module,path in patched:
            module.Path=path
        sys.path[:]=saved_path
        sys.dont_write_bytecode = saved_bytecode
        for name,saved in saved_modules.items():
            sys.modules.pop(name,None)
            if saved is not None:
                sys.modules[name]=saved


def reproduce(code_root, mappings, campaigns, binding_hashes, selection, output, old_summary=None):
    if len(campaigns) != len(binding_hashes):
        raise ValueError("One explicit binding hash is required per phase")
    code_root=absolute(code_root)
    code_manifest=verified_code(code_root)
    relocation=Relocation(mappings,code_root,output)
    argv=[]
    for campaign,binding_hash in zip(campaigns,binding_hashes):
        argv += ["--campaign",str(absolute(campaign)),"--binding-sha256",binding_hash]
    argv += ["--selection",str(absolute(selection)),"--output",str(absolute(output))]
    if old_summary:
        argv += ["--old-summary",str(absolute(old_summary))]
    with offline_modules(code_root,relocation) as summary:
        status=summary.main(argv)
    return {"status":"passed" if status==0 else "failed","network_runs_started":0,
        "immutable_evidence_rewritten":False,"original_path_fallback":False,
        "lookup_counts":relocation.lookups,"frozen_code":code_manifest,
        "output":str(absolute(output)),"output_sha256":digest(output)}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--code-root",type=NativePath,default=NativePath(__file__).resolve().parent/"frozen")
    parser.add_argument("--relocate",action="append",required=True,metavar="ORIGINAL=ACTUAL")
    parser.add_argument("--campaign",action="append",required=True,type=NativePath)
    parser.add_argument("--binding-sha256",action="append",required=True)
    parser.add_argument("--selection",required=True,type=NativePath)
    parser.add_argument("--output",required=True,type=NativePath)
    parser.add_argument("--old-summary",type=NativePath)
    args=parser.parse_args(argv)
    result=reproduce(args.code_root,args.relocate,args.campaign,args.binding_sha256,args.selection,args.output,args.old_summary)
    print(json.dumps(result,sort_keys=True))
    return 0


if __name__=="__main__":
    try:
        raise SystemExit(main())
    except (ValueError,KeyError,TypeError,OSError) as error:
        print("OFFLINE_REPRODUCTION_BLOCKED: "+str(error),file=sys.stderr)
        raise SystemExit(2)
