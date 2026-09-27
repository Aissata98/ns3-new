#!/usr/bin/env python3
"""Optional cached build; plan is offline, inspect is read-only, build is explicit."""
import argparse
import json
import os
import re
from pathlib import Path
import subprocess
import sys
import uuid

import replay

ROOT = Path(__file__).resolve().parent


def identity(root=ROOT):
    return json.loads((root / 'LOCAL_BASE_IDENTITY.json').read_text())


def inspect_base(expected):
    items = json.loads(subprocess.check_output(
        ['docker', 'image', 'inspect', expected['image_tag']], text=True))
    if len(items) != 1:
        raise ValueError('Expected exactly one locally available base image')
    item = items[0]
    if any(item.get(field) != expected[key] for field, key in
           [('Id', 'image_id'), ('Architecture', 'architecture'), ('Os', 'os')]):
        raise ValueError('Local base identity differs; do not pull or substitute a new base')
    return {key: item[key] for key in ('Id', 'Architecture', 'Os')}


def build_commands(image, name, root=ROOT):
    if not re.fullmatch(r'[a-z0-9][a-z0-9._/-]*:[A-Za-z0-9][A-Za-z0-9_.-]*', image):
        raise ValueError('Use an explicit local repository:tag')
    expected = identity(root)
    if image == expected['image_tag']:
        raise ValueError('Never overwrite the dependency base tag')
    if not re.fullmatch(r'reviewed-v6-build-[a-z0-9-]+', name):
        raise ValueError('A dedicated build-container name is required')
    return {
        'create': ['docker', 'create', '--name', name, '--pull=never', '--network=none',
                   '--cpus=2', '--memory=4g', '--memory-swap=4g', '--pids-limit=256',
                   '--entrypoint=/bin/sleep', expected['image_id'], 'infinity'],
        'compile': ['docker', 'exec', name, 'bash', '/artifact/local_build_driver.sh'],
        'commit': ['docker', 'commit', '--change', 'ENTRYPOINT ["python3", "/artifact/replay.py"]',
                   '--change', 'CMD ["--help"]', '--change', 'WORKDIR /output',
                   '--change', 'LABEL org.reviewed-v6.local-base-id=' + expected['image_id'],
                   name, image]}


def build(image, receipt_dir, timeout_s, root=ROOT):
    if not 1 <= timeout_s <= 3600:
        raise ValueError('Build wall deadline must be 1..3600 seconds')
    receipt_dir = receipt_dir.resolve()
    if receipt_dir == root.resolve() or root.resolve() in receipt_dir.parents:
        raise ValueError('Build receipts must be outside the immutable source package')
    expected = identity(root)
    checked_base = inspect_base(expected)
    name = 'reviewed-v6-build-' + uuid.uuid4().hex[:16]
    commands = build_commands(image, name, root)
    # Never overwrite an existing output image tag.
    probe = subprocess.run(['docker', 'image', 'inspect', image], capture_output=True, text=True)
    if probe.returncode == 0:
        raise ValueError('Output image already exists; choose a new local tag')
    receipt_dir.mkdir(parents=False, exist_ok=False)
    intent = {'schema': 'reviewed-v6-local-build-intent-v1', 'created_at_utc': replay.now(),
              'base_checked': checked_base, 'container': name, 'commands': commands,
              'wall_timeout_s': timeout_s, 'cpu_limit': 2,
              'memory_and_swap_limit_bytes': 4 * 1024**3, 'network_runs_executed': 0,
              'historical_receipts_unchanged': True}
    replay.write_new(receipt_dir / 'BUILD_LOCAL_INTENT.json', intent)
    created = False
    result = dict(intent, status='failed', container_retained=True)
    try:
        subprocess.run(commands['create'], check=True)
        created = True
        subprocess.run(['docker', 'start', name], check=True)
        subprocess.run(['docker', 'exec', name, 'mkdir', '-p', '/artifact', '/output'], check=True)
        for filename in ('model', 'replay.py', 'local_build.py', 'local_build_driver.sh',
                         'MODEL_MANIFEST.json', 'LOCAL_BASE_IDENTITY.json', 'LICENSE', 'NOTICE.md'):
            subprocess.run(['docker', 'cp', str(root / filename), name + ':/artifact/' + filename], check=True)
        with (receipt_dir / 'build.stdout.log').open('xb') as stdout, (receipt_dir / 'build.stderr.log').open('xb') as stderr:
            subprocess.run(commands['compile'], check=True, timeout=timeout_s, stdout=stdout, stderr=stderr)
        for filename in ('BUILD_REPRODUCED.json', 'LOCAL_DEPENDENCY_CHECK.json'):
            subprocess.run(['docker', 'cp', name + ':/artifact/' + filename, str(receipt_dir / filename)], check=True)
        # Rebuild identity is new; the historical binary is not overwritten.
        result['rebuilt_binary_sha256'] = replay.read_json(receipt_dir / 'BUILD_REPRODUCED.json')['binary_sha256']
        subprocess.run(['docker', 'stop', '--time=2', name], check=True)
        result['rebuilt_image_id'] = subprocess.check_output(commands['commit'], text=True).strip()
        result['status'] = 'built_not_network_exercised'
        return result
    except BaseException as error:
        result['error'] = type(error).__name__ + ': ' + str(error)
        raise
    finally:
        if created:
            stopped = subprocess.run(['docker', 'stop', '--time=2', name], capture_output=True, text=True)
            result['stop_return_code'] = stopped.returncode
        result['finished_at_utc'] = replay.now()
        replay.write_new(receipt_dir / 'BUILD_LOCAL_RECEIPT.json', result)


def check_dependencies(ns3_root, output, root=ROOT):
    expected = identity(root)
    for relative, wanted in expected['ns3_files'].items():
        if replay.digest(ns3_root / relative) != wanted:
            raise ValueError('ns-3 dependency fingerprint differs: ' + relative)
    if (ns3_root / 'VERSION').read_text().strip() != expected['ns3_version']:
        raise ValueError('ns-3 version differs')
    aqua = ns3_root / 'src/aqua-sim-ng'
    actual = subprocess.check_output(['git', '-C', str(aqua), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != expected['aqua_sim_commit']:
        raise ValueError('Aqua-Sim commit differs')
    subprocess.run(['git', '-C', str(aqua), 'diff', '--quiet', 'HEAD', '--'], check=True)
    # Untracked source can affect CMake globbing; reject it, not only tracked diffs.
    extra = subprocess.check_output(['git', '-C', str(aqua), 'ls-files',
                                     '--others', '--exclude-standard'], text=True).strip()
    if extra:
        raise ValueError('Untracked Aqua-Sim source present')
    receipt = {'schema': 'reviewed-v6-local-dependency-check-v1',
               'checked_at_utc': replay.now(), 'status': 'passed',
               'expected_local_base': expected, 'network_runs_executed': 0,
               'aqua_tracked_tree_clean': True, 'aqua_untracked_files': 0,
               'full_ns3_tree_attested': False}
    replay.write_new(output, receipt)
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['plan', 'inspect', 'build', '_check-dependencies'])
    parser.add_argument('--image', default='edc-reviewed-v6:cached')
    parser.add_argument('--ns3-root', type=Path, default=Path('/opt/ns-allinone-3.40/ns-3.40'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--receipt-dir', type=Path)
    parser.add_argument('--wall-timeout-s', type=int, default=900)
    args = parser.parse_args(argv)
    if args.mode == '_check-dependencies':
        if args.output is None:
            parser.error('--output is required for the new dependency check receipt')
        result = check_dependencies(args.ns3_root, args.output)
    else:
        replay.verify_model()
        expected = identity()
        commands = build_commands(args.image, 'reviewed-v6-build-plan')
        result = {'mode': args.mode, 'build_commands': commands,
                  'expected_base': expected, 'network_simulations_executed': 0,
                  'recipe_previously_built': False,
                  'build_resource_note': 'Dedicated container: 2 CPU, 4 GiB total memory+swap, no network, finite compiler wall deadline; no scientific workload.'}
        if args.mode != 'plan':
            result['base_before'] = inspect_base(expected)
        if args.mode == 'build':
            if args.receipt_dir is None:
                parser.error('--receipt-dir must name a new directory for the new build provenance')
            result = build(args.image, args.receipt_dir, args.wall_timeout_s)
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print('LOCAL_BUILD_BLOCKED: ' + str(error), file=sys.stderr)
        raise SystemExit(2)
