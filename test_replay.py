"""Synthetic launcher tests only: no Docker, compilation or network simulation."""
import contextlib
import gzip
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import replay


class ReplayTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='v6-artifact-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.model = self.root / 'model'
        self.model.mkdir()
        for name in ('edc-reviewed.cc', 'closure_phy.h', 'test_closure_phy.cc'):
            (self.model / name).write_text('synthetic:' + name)
        self.manifest = {'schema': 'reviewed-v6-source-package-v1',
            'model_files': {p.name: replay.digest(p) for p in self.model.iterdir()},
            'historical_binary_sha256': 'a' * 64, 'aqua_sim_commit': 'b' * 40,
            'allowed_argv_names': sorted([*replay.OUTPUTS, 'workloadReplayCsv',
                                         'enableAnimation', 'closureFullValidation', 'RngRun'])}
        self.save(self.root / 'MODEL_MANIFEST.json', self.manifest)
        self.archive = self.root / 'input with spaces'
        self.archive.mkdir()
        (self.archive / 'workload.csv').write_text('synthetic workload\n')
        workload = replay.digest(self.archive / 'workload.csv')
        self.job = {'schema_version': 1, 'job_id': 'fixture', 'full_physical_validation': False,
            'source': {'path': '/old/source/edc-reviewed.cc', 'sha256': self.manifest['model_files']['edc-reviewed.cc']},
            'binary': {'path': '/old/binary', 'sha256': 'a' * 64}, 'model_sha256': 'a' * 64,
            'headers': [{'path': '/old/closure_phy.h', 'sha256': self.manifest['model_files']['closure_phy.h']}],
            'workload_sha256': workload, 'generated_ledger_sha256': workload, 'wall_timeout_s': 15,
            'argv': ['--RngRun=321', '--enableAnimation=0', '--closureFullValidation=0',
                     '--workloadReplayCsv=/old/workload.csv',
                     *('--' + k + '=/old/' + v for k, v in replay.OUTPUTS.items())]}
        self.freeze_job()
        self.save(self.archive / 'execution.json', {'job_sha256': self.job['job_sha256'],
                                                  'status': 'completed', 'return_code': 0})
        self.save(self.archive / 'energy.json', {'synthetic': True})
        self.entries = []
        for name in ('metrics.csv', 'packets.csv', 'storage.csv', 'ingress.csv', 'mission.csv',
                     'stdout.log', 'stderr.log'):
            content = ('synthetic ' + name).encode()
            path = self.archive / (name + '.gz')
            path.write_bytes(gzip.compress(content, mtime=0))
            self.entries.append({'path': path.name, 'sha256': replay.digest(path),
                'compressed_bytes': path.stat().st_size, 'original_bytes': len(content),
                'original_sha256': replay.hashlib.sha256(content).hexdigest()})
        self.index()

    def save(self, path, value):
        path.write_text(json.dumps(value, indent=2))

    def freeze_job(self):
        self.job.pop('job_sha256', None)
        self.job['job_sha256'] = replay.content_hash(self.job)
        self.save(self.archive / 'job.json', self.job)

    def index(self):
        entries = list(self.entries)
        for name in ('job.json', 'execution.json', 'energy.json'):
            path = self.archive / name
            entries.append({'path': name, 'sha256': replay.digest(path), 'bytes': path.stat().st_size})
        self.save(self.archive / 'artifacts.json', {'files': entries,
                  'paired_workload_sha256': self.job['workload_sha256']})

    def verify(self, deep=False):
        return replay.verify_archive(self.archive, self.root, deep)

    def test_valid_shallow(self):
        result, _ = self.verify()
        self.assertEqual(result['status'], 'passed')
        self.assertEqual(result['network_runs_executed'], 0)

    def test_valid_deep(self):
        self.assertTrue(self.verify(True)[0]['deep_compression_check'])

    def test_model_tamper(self):
        (self.model / 'edc-reviewed.cc').write_text('changed')
        with self.assertRaisesRegex(ValueError, 'Source hash'):
            self.verify()

    def test_workload_tamper(self):
        (self.archive / 'workload.csv').write_text('changed')
        with self.assertRaisesRegex(ValueError, 'Workload hash'):
            self.verify()

    def test_canonical_job_tamper(self):
        self.job['argv'][0] = '--RngRun=999'
        self.save(self.archive / 'job.json', self.job)
        with self.assertRaisesRegex(ValueError, 'Canonical'):
            self.verify()

    def test_old_source_rejected(self):
        self.job['source']['sha256'] = 'c' * 64
        self.freeze_job()
        with self.assertRaisesRegex(ValueError, 'not the packaged'):
            self.verify()

    def test_duplicate_option(self):
        self.job['argv'].append('--RngRun=1')
        self.freeze_job()
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            self.verify()

    def test_unreviewed_file_option(self):
        self.job['argv'].append('--animationFile=/elsewhere/overwrite')
        self.freeze_job()
        with self.assertRaisesRegex(ValueError, 'unreviewed'):
            self.verify()

    def test_missing_trace_option(self):
        self.job['argv'] = [a for a in self.job['argv'] if not a.startswith('--metricsCsv=')]
        self.freeze_job()
        with self.assertRaisesRegex(ValueError, 'Missing required trace'):
            self.verify()

    def test_nonfinite_wall(self):
        self.job['wall_timeout_s'] = -1
        self.freeze_job()
        with self.assertRaisesRegex(ValueError, 'wall limit'):
            self.verify()

    def test_archive_traversal(self):
        self.entries[0]['path'] = '../outside.gz'
        self.index()
        with self.assertRaisesRegex(ValueError, 'Unsafe archive'):
            self.verify()

    def test_symlink_member(self):
        path = self.archive / 'energy.json'
        path.unlink()
        path.symlink_to(self.root / 'MODEL_MANIFEST.json')
        self.index()
        with self.assertRaisesRegex(ValueError, 'symlinked'):
            self.verify()

    def test_compressed_tamper(self):
        (self.archive / self.entries[0]['path']).write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError, 'Archived file hash'):
            self.verify()

    def test_original_digest_tamper(self):
        self.entries[0]['original_sha256'] = '0' * 64
        self.index()
        with self.assertRaisesRegex(ValueError, 'Original compressed'):
            self.verify(True)

    def test_bounded_decompression(self):
        self.entries[0]['original_bytes'] = 1
        self.index()
        with self.assertRaisesRegex(ValueError, 'Decompressed size'):
            self.verify(True)

    def test_missing_archived_evidence(self):
        self.entries = [e for e in self.entries if e['path'] != 'mission.csv.gz']
        self.index()
        with self.assertRaisesRegex(ValueError, 'Required archived'):
            self.verify()

    def test_failed_execution_is_reported_not_hidden(self):
        self.save(self.archive / 'execution.json', {'job_sha256': self.job['job_sha256'],
                   'status': 'execution_failed', 'return_code': -9})
        self.index()
        result, _ = self.verify()
        self.assertEqual(result['historical_execution_status'], 'execution_failed')
        self.assertIn('not_independent_authentication_or_scientific_validity', result['scope'])

    def test_scientific_parameters_and_order_unchanged(self):
        result = replay.relocated_argv(self.job, '/input/workload.csv')
        self.assertEqual(result[:3], self.job['argv'][:3])
        names = set(replay.OUTPUTS) | {'workloadReplayCsv'}
        scientific = lambda args: [a for a in args if a[2:].split('=', 1)[0] not in names]
        self.assertEqual(scientific(result), scientific(self.job['argv']))
        self.assertIn('--metricsCsv=metrics.csv', result)

    def test_plan_is_bounded_and_does_not_create_output(self):
        out = self.root / 'new output with spaces'
        command = replay.replay_command(self.archive, out, 'fixture:local')
        self.assertFalse(out.exists())
        for flag in ('--pull=never', '--network=none', '--cpus=2', '--memory=4g', '--memory-swap=4g', '--read-only'):
            self.assertIn(flag, command)
        self.assertIn(str(self.archive.resolve()) + ':/input:ro', command)

    def test_output_inside_archive_rejected(self):
        with self.assertRaisesRegex(ValueError, 'separate'):
            replay.replay_command(self.archive, self.archive / 'new', 'fixture')

    def test_existing_output_rejected(self):
        with self.assertRaisesRegex(ValueError, 'must not exist'):
            replay.replay_command(self.archive, self.model, 'fixture')

    def test_cgroup_limits(self):
        group = self.root / 'cgroup'
        group.mkdir()
        (group / 'cpu.max').write_text('200000 100000')
        (group / 'memory.max').write_text(str(4 * 1024**3))
        self.assertEqual(replay.check_cgroup(group)['cpus'], 2)
        (group / 'cpu.max').write_text('300000 100000')
        with self.assertRaisesRegex(ValueError, 'exceeds'):
            replay.check_cgroup(group)
        (group / 'cpu.max').write_text('200000 100000')
        (group / 'memory.max').write_text(str(5 * 1024**3))
        with self.assertRaisesRegex(ValueError, '4 GiB'):
            replay.check_cgroup(group)

    def test_plan_cli_cannot_launch_docker(self):
        with patch.object(replay, 'verify_archive', return_value=self.verify()), \
             patch.object(replay.subprocess, 'run', side_effect=AssertionError('must not launch')), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(replay.main(['plan', '--archive', str(self.archive),
                             '--output', str(self.root / 'new')]), 0)

    def test_build_dry_run_cannot_launch_docker(self):
        with patch.object(replay, 'verify_model', return_value=self.manifest), \
             patch.object(replay.subprocess, 'run', side_effect=AssertionError('must not launch')), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(replay.main(['build', '--dry-run']), 0)

    def test_rebuild_keeps_new_binary_identity_distinct(self):
        ns = self.root / 'ns3'
        (ns / 'scratch').mkdir(parents=True)
        (ns / 'build/scratch').mkdir(parents=True)
        for source in self.model.iterdir():
            (ns / 'scratch' / source.name).write_bytes(source.read_bytes())
        (ns / 'VERSION').write_text('3.40\n')
        (ns / 'build/scratch/ns3.40-edc-reviewed-optimized').write_bytes(b'synthetic rebuilt binary')
        with patch.object(replay, 'ROOT', self.root), \
             patch.object(replay, 'verify_model', return_value=self.manifest), \
             patch.object(replay.subprocess, 'check_output', side_effect=['b' * 40 + '\n', 'synthetic compiler']):
            result = replay.record_build(ns, self.root / 'new-build.json')
        self.assertNotEqual(result['binary_sha256'], result['historical_binary_sha256'])
        self.assertEqual(result['network_runs_executed'], 0)
        self.assertFalse(result['compiled_network_model_tested'])

    def test_duplicate_header_rejected(self):
        self.job['headers'].append(dict(self.job['headers'][0]))
        self.freeze_job()
        with self.assertRaisesRegex(ValueError, 'header identities'):
            self.verify()

    def test_unknown_historical_binary_rejected(self):
        self.job['binary']['sha256'] = '0' * 64
        self.freeze_job()
        with self.assertRaisesRegex(ValueError, 'historical binary'):
            self.verify()


if __name__ == '__main__':
    unittest.main()
