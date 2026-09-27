"""Synthetic tests only. No Docker process, compiler or simulation is started."""
import contextlib
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import local_build as local


class LocalBuildTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='v6-local-build-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.ns = self.root / 'ns3'
        self.ns.mkdir()
        (self.ns / 'VERSION').write_text('3.40\n')
        self.expected = dict(local.identity(), ns3_files={'VERSION': local.replay.digest(self.ns / 'VERSION')})
        (self.root / 'LOCAL_BASE_IDENTITY.json').write_text(json.dumps(self.expected))

    def test_plan_does_not_touch_docker(self):
        with patch.object(local.replay, 'verify_model'), \
             patch.object(local.subprocess, 'run', side_effect=AssertionError('no process')), \
             patch.object(local.subprocess, 'check_output', side_effect=AssertionError('no process')), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(local.main(['plan']), 0)

    def test_create_uses_exact_id_and_finite_cgroup(self):
        command = local.build_commands('v6:test', 'reviewed-v6-build-fixture')['create']
        for flag in ('--pull=never', '--network=none', '--cpus=2', '--memory=4g', '--memory-swap=4g', '--pids-limit=256'):
            self.assertIn(flag, command)
        self.assertIn(self.expected['image_id'], command)
        self.assertNotIn(self.expected['image_tag'], command)

    def test_commit_only_dedicated_container(self):
        command = local.build_commands('v6:test', 'reviewed-v6-build-fixture')['commit']
        self.assertEqual(command[-2:], ['reviewed-v6-build-fixture', 'v6:test'])
        self.assertIn('ENTRYPOINT ["python3", "/artifact/replay.py"]', command)
        self.assertIn('WORKDIR /output', command)

    def test_existing_campaign_container_refused(self):
        with self.assertRaises(ValueError):
            local.build_commands('v6:test', 'sensors-revision-20260919')

    def test_base_tag_cannot_be_overwritten(self):
        with self.assertRaises(ValueError):
            local.build_commands(self.expected['image_tag'], 'reviewed-v6-build-fixture')

    def test_invalid_image_argument_refused(self):
        with self.assertRaises(ValueError):
            local.build_commands('--privileged', 'reviewed-v6-build-fixture')

    def test_exact_image_inspection(self):
        image = {'Id': self.expected['image_id'], 'Architecture': 'arm64', 'Os': 'linux'}
        with patch.object(local.subprocess, 'check_output', return_value=json.dumps([image])):
            self.assertEqual(local.inspect_base(self.expected), image)
        image['Id'] = 'sha256:' + '0' * 64
        with patch.object(local.subprocess, 'check_output', return_value=json.dumps([image])):
            with self.assertRaisesRegex(ValueError, 'identity differs'):
                local.inspect_base(self.expected)

    def test_dependency_check_creates_new_receipt(self):
        out = self.root / 'checked.json'
        with patch.object(local.subprocess, 'check_output', side_effect=[self.expected['aqua_sim_commit'], '']), \
             patch.object(local.subprocess, 'run'):
            result = local.check_dependencies(self.ns, out, self.root)
        self.assertTrue(out.is_file())
        self.assertEqual(result['network_runs_executed'], 0)
        self.assertFalse(result['full_ns3_tree_attested'])

    def test_dependency_hash_change_refused(self):
        (self.ns / 'VERSION').write_text('changed')
        with self.assertRaisesRegex(ValueError, 'fingerprint'):
            local.check_dependencies(self.ns, self.root / 'out.json', self.root)

    def test_untracked_source_refused(self):
        with patch.object(local.subprocess, 'check_output', side_effect=[self.expected['aqua_sim_commit'], 'model/new.cc']), \
             patch.object(local.subprocess, 'run'):
            with self.assertRaisesRegex(ValueError, 'Untracked'):
                local.check_dependencies(self.ns, self.root / 'out.json', self.root)

    def test_tracked_modification_refused(self):
        with patch.object(local.subprocess, 'check_output', return_value=self.expected['aqua_sim_commit']), \
             patch.object(local.subprocess, 'run', side_effect=subprocess.CalledProcessError(1, ['git', 'diff'])):
            with self.assertRaises(subprocess.CalledProcessError):
                local.check_dependencies(self.ns, self.root / 'out.json', self.root)

    def test_wall_time_must_be_finite(self):
        for value in (0, -1, 3601):
            with self.assertRaisesRegex(ValueError, 'deadline'):
                local.build('v6:test', self.root / 'receipts', value, self.root)

    def test_output_image_collision_refused_before_create(self):
        with patch.object(local, 'inspect_base', return_value={}), \
             patch.object(local.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0)) as run:
            with self.assertRaisesRegex(ValueError, 'already exists'):
                local.build('v6:test', self.root.parent / 'not-created', 30, self.root)
            self.assertEqual(run.call_count, 1)

    def test_failed_build_stops_new_container_and_preserves_receipt(self):
        out = self.root.parent / (self.root.name + '-receipts')
        self.addCleanup(lambda: __import__('shutil').rmtree(out, ignore_errors=True))
        def run(command, **kwargs):
            if command[:3] == ['docker', 'image', 'inspect']:
                return subprocess.CompletedProcess(command, 1)
            if 'bash' in command:
                raise subprocess.TimeoutExpired(command, 1)
            return subprocess.CompletedProcess(command, 0)
        with patch.object(local, 'inspect_base', return_value={}), \
             patch.object(local.subprocess, 'run', side_effect=run) as calls:
            with self.assertRaises(subprocess.TimeoutExpired):
                local.build('v6:test', out, 1, self.root)
        receipt = json.loads((out / 'BUILD_LOCAL_RECEIPT.json').read_text())
        self.assertEqual(receipt['status'], 'failed')
        self.assertTrue(receipt['container_retained'])
        commands = [call.args[0] for call in calls.call_args_list]
        self.assertTrue(any(command[:3] == ['docker', 'stop', '--time=2'] for command in commands))
        self.assertFalse(any('commit' in command or 'rm' in command for command in commands))


if __name__ == '__main__':
    unittest.main()
