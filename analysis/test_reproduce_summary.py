"""Offline relocation tests. No Docker, simulation or network is used."""
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import reproduce_summary as reproduction


HERE=Path(__file__).resolve().parent


class RelocationTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name).resolve()
        self.original=self.root/"original"
        self.actual=self.root/"archive"
        self.code=self.root/"code"
        for path in (self.original,self.actual,self.code):path.mkdir()
        self.output=self.root/"new-summary.json"
        self.pairs=[str(self.original)+"="+str(self.actual)]
        self.mapping=reproduction.Relocation(self.pairs,self.code,self.output)
        self.Lookup=self.mapping.path_class()

    def tearDown(self):
        self.temp.cleanup()

    def write_fixture(self,path,value):
        with path.open("x") as f:f.write(value)

    def test_lookup_preserves_logical_json_and_digest(self):
        self.write_fixture(self.actual/"evidence.json",'{"path":"'+str(self.original)+'/job"}\n')
        logical=self.Lookup(self.original/"evidence.json")
        self.assertEqual(str(logical),str(self.original/"evidence.json"))
        self.assertEqual(logical.read_text(),(self.actual/"evidence.json").read_text())
        self.assertEqual(str(logical.resolve()),str(logical))

    def test_actual_cli_path_is_translated_back_to_logical_identity(self):
        path=self.Lookup(self.actual/"phase")
        self.assertEqual(str(path),str(self.original/"phase"))
        self.assertEqual(str(path/"job"),str(self.original/"phase/job"))

    def test_missing_archive_never_falls_back_to_original(self):
        self.write_fixture(self.original/"only-at-origin.txt","must not be read")
        with self.assertRaises(FileNotFoundError):
            self.Lookup(self.original/"only-at-origin.txt").read_text()
        self.assertFalse(self.Lookup(self.original/"only-at-origin.txt").exists())

    def test_unmapped_read_is_denied(self):
        self.write_fixture(self.root/"unmapped.txt","not in evidence")
        with self.assertRaises(PermissionError):
            self.Lookup(self.root/"unmapped.txt").read_text()

    def test_component_prefix_not_string_prefix(self):
        with self.assertRaises(PermissionError):
            self.Lookup(str(self.original)+"-other/evidence").read_text()

    def test_escape_and_symlink_escape_are_denied(self):
        with self.assertRaises(ValueError):
            self.mapping.lookup(self.original/"../escape")
        self.write_fixture(self.root/"outside.txt","outside")
        (self.actual/"symlink.txt").symlink_to(self.root/"outside.txt")
        with self.assertRaises(PermissionError):
            self.Lookup(self.original/"symlink.txt").read_text()

    def test_archive_writes_and_nonexclusive_output_are_denied(self):
        for path,mode in ((self.original/"new","x"),(self.output,"w"),(self.output,"a"),(self.output,"r+")):
            with self.assertRaises(PermissionError):
                self.Lookup(path).open(mode)
        with self.Lookup(self.output).open("x") as stream:stream.write("{}")
        with self.assertRaises(FileExistsError):
            self.Lookup(self.output).open("x")

    def test_ambiguous_roots_and_output_in_archive_are_denied(self):
        with self.assertRaises(ValueError):
            reproduction.Relocation(self.pairs+self.pairs,self.code,self.output)
        with self.assertRaises(ValueError):
            reproduction.Relocation(self.pairs,self.code,self.actual/"summary.json")

    def test_no_mapping_is_not_a_fallback_mode(self):
        with self.assertRaises(ValueError):
            reproduction.Relocation([],self.code,self.output)

    def test_frozen_code_inventory_is_complete(self):
        manifest=reproduction.verified_code(HERE/"frozen")
        self.assertEqual(len(manifest["files"]),7)

    def test_processes_are_denied_and_imports_restored(self):
        original_run=reproduction.subprocess.run
        with reproduction.offline_modules(HERE/"frozen",self.mapping) as summary:
            self.assertTrue(reproduction.sys.dont_write_bytecode)
            with self.assertRaises(PermissionError):
                reproduction.subprocess.run(["should-never-execute"])
            self.assertEqual(summary.BOOTSTRAP_RESAMPLES,10000)
        self.assertIs(reproduction.subprocess.run,original_run)

    def test_scope_and_union_paths_remain_logical(self):
        self.write_fixture(self.actual/"binding.json","{}")
        # This is the same resolve/string operation used by union receipts.
        relocated=self.Lookup(self.actual)
        expected={"continuation_root":str(self.original),"job_root":str(self.original/"phase")}
        observed={"continuation_root":str(relocated.resolve()),"job_root":str(relocated/"phase")}
        self.assertEqual(observed,expected)


if __name__=="__main__":
    unittest.main()
