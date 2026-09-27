"""Read-only fixture tests. No plotted scientific result or simulation is generated."""
import copy
import importlib.util
import json
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("figure_builder", HERE / "plot_revision_results.py")
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)
SOURCE = HERE.parents[1] / "sensors-20260920-closure" / "finalization" / "CONFIRMATION_SUMMARY_V6.json"


class FigureChecks(unittest.TestCase):
    def setUp(self):
        self.doc = json.loads(SOURCE.read_text())

    def test_confirmation_numeric(self):
        groups = builder.validate_groups(self.doc)
        direct = builder.numerical_row(groups["confirmation", "nominal", "D-P"])
        hybrid = builder.numerical_row(groups["confirmation", "nominal", "H-S"])
        self.assertEqual(direct["mean_mature_low_windows"], 1648)
        self.assertEqual(direct["low_timely_percent"], 100)
        self.assertAlmostEqual(hybrid["low_timely_percent"], 100 * 13.8 / 1648)
        self.assertAlmostEqual(hybrid["total_energy_MJ"], 29.087476782525826)
        self.assertAlmostEqual(hybrid["surface_plus_vehicle_energy_MJ"], 27.571904399607788)

    def test_duplicate_fails(self):
        self.doc["groups"].append(copy.deepcopy(self.doc["groups"][0]))
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            builder.validate_groups(self.doc)

    def test_nonfinite_fails(self):
        self.doc["groups"][0]["metrics"]["high_miss_fraction"]["mean"] = float("nan")
        with self.assertRaisesRegex(ValueError, "Nonfinite"):
            builder.validate_groups(self.doc)

    def test_wrong_cohort_fails(self):
        self.doc["groups"][0]["metrics"]["mature_low_windows"] = dict(mean=1649, min=1649, max=1649)
        with self.assertRaisesRegex(ValueError, "cohorts"):
            builder.validate_groups(self.doc)

    def test_wrong_model_fails(self):
        self.doc["groups"][0]["comparability"]["model_sha256"] = "different"
        with self.assertRaisesRegex(ValueError, "model"):
            builder.validate_groups(self.doc)

    def test_negative_energy_partition_fails(self):
        self.doc["groups"][0]["metrics"]["immersed_energy_j"] = dict(mean=1e12, min=1e12, max=1e12)
        with self.assertRaisesRegex(ValueError, "partition"):
            builder.validate_groups(self.doc)

    def test_incomplete_transfer_fails(self):
        group = copy.deepcopy(self.doc["groups"][0])
        group.update(phase="transfer", context_id="load", n_seeds=5,
                     seeds=[321, 322, 323, 324, 325], row_provenance=group["row_provenance"][:5])
        self.doc["groups"].append(group)
        with self.assertRaisesRegex(ValueError, "all six"):
            builder.validate_groups(self.doc)

    def test_complete_transfer_fixture(self):
        for context in builder.CONTEXTS:
            for base in self.doc["groups"][:2]:
                group = copy.deepcopy(base)
                group.update(phase="transfer", context_id=context, n_seeds=5,
                             seeds=[321, 322, 323, 324, 325], row_provenance=group["row_provenance"][:5])
                self.doc["groups"].append(group)
        self.assertEqual(len(builder.validate_groups(self.doc)), 14)


if __name__ == "__main__":
    unittest.main()
