"""Fail-closed closure campaign checks. This module never launches a simulator.

Reference choices are not measured hardware performance. Evidence receipts are
necessary integrity checks, not an automated substitute for reviewing the model.
Reports are generated separately from the unchanged source and manuscript.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
from typing import Any


CHECKS = (
    "acoustic_physical_family_and_range",
    "reverse_ack_and_retry_semantics",
    "complete_state_energy_and_finite_batteries",
    "finite_ingress_storage_and_conservation",
    "continuous_route_contact_and_safe_return",
    "causal_planner_and_control_telemetry",
    "paired_input_and_outcome_audit",
    "compiled_model_and_regression",
)
UNRESOLVED_PATHS = (
    "acoustic.fixed_range_m", "acoustic.tx_electrical_power_w",
    "acoustic.rx_active_power_w", "acoustic.frame_and_ack_model",
    "optical.startup_delay_s", "optical.sleep_power_w",
    "optical.supply_conversion_efficiency", "optical.channel_model",
    "vehicle.certified_route_catalogue", "vehicle.return_and_offload_geometry",
    "storage_and_ingress.source_archive_capacity_bytes",
    "storage_and_ingress.working_ram_capacity_bytes",
    "storage_and_ingress.vehicle_store_capacity_bytes",
    "storage_and_ingress.dedicated_ingress_medium",
    "storage_and_ingress.dedicated_ingress_service_bps",
    "storage_and_ingress.retention_and_backpressure",
    "storage_and_ingress.cable_termination_and_power_evidence",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def digest_valid(value: Any) -> bool:
    return isinstance(value, str) and re.fullmatch(r"[a-f0-9]{64}", value) is not None


def numeric(value: Any, *, positive: bool = True) -> bool:
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and (value > 0 if positive else value >= 0))


def lookup(document: dict, dotted: str) -> Any:
    result: Any = document
    for part in dotted.split("."):
        if not isinstance(result, dict):
            return None
        result = result.get(part)
    return result


def fresh_timestamp(value: Any, max_age_s: float = 120.0) -> bool:
    if not isinstance(value, str):
        return False
    try:
        stamp = datetime.fromisoformat(value.replace("Z", "+00:00"))
        if stamp.tzinfo is None:
            return False
        age = (datetime.now(timezone.utc) - stamp).total_seconds()
        return -5.0 <= age <= max_age_s
    except (ValueError, OverflowError):
        return False


def confined_file(base: Path, value: Any) -> Path:
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        raise ValueError("Evidence paths must be nonempty relative paths")
    path = (base / value).resolve()
    if not path.is_relative_to(base.resolve()) or not path.is_file():
        raise ValueError("Evidence path escapes its root or is not a file")
    return path


def parse_thermal(return_code: int, stdout: str, stderr: str = "") -> dict:
    """Recognize absence of recorded warnings, not a Celsius measurement."""
    combined = stdout + "\n" + stderr
    if return_code != 0 or re.search(r"error|failed|denied", combined, re.I):
        status = "unavailable"
    elif ("No thermal warning level has been recorded" in stdout
          and "No performance warning level has been recorded" in stdout
          and not re.search(r"CPU_Speed_Limit\s*=\s*(?:[1-9]?\d)\b", stdout)):
        status = "no_recorded_warning"
    else:
        # An unrecognized or actual warning must be inspected, never greenlit.
        status = "warning_or_unrecognized"
    return {"status": status, "return_code": return_code,
            "stdout": stdout.strip(), "stderr": stderr.strip(),
            "temperature_celsius": None}


def read_machine(root: Path) -> dict:
    free = shutil.disk_usage(root).free
    try:
        result = subprocess.run(["pmset", "-g", "therm"], text=True,
                                capture_output=True, timeout=10, check=False)
        thermal = parse_thermal(result.returncode, result.stdout, result.stderr)
    except (OSError, subprocess.TimeoutExpired) as error:
        thermal = {"status": "unavailable", "error": str(error),
                   "temperature_celsius": None}
    return {"checked_at_utc": datetime.now(timezone.utc).isoformat(),
            "free_bytes": free, "thermal": thermal}


def validate_contract(contract: dict, root: Path, *, machine: dict | None = None,
                      evidence_root: Path | None = None) -> dict:
    """Return every blocking reason; passed receipts must match one model hash."""
    if not isinstance(contract, dict):
        raise ValueError("Contract must be an object")
    root = root.resolve()
    evidence_root = (evidence_root or root).resolve()
    blockers: list[str] = []
    warnings: list[str] = []
    if contract.get("schema_version") != 1:
        blockers.append("unsupported_contract_schema")
    if contract.get("status") != "ready_for_reviewed_launch":
        blockers.append("implementation_not_ready")
    model_hash = contract.get("model_sha256")
    if not digest_valid(model_hash):
        blockers.append("missing_compiled_model_identity")
    if not digest_valid(contract.get("accepted_plan_sha256")):
        blockers.append("missing_plan_identity")
    try:
        plan_path = confined_file(root, contract.get("accepted_plan_path"))
        if sha256(plan_path) != contract.get("accepted_plan_sha256"):
            raise ValueError("Accepted plan hash mismatch")
    except (ValueError, OSError) as error:
        blockers.append("accepted_plan_check:" + str(error))

    for field in UNRESOLVED_PATHS:
        if lookup(contract, field) is None or lookup(contract, field) == "":
            blockers.append("unresolved:" + field)
    band = lookup(contract, "acoustic.band_khz")
    center = lookup(contract, "acoustic.study_center_khz")
    if (not isinstance(band, list) or len(band) != 2
            or not all(numeric(x) for x in band) or not numeric(center)
            or not band[0] <= center <= band[1]):
        blockers.append("incoherent_acoustic_frequency")
    rate = lookup(contract, "acoustic.nominal_mode_bps")
    maximum = lookup(contract, "acoustic.maximum_documented_mode_bps")
    if not numeric(rate) or not numeric(maximum) or rate > maximum:
        blockers.append("incoherent_acoustic_mode_rate")
    for field in ("acoustic.fixed_range_m", "acoustic.tx_electrical_power_w",
                  "acoustic.rx_active_power_w",
                  "storage_and_ingress.source_archive_capacity_bytes",
                  "storage_and_ingress.working_ram_capacity_bytes",
                  "storage_and_ingress.vehicle_store_capacity_bytes",
                  "storage_and_ingress.outbox_capacity_bytes",
                  "storage_and_ingress.dedicated_ingress_service_bps"):
        value = lookup(contract, field)
        if value is not None and not numeric(value):
            blockers.append("invalid_positive_value:" + field)
    for field in ("optical.startup_delay_s", "optical.sleep_power_w"):
        value = lookup(contract, field)
        if value is not None and not numeric(value, positive=False):
            blockers.append("invalid_nonnegative_value:" + field)
    efficiency = lookup(contract, "optical.supply_conversion_efficiency")
    if efficiency is not None and (not numeric(efficiency) or efficiency > 1):
        blockers.append("invalid_supply_efficiency")
    reserve = lookup(contract, "vehicle.reserve_fraction")
    if not numeric(reserve, positive=False) or reserve >= 1:
        blockers.append("invalid_vehicle_reserve")
    dimensions = [lookup(contract, "workload." + name) for name in
                  ("channels", "sample_rate_hz", "study_sample_bits", "window_s")]
    if (not all(numeric(value) for value in dimensions)
            or math.prod(dimensions) / 8 != lookup(contract, "workload.raw_window_bytes")):
        blockers.append("incoherent_acquisition_payload")

    evidence = contract.get("required_evidence", {})
    if not isinstance(evidence, dict) or set(evidence) != set(CHECKS):
        blockers.append("evidence_check_set_mismatch")
        evidence = evidence if isinstance(evidence, dict) else {}
    checked_receipts = []
    for name in CHECKS:
        receipt = evidence.get(name)
        if not isinstance(receipt, dict):
            blockers.append("missing_evidence:" + name)
            continue
        try:
            path = confined_file(evidence_root, receipt.get("path"))
            if not digest_valid(receipt.get("sha256")) or sha256(path) != receipt["sha256"]:
                raise ValueError("Evidence hash mismatch")
            data = json.loads(path.read_text())
            if (not isinstance(data, dict) or data.get("status") != "passed"
                    or data.get("check") != name or not digest_valid(model_hash)
                    or data.get("model_sha256") != model_hash
                    or data.get("contract_id") != contract.get("contract_id")):
                raise ValueError("Evidence is not a passed check for this model and contract")
            checked_receipts.append(name)
        except (ValueError, KeyError, OSError, TypeError) as error:
            blockers.append("invalid_evidence:" + name + ":" + str(error))

    protected = contract.get("protected_inputs", [])
    if not isinstance(protected, list) or not protected:
        blockers.append("missing_protected_input_identities")
        protected = []
    verified_inputs = []
    for entry in protected:
        try:
            path = confined_file(root, entry["path"])
            if not digest_valid(entry.get("sha256")) or sha256(path) != entry["sha256"]:
                raise ValueError("Protected input changed; inspect user changes, never overwrite")
            verified_inputs.append(entry["path"])
        except (ValueError, OSError, TypeError, KeyError) as error:
            blockers.append("protected_input_check:" + str(error))

    policy = contract.get("resource_policy", {})
    required_policy = {"max_network_runs": 400, "cpu_limit": 2,
                       "maximum_isolated_workers": 2,
                       "memory_limit_bytes": 6442450944,
                       "per_run_stop_free_bytes": 1500000000,
                       "delete_old_results": False}
    for name, expected in required_policy.items():
        value = policy.get(name) if isinstance(policy, dict) else None
        if type(value) is not type(expected) or value != expected:
            blockers.append("resource_policy_mismatch:" + name)
    start_min = policy.get("minimum_start_free_bytes") if isinstance(policy, dict) else None
    if not numeric(start_min) or start_min < 4294967296:
        blockers.append("invalid_campaign_start_disk_floor")
    if not isinstance(machine, dict):
        blockers.append("fresh_machine_check_required")
    else:
        if not fresh_timestamp(machine.get("checked_at_utc")):
            blockers.append("fresh_machine_check_required")
        free = machine.get("free_bytes")
        if not numeric(free, positive=False) or not numeric(start_min) or free < start_min:
            blockers.append("insufficient_campaign_disk_headroom")
        thermal_data = machine.get("thermal")
        thermal = thermal_data.get("status") if isinstance(thermal_data, dict) else None
        if thermal != "no_recorded_warning":
            blockers.append("thermal_status_unavailable_or_warning")
        warnings.append("No Celsius measurement or future thermal guarantee is inferred.")

    return {"schema_version": 1, "contract_id": contract.get("contract_id"),
            "model_sha256": model_hash,
            "accepted_plan_sha256": contract.get("accepted_plan_sha256"),
            "status": "blocked" if blockers else "ready_for_reviewed_launch",
            "network_launch_allowed": not blockers,
            "blockers": blockers, "warnings": warnings,
            "verified_protected_inputs": verified_inputs,
            "verified_evidence_checks": checked_receipts,
            "machine": machine,
            "interpretation": "Integrity and readiness checks, not hardware or sea-trial validation."}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    contract = json.loads(args.contract.read_text())
    result = validate_contract(contract, args.workspace, machine=read_machine(args.workspace),
                               evidence_root=args.contract.parent)
    result["contract_sha256"] = sha256(args.contract)
    result["checked_at_utc"] = datetime.now(timezone.utc).isoformat()
    content = json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    if args.output:
        if args.output.exists():
            raise SystemExit("Refusing to overwrite an existing preflight receipt")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as stream:
            stream.write(content)
    print(content, end="")
    return 0 if result["network_launch_allowed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
