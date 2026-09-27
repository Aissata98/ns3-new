"""Frozen selection contract for the closure campaign; never launches simulations.

All probabilities used by selection are derived from mature-cohort counts and
averaged per run, not from pooled packets. ``protocol_manifest()`` is available
while the hardware gate is blocked; executable job descriptions are not.

Gate evidence is a structural/provenance check, not automatic verification of
physical truth. Only a fresh, file-reverified hardware preflight can create a
ready gate. Merely constructing ``HardwareGate(status="passed", ...)`` cannot.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
from types import MappingProxyType
from typing import Any, Iterable, Mapping

import hardware_preflight as _preflight


SCHEMA_VERSION = "sensors-closure-protocol-v2"
FAMILIES = ("D-P", "H-S", "H-A")
TRAIN_SEEDS = tuple(range(301, 306))
CONFIRM_SEEDS = tuple(range(311, 321))
TRANSFER_SEEDS = tuple(range(321, 326))
TRANSFER_CONTEXTS = ("load", "long", "sparse", "raw32", "memory1mb", "perturbed")
PHASE_CAPS = MappingProxyType({
    "tuning": 195,
    "confirmation": 30,
    "transfer": 90,
    "summary": 15,
    "mechanism_ablation": 15,
    "qa": 20,
    "immediate_ablation": 15,
    "technical_retry": 20,
})
TOTAL_CAP = 400
LIMITS = MappingProxyType({"high_miss_fraction": 0.01, "medium_miss_fraction": 0.05,
                          "low_window_miss_fraction": 0.05})
GATE_CHECKS = tuple(_preflight.CHECKS)
PREFLIGHT_MAX_AGE_SECONDS = 120.0
_VERIFIED_PREFLIGHT_CAPABILITY = object()
_SHA256 = re.compile(r"[0-9a-f]{64}\Z")
_IDENTIFIER = re.compile(r"[a-z][a-z0-9_-]*\Z")


class ProtocolError(ValueError):
    """Input violates the predeclared experimental contract."""


class HardwareGateError(ProtocolError):
    """A simulation campaign is not yet permitted by the evidence gate."""


def canonical_json(value: Any) -> str:
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"),
                          ensure_ascii=True, allow_nan=False)
    except (TypeError, ValueError) as exc:
        raise ProtocolError("Only finite JSON values can be frozen") from exc


def content_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def _hash(value: Any, label: str) -> None:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ProtocolError(f"{label} must be a lowercase SHA-256 digest")


def _integer(value: Any, label: str, minimum: int = 0) -> None:
    if type(value) is not int or value < minimum:
        raise ProtocolError(f"{label} must be an integer >= {minimum}")


def _number(value: Any, label: str, minimum: float = 0.0) -> None:
    if type(value) not in (int, float) or not math.isfinite(value) or value < minimum:
        raise ProtocolError(f"{label} must be finite and >= {minimum}")


@dataclass(frozen=True)
class Candidate:
    family: str
    retries: int
    medium_age_s: int | None = None
    medium_refill_milli_records_per_s: int | None = None

    def __post_init__(self) -> None:
        if self.family not in FAMILIES or type(self.retries) is not int or self.retries not in (0, 1, 3):
            raise ProtocolError("Candidate family/retransmission limit is outside the frozen grid")
        if self.family == "D-P":
            if self.medium_age_s is not None or self.medium_refill_milli_records_per_s is not None:
                raise ProtocolError("Direct candidates must not have artificial hybrid controls")
        elif (type(self.medium_age_s) is not int or self.medium_age_s not in (60, 120)
              or type(self.medium_refill_milli_records_per_s) is not int
              or self.medium_refill_milli_records_per_s not in (15, 20, 30)):
            raise ProtocolError("Hybrid candidate is outside the frozen age/reserve grid")

    @property
    def candidate_id(self) -> str:
        suffix = "" if self.family == "D-P" else f"-a{self.medium_age_s:03d}-q{self.medium_refill_milli_records_per_s:03d}"
        return f"{self.family}-r{self.retries}{suffix}"

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["candidate_id"] = self.candidate_id
        result["medium_refill_records_per_s"] = (
            None if self.medium_refill_milli_records_per_s is None
            else self.medium_refill_milli_records_per_s / 1000.0)
        result["retry_semantics"] = "requested_per_leg_cycle_limit_pending_acoustic_semantics_gate"
        return result


def candidate_grid() -> tuple[Candidate, ...]:
    """Stable D-P, H-S, H-A order, then retries, age, reserve."""
    direct = tuple(Candidate("D-P", retries) for retries in (0, 1, 3))
    hybrid = tuple(Candidate(family, retries, age, reserve)
                   for family in ("H-S", "H-A") for retries in (0, 1, 3)
                   for age in (60, 120) for reserve in (15, 20, 30))
    return direct + hybrid


def validate_candidate(candidate: Candidate) -> None:
    if type(candidate) is not Candidate or candidate not in candidate_grid():
        raise ProtocolError("Candidate must be an exact member of the frozen grid")


@dataclass(frozen=True)
class HardwareGate:
    status: str = "blocked"
    hardware_sha256: str | None = None
    model_sha256: str | None = None
    evidence: tuple[tuple[str, str], ...] = ()
    _verified_snapshot_json: str | None = field(default=None, repr=False)
    _verification_capability: object | None = field(default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        if self.status not in ("blocked", "passed"):
            raise ProtocolError("Hardware gate status must be blocked or passed")
        if type(self.evidence) is not tuple or any(type(item) is not tuple or len(item) != 2 for item in self.evidence):
            raise ProtocolError("Evidence must be immutable (check, digest) pairs")
        names = [item[0] for item in self.evidence]
        if any(not isinstance(name, str) for name in names):
            raise ProtocolError("Evidence check names must be strings")
        if len(names) != len(set(names)) or any(name not in GATE_CHECKS for name in names):
            raise ProtocolError("Unknown or duplicate hardware gate check")
        for name, digest in self.evidence:
            _hash(digest, f"evidence[{name}]")
        for name in ("hardware_sha256", "model_sha256"):
            if getattr(self, name) is not None:
                _hash(getattr(self, name), name)

    def require_ready(self) -> None:
        if self.status != "passed" or self.hardware_sha256 is None or self.model_sha256 is None:
            raise HardwareGateError("Hardware gate is blocked; no campaign jobs may be generated")
        if set(dict(self.evidence)) != set(GATE_CHECKS):
            raise HardwareGateError("Hardware gate lacks one or more required evidence checks")
        if (self._verification_capability is not _VERIFIED_PREFLIGHT_CAPABILITY
                or self._verified_snapshot_json is None):
            raise HardwareGateError("A hash-only pass assertion cannot replace a verified preflight")
        verified = _verify_preflight_snapshot(json.loads(self._verified_snapshot_json))
        if (verified["contract_sha256"] != self.hardware_sha256
                or verified["model_sha256"] != self.model_sha256
                or verified["evidence"] != self.evidence):
            raise HardwareGateError("Gate identity differs from its file-verified preflight")

    def to_dict(self) -> dict[str, Any]:
        snapshot = None if self._verified_snapshot_json is None else json.loads(self._verified_snapshot_json)
        return {"status": self.status, "hardware_sha256": self.hardware_sha256,
                "model_sha256": self.model_sha256, "evidence_sha256": dict(self.evidence),
                "required_checks": list(GATE_CHECKS),
                "hardware_identity_scope": "exact_HARDWARE_CONTRACT_file_bytes",
                "preflight_report_sha256": None if snapshot is None else content_sha256(snapshot["report"]),
                "accepted_plan_sha256": None if snapshot is None else snapshot["accepted_plan_sha256"],
                "preflight_max_age_seconds": PREFLIGHT_MAX_AGE_SECONDS}


def _timestamp(value: Any, label: str) -> datetime:
    if not isinstance(value, str):
        raise HardwareGateError(f"{label} requires an explicit timezone-aware timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise HardwareGateError(f"Invalid {label}") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise HardwareGateError(f"{label} must not be a naive timestamp")
    return parsed.astimezone(timezone.utc)


def _verify_preflight_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Verify the report AND reread its contract, plan, evidence and protected files.

    The machine observation is a maximum-120-second-old snapshot. It is not a
    substitute for the launcher's fresh resource check and per-run guard.
    """
    try:
        report = snapshot["report"]
        if not isinstance(report, dict) or type(report.get("schema_version")) is not int or report["schema_version"] != 1:
            raise HardwareGateError("Unsupported preflight report schema")
        if (report.get("status") != "ready_for_reviewed_launch"
                or report.get("network_launch_allowed") is not True
                or report.get("blockers") != []):
            raise HardwareGateError("Preflight did not authorize a reviewed launch")
        checks = report.get("verified_evidence_checks")
        if (not isinstance(checks, list) or any(not isinstance(check, str) for check in checks)
                or len(checks) != len(GATE_CHECKS) or set(checks) != set(GATE_CHECKS)):
            raise HardwareGateError("Preflight does not contain exactly the eight file-verified checks")
        machine = report.get("machine")
        if not isinstance(machine, dict):
            raise HardwareGateError("Preflight lacks a machine observation")
        report_time = _timestamp(report.get("checked_at_utc"), "preflight checked_at_utc")
        machine_time = _timestamp(machine.get("checked_at_utc"), "machine checked_at_utc")
        now = datetime.now(timezone.utc)
        for label, timestamp in (("report", report_time), ("machine", machine_time)):
            age = (now - timestamp).total_seconds()
            if age < -5.0 or age > PREFLIGHT_MAX_AGE_SECONDS:
                raise HardwareGateError(f"Preflight {label} is stale or future-dated")
        if (machine_time - report_time).total_seconds() > 5.0:
            raise HardwareGateError("Machine observation postdates the preflight report")
        contract_path = Path(snapshot["contract_path"])
        accepted_plan_path = Path(snapshot["accepted_plan_path"])
        workspace = Path(snapshot["workspace"])
        evidence_root = Path(snapshot["evidence_root"])
        contract_hash = _preflight.sha256(contract_path)
        if report.get("contract_sha256") != contract_hash:
            raise HardwareGateError("Preflight contract hash does not match the current file")
        contract = json.loads(contract_path.read_text(encoding="utf-8"))
        if not isinstance(contract, dict) or type(contract.get("schema_version")) is not int or contract["schema_version"] != 1:
            raise HardwareGateError("Unsupported hardware contract schema")
        if not isinstance(contract.get("contract_id"), str) or not contract["contract_id"]:
            raise HardwareGateError("Hardware contract needs a nonempty identity")
        if report.get("contract_id") != contract["contract_id"]:
            raise HardwareGateError("Preflight and hardware contract identities disagree")
        plan_hash = _preflight.sha256(accepted_plan_path)
        if (plan_hash != contract.get("accepted_plan_sha256")
                or plan_hash != snapshot["accepted_plan_sha256"]):
            raise HardwareGateError("Accepted plan changed or does not match the contract")
        model_hash = contract.get("model_sha256")
        _hash(model_hash, "contract model_sha256")
        # The preflight report repeats the model and accepted plan identity;
        # both copies must agree with the independently verified contract.
        for name, expected in (("model_sha256", model_hash), ("accepted_plan_sha256", plan_hash)):
            if report.get(name) != expected:
                raise HardwareGateError(f"Preflight {name} differs from the verified contract")
        repeated = _preflight.validate_contract(contract, workspace, machine=machine, evidence_root=evidence_root)
        if repeated.get("network_launch_allowed") is not True or repeated.get("blockers") != []:
            raise HardwareGateError("Current file revalidation blocked: " + "; ".join(repeated.get("blockers", [])))
        if repeated.get("verified_evidence_checks") != list(GATE_CHECKS):
            raise HardwareGateError("Current evidence set was not fully verified")
        if report.get("verified_protected_inputs") != repeated.get("verified_protected_inputs"):
            raise HardwareGateError("Report protected-file identities differ from current revalidation")
        evidence = tuple((name, contract["required_evidence"][name]["sha256"]) for name in GATE_CHECKS)
        return {"contract_sha256": contract_hash, "model_sha256": model_hash, "evidence": evidence}
    except HardwareGateError:
        raise
    except (OSError, KeyError, TypeError, ValueError) as exc:
        raise HardwareGateError(f"Unable to verify preflight evidence: {exc}") from exc


def gate_from_preflight(report: Mapping[str, Any], *, contract_path: Path,
                        workspace: Path, accepted_plan_path: Path,
                        evidence_root: Path | None = None) -> HardwareGate:
    """Adapt the actual eight-check preflight; every missing proof fails closed.

    ``hardware_sha256`` in a ContextSpec is the SHA-256 of the exact contract
    file, including its model/evidence identities. No ninth assertion is made.
    This function verifies files only and never launches a process.
    """
    contract_path = Path(contract_path).resolve()
    accepted_plan_path = Path(accepted_plan_path).resolve()
    try:
        plan_hash = _preflight.sha256(accepted_plan_path)
    except OSError as exc:
        raise HardwareGateError("Accepted plan file is unavailable") from exc
    snapshot = {"report": dict(report), "contract_path": str(contract_path),
                "workspace": str(Path(workspace).resolve()),
                "accepted_plan_path": str(accepted_plan_path), "accepted_plan_sha256": plan_hash,
                "evidence_root": str((contract_path.parent if evidence_root is None else Path(evidence_root)).resolve())}
    serialized = canonical_json(snapshot)
    verified = _verify_preflight_snapshot(json.loads(serialized))
    return HardwareGate("passed", verified["contract_sha256"], verified["model_sha256"],
                        verified["evidence"], serialized, _VERIFIED_PREFLIGHT_CAPABILITY)


@dataclass(frozen=True)
class ContextSpec:
    """Canonical serialized definition makes the context deeply immutable."""
    context_id: str
    definition_json: str

    def __post_init__(self) -> None:
        if not isinstance(self.context_id, str) or _IDENTIFIER.fullmatch(self.context_id) is None:
            raise ProtocolError("Invalid context identifier")
        try:
            definition = json.loads(self.definition_json)
        except (TypeError, ValueError) as exc:
            raise ProtocolError("Invalid context definition JSON") from exc
        if not isinstance(definition, dict) or self.definition_json != canonical_json(definition):
            raise ProtocolError("Context definition must be a canonical JSON object")
        required = {"hardware_sha256", "model_sha256", "observation_horizon_s",
                    "workload_profile", "parameters"}
        if not required.issubset(definition):
            raise ProtocolError(f"Context lacks required fields: {sorted(required - set(definition))}")
        for name in ("hardware_sha256", "model_sha256"):
            _hash(definition[name], name)
        _number(definition["observation_horizon_s"], "observation_horizon_s")
        if definition["observation_horizon_s"] <= 0:
            raise ProtocolError("Observation horizon must be positive")
        if not isinstance(definition["workload_profile"], str) or not definition["workload_profile"]:
            raise ProtocolError("Context requires an explicit workload profile")
        if not isinstance(definition["parameters"], dict) or not definition["parameters"]:
            raise ProtocolError("Context requires explicit physical/study parameters")

    @property
    def definition(self) -> dict[str, Any]:
        return json.loads(self.definition_json)

    @property
    def context_sha256(self) -> str:
        return content_sha256({"context_id": self.context_id, "definition": self.definition})

    def to_dict(self) -> dict[str, Any]:
        return {"context_id": self.context_id, "context_sha256": self.context_sha256,
                "definition": self.definition}


def freeze_context(context_id: str, definition: Mapping[str, Any]) -> ContextSpec:
    return ContextSpec(context_id, canonical_json(dict(definition)))


@dataclass(frozen=True)
class WorkloadRef:
    context_sha256: str
    seed: int
    workload_sha256: str
    generated_ledger_sha256: str

    def __post_init__(self) -> None:
        _integer(self.seed, "seed", 1)
        for name in ("context_sha256", "workload_sha256", "generated_ledger_sha256"):
            _hash(getattr(self, name), name)


@dataclass(frozen=True)
class RunMetrics:
    high_mature_generated: int
    high_mature_missed: int
    medium_mature_generated: int
    medium_mature_missed: int
    low_windows_mature_generated: int
    low_windows_mature_missed: int
    immersed_energy_j: float
    full_mission_energy_j: float
    mission_feasible: bool
    storage_feasible: bool
    trajectory_feasible: bool
    conservation_passed: bool

    def __post_init__(self) -> None:
        for stem in ("high", "medium", "low_windows"):
            generated = getattr(self, stem + "_mature_generated")
            missed = getattr(self, stem + "_mature_missed")
            _integer(generated, stem + "_mature_generated", 1)
            _integer(missed, stem + "_mature_missed")
            if missed > generated:
                raise ProtocolError("Mature missed count exceeds its generated cohort")
        for name in ("immersed_energy_j", "full_mission_energy_j"):
            _number(getattr(self, name), name)
        if self.full_mission_energy_j < self.immersed_energy_j:
            raise ProtocolError("Full mission energy must include immersed energy")
        for name in ("mission_feasible", "storage_feasible", "trajectory_feasible", "conservation_passed"):
            if type(getattr(self, name)) is not bool:
                raise ProtocolError(f"{name} must be boolean")
        if not self.conservation_passed:
            raise ProtocolError("A failed conservation audit cannot enter the selector")

    def run_values(self) -> dict[str, float]:
        return {"high_miss_fraction": self.high_mature_missed / self.high_mature_generated,
                "medium_miss_fraction": self.medium_mature_missed / self.medium_mature_generated,
                "low_window_miss_fraction": self.low_windows_mature_missed / self.low_windows_mature_generated,
                "immersed_energy_j": float(self.immersed_energy_j),
                "full_mission_energy_j": float(self.full_mission_energy_j)}

    @property
    def physically_feasible(self) -> bool:
        return self.mission_feasible and self.storage_feasible and self.trajectory_feasible


@dataclass(frozen=True)
class RunResult:
    phase: str
    candidate_id: str
    context_sha256: str
    seed: int
    workload_sha256: str
    generated_ledger_sha256: str
    hardware_sha256: str
    model_sha256: str
    metrics: RunMetrics

    def __post_init__(self) -> None:
        if self.phase not in PHASE_CAPS:
            raise ProtocolError("Unknown phase")
        if self.candidate_id not in {c.candidate_id for c in candidate_grid()}:
            raise ProtocolError("Unknown candidate identifier")
        _integer(self.seed, "seed", 1)
        for name in ("context_sha256", "workload_sha256", "generated_ledger_sha256",
                     "hardware_sha256", "model_sha256"):
            _hash(getattr(self, name), name)
        if type(self.metrics) is not RunMetrics:
            raise ProtocolError("Results require validated per-run mature-cohort metrics")


def validate_phase_counts(counts: Mapping[str, int]) -> int:
    if any(phase not in PHASE_CAPS for phase in counts):
        raise ProtocolError("Unknown phase in execution count ledger")
    for phase, count in counts.items():
        _integer(count, phase + " count")
        if count > PHASE_CAPS[phase]:
            raise ProtocolError(f"Phase {phase} exceeds its frozen cap {PHASE_CAPS[phase]}")
    total = sum(counts.values())
    if total > TOTAL_CAP:
        raise ProtocolError("Campaign exceeds the 400-run ceiling")
    return total


def _validated_context(context: ContextSpec, gate: HardwareGate) -> None:
    gate.require_ready()
    if type(context) is not ContextSpec:
        raise ProtocolError("Expected frozen context")
    for name in ("hardware_sha256", "model_sha256"):
        if context.definition[name] != getattr(gate, name):
            raise HardwareGateError(f"Context {name} is not covered by the passed gate")
    if context.context_id == "nominal":
        snapshot = json.loads(gate._verified_snapshot_json)
        contract = json.loads(Path(snapshot["contract_path"]).read_text(encoding="utf-8"))
        if context.definition["observation_horizon_s"] != contract["vehicle"]["nominal_observation_s"]:
            raise ProtocolError("Nominal tuning cannot silently shorten or extend the approved observation horizon")
        if context.definition["workload_profile"] != "motion-5min16":
            raise ProtocolError("Nominal tuning requires the frozen five-minute 16-bit raw workload")


def _workloads(context: ContextSpec, refs: Iterable[WorkloadRef], seeds: tuple[int, ...]) -> dict[int, WorkloadRef]:
    result: dict[int, WorkloadRef] = {}
    for ref in refs:
        if type(ref) is not WorkloadRef or ref.context_sha256 != context.context_sha256:
            raise ProtocolError("Workload belongs to a different or unfrozen context")
        if ref.seed not in seeds or ref.seed in result:
            raise ProtocolError("Unexpected/duplicate workload seed")
        result[ref.seed] = ref
    if set(result) != set(seeds):
        raise ProtocolError("The complete pre-exported workload set is required")
    return result


def protocol_manifest(gate: HardwareGate | None = None,
                      contexts: Iterable[ContextSpec] = ()) -> dict[str, Any]:
    gate = HardwareGate() if gate is None else gate
    specs = tuple(contexts)
    if any(type(context) is not ContextSpec for context in specs):
        raise ProtocolError("Manifest contexts must be frozen")
    if len({context.context_id for context in specs}) != len(specs):
        raise ProtocolError("Duplicate context identity")
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "status": "protocol_frozen_hardware_pending" if gate.status != "passed" else "hardware_evidence_supplied",
        "hardware_gate": gate.to_dict(),
        "candidates": [candidate.to_dict() for candidate in candidate_grid()],
        "phases": dict(PHASE_CAPS), "total_run_cap": TOTAL_CAP,
        "seeds": {"tuning": list(TRAIN_SEEDS), "confirmation": list(CONFIRM_SEEDS),
                  "transfer": list(TRANSFER_SEEDS)},
        "transfer_contexts": list(TRANSFER_CONTEXTS),
        "selection": {"metric_units": "fractions_0_to_1_and_joules",
                      "aggregation": "unweighted_mean_of_per_run_mature_cohort_ratios",
                      "limits": dict(LIMITS),
                      "order": ["all_runs_physically_feasible_and_mean_service_within_limits",
                                "immersed_energy_j", "full_mission_energy_j", "candidate_id"],
                      "infeasible_diagnostic": "minimum_max_positive_normalized_service_violation_among_physical_candidates",
                      "selection_data": "nominal_tuning_only_complete_195_runs",
                      "reselect_on_confirmation_or_transfer": False},
        "contexts": [context.to_dict() for context in sorted(specs, key=lambda item: item.context_id)],
        "machine_limits": {"workers": 2, "cpus": 2, "memory_gib": 6,
                           "pause_between_runs_s": 0.5, "minimum_start_free_bytes": 4294967296,
                           "per_run_stop_free_bytes": 1500000000,
                           "scope": "shared_envelope_at_most_two_isolated_workers"},
    }
    if gate.status == "passed":
        gate.require_ready()
        for context in specs:
            _validated_context(context, gate)
    manifest["manifest_sha256"] = content_sha256(manifest)
    return manifest


def build_tuning_jobs(context: ContextSpec, workloads: Iterable[WorkloadRef],
                      gate: HardwareGate) -> tuple[dict[str, Any], ...]:
    """Return descriptions only, gated; this module has no process launcher."""
    _validated_context(context, gate)
    if context.context_id != "nominal":
        raise ProtocolError("Only the nominal context can be used for tuning")
    refs = _workloads(context, workloads, TRAIN_SEEDS)
    jobs = []
    for candidate in candidate_grid():
        for seed in TRAIN_SEEDS:
            job = {"phase": "tuning", "candidate": candidate.to_dict(),
                   "context_sha256": context.context_sha256, "seed": seed,
                   "workload_sha256": refs[seed].workload_sha256,
                   "generated_ledger_sha256": refs[seed].generated_ledger_sha256,
                   "hardware_sha256": gate.hardware_sha256, "model_sha256": gate.model_sha256}
            job["job_sha256"] = content_sha256(job)
            jobs.append(job)
    validate_phase_counts({"tuning": len(jobs)})
    return tuple(jobs)


def validate_tuning_results(results: Iterable[RunResult], context: ContextSpec,
                            workloads: Iterable[WorkloadRef], gate: HardwareGate) -> tuple[RunResult, ...]:
    _validated_context(context, gate)
    if context.context_id != "nominal":
        raise ProtocolError("No per-context transfer re-optimization is allowed")
    refs = _workloads(context, workloads, TRAIN_SEEDS)
    expected = {(c.candidate_id, seed) for c in candidate_grid() for seed in TRAIN_SEEDS}
    found: dict[tuple[str, int], RunResult] = {}
    cohorts: dict[int, tuple[int, int, int]] = {}
    for result in results:
        if type(result) is not RunResult or result.phase != "tuning":
            raise ProtocolError("Selection accepts tuning rows only, never confirmation/transfer")
        key = (result.candidate_id, result.seed)
        if key not in expected or key in found:
            raise ProtocolError("Unexpected/duplicate candidate-seed row")
        if result.context_sha256 != context.context_sha256:
            raise ProtocolError("Mixed context identities in tuning results")
        for name in ("hardware_sha256", "model_sha256"):
            if getattr(result, name) != getattr(gate, name):
                raise ProtocolError(f"Result {name} differs from the frozen model")
        ref = refs[result.seed]
        if (result.workload_sha256 != ref.workload_sha256
                or result.generated_ledger_sha256 != ref.generated_ledger_sha256):
            raise ProtocolError("Unpaired workload or changed generated-cohort ledger")
        denominators = (result.metrics.high_mature_generated, result.metrics.medium_mature_generated,
                        result.metrics.low_windows_mature_generated)
        if result.seed in cohorts and cohorts[result.seed] != denominators:
            raise ProtocolError("Mature generated cohorts must match across candidates for each seed")
        cohorts[result.seed] = denominators
        found[key] = result
    if set(found) != expected:
        raise ProtocolError(f"Tuning grid is incomplete: expected 195 rows, received {len(found)}")
    validate_phase_counts({"tuning": len(found)})
    return tuple(found[(candidate.candidate_id, seed)]
                 for candidate in candidate_grid() for seed in TRAIN_SEEDS)


@dataclass(frozen=True)
class FrozenSelection:
    payload_json: str

    @property
    def payload(self) -> dict[str, Any]:
        return json.loads(self.payload_json)

    @property
    def selection_sha256(self) -> str:
        return content_sha256(self.payload)


def select_representatives(results: Iterable[RunResult], context: ContextSpec,
                           workloads: Iterable[WorkloadRef], gate: HardwareGate) -> FrozenSelection:
    ordered = validate_tuning_results(results, context, workloads, gate)
    candidates = candidate_grid()
    summaries = []
    for index, candidate in enumerate(candidates):
        rows = ordered[index * len(TRAIN_SEEDS):(index + 1) * len(TRAIN_SEEDS)]
        values = [row.metrics.run_values() for row in rows]
        means = {name: math.fsum(value[name] for value in values) / len(values)
                 for name in values[0]}
        physical = all(row.metrics.physically_feasible for row in rows)
        violation = max(0.0, *(means[name] / limit - 1.0 for name, limit in LIMITS.items()))
        summaries.append({"candidate": candidate.to_dict(), "n_runs": len(rows),
                          "mean_run_metrics": means, "all_runs_physically_feasible": physical,
                          "max_normalized_service_violation": violation,
                          "empirically_feasible": physical and violation <= 0.0})
    families = {}
    for family in FAMILIES:
        family_rows = [row for row in summaries if row["candidate"]["family"] == family]
        feasible = [row for row in family_rows if row["empirically_feasible"]]
        physical = [row for row in family_rows if row["all_runs_physically_feasible"]]
        def energy_key(row: dict[str, Any]) -> tuple[float, float, str]:
            metrics = row["mean_run_metrics"]
            return (metrics["immersed_energy_j"], metrics["full_mission_energy_j"], row["candidate"]["candidate_id"])
        if feasible:
            chosen = min(feasible, key=energy_key)
            families[family] = {"status": "empirically_feasible", "selected_candidate_id": chosen["candidate"]["candidate_id"],
                                "diagnostic_candidate_id": None}
        elif physical:
            chosen = min(physical, key=lambda row: (row["max_normalized_service_violation"], *energy_key(row)))
            families[family] = {"status": "no_service_feasible_candidate", "selected_candidate_id": None,
                                "diagnostic_candidate_id": chosen["candidate"]["candidate_id"]}
        else:
            families[family] = {"status": "no_physically_feasible_candidate", "selected_candidate_id": None,
                                "diagnostic_candidate_id": None}
    payload = {"schema_version": SCHEMA_VERSION, "selection_scope": "nominal_tuning_only",
               "context_sha256": context.context_sha256, "hardware_sha256": gate.hardware_sha256,
               "model_sha256": gate.model_sha256, "training_seeds": list(TRAIN_SEEDS),
               "training_results_sha256": content_sha256([asdict(row) for row in ordered]),
               "families": families, "all_candidates": summaries,
               "claim_scope": "empirical_selection_not_global_optimality_or_confirmed_feasibility"}
    return FrozenSelection(canonical_json(payload))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Create protocol JSON exclusively; an existing path is never overwritten")
    args = parser.parse_args(argv)
    rendered = json.dumps(protocol_manifest(), sort_keys=True, indent=2, allow_nan=False) + "\n"
    if args.output is None:
        print(rendered, end="")
    else:
        with args.output.open("x", encoding="utf-8") as handle:
            handle.write(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
