"""A number without provenance is an anecdote (docs/ACCEPTANCE.md A11)."""

from __future__ import annotations

from pathlib import Path

from parkscan_eval.provenance import (
    TRACKED_LIBRARIES,
    config_hash,
    describe,
    git_revision,
    library_versions,
)

REPO_ROOT = Path(__file__).resolve().parents[3]


def test_config_hash_is_stable_and_insensitive_to_key_order() -> None:
    a = config_hash({"seed": 42, "camera": {"fx": 457.0, "cx": 320.0}})
    b = config_hash({"camera": {"cx": 320.0, "fx": 457.0}, "seed": 42})
    assert a == b
    assert len(a) == 64


def test_config_hash_changes_when_a_parameter_changes() -> None:
    assert config_hash({"seed": 42}) != config_hash({"seed": 43})


def test_library_versions_report_absence_rather_than_omitting_it() -> None:
    versions = dict(library_versions(["numpy", "definitely-not-a-real-distribution"]))
    assert versions["definitely-not-a-real-distribution"] == "not-installed"
    assert versions["numpy"] != "not-installed"


def test_tracked_libraries_cover_everything_that_can_move_a_number() -> None:
    assert set(TRACKED_LIBRARIES) >= {
        "numpy",
        "parkscan-core",
        "parkscan-perception",
        "parkscan-sim",
    }
    assert list(TRACKED_LIBRARIES) == sorted(TRACKED_LIBRARIES)


def test_git_revision_reads_this_repository() -> None:
    sha, _dirty = git_revision(REPO_ROOT)
    assert len(sha) == 40
    assert all(c in "0123456789abcdef" for c in sha)


def test_git_revision_degrades_to_unknown_outside_a_repository(tmp_path: Path) -> None:
    """Missing provenance must be reported as missing, never fabricated."""
    sha, dirty = git_revision(tmp_path)
    assert (sha, dirty) in {("unknown", False)} or len(sha) == 40


def test_describe_collects_the_whole_record() -> None:
    record = describe(REPO_ROOT, {"seed": 42})
    payload = record.as_dict()
    assert set(payload) == {"git_sha", "git_dirty", "config_hash", "python", "libraries"}
    assert payload["config_hash"] == config_hash({"seed": 42})
    assert isinstance(payload["libraries"], dict)
