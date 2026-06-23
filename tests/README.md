# Tests

Run all tests from the repo root:

```
pytest tests/ -v
```

## Structure

```
tests/
├── test_kinematics.py   Unit tests for FK/IK (once written in src/control/)
├── test_protocol.py     Unit tests for PC↔ESP32 message packing/unpacking
└── conftest.py          Shared fixtures
```

## Rules

- One test file per source module.
- Test file name: `test_<module_name>.py`.
- Every public function in `src/control/` and `src/simulation/` needs at least one test.
- Tests must be deterministic, no random seeds, no time dependencies.
- Tests must not require hardware (ESP32 or motors). Hardware tests go in `scripts/` with a `_hardware_test` suffix.
