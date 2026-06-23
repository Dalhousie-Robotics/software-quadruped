# Contributing to Dal Robotics Quadruped

Welcome. Read this before writing any code or opening any PR.

---

## Table of Contents

1. [Branch Naming](#branch-naming)
2. [Commit Messages](#commit-messages)
3. [Pull Request Process](#pull-request-process)
4. [Code Review Standards](#code-review-standards)
5. [Code Style](#code-style)
6. [Testing Requirements](#testing-requirements)
7. [Reporting Bugs](#reporting-bugs)

---

## Branch Naming

Always branch from `develop`, never from `main`.

```
feature/<short-description>     New functionality
fix/<short-description>         Bug fix
docs/<short-description>        Documentation only
refactor/<short-description>    Code restructure, no behavior change
test/<short-description>        Adding or fixing tests
```

Examples:
```
feature/ak40-can-driver
fix/ik-singularity-edge-case
docs/esp32-setup-guide
```

---

## Commit Messages

Use [Conventional Commits](https://www.conventionalcommits.org/) format:

```
<type>(<scope>): <short summary>

[optional body]
```

Types: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`

Examples:
```
feat(firmware): add MIT CAN protocol encoder/decoder
fix(kinematics): correct sign error in hip abduction IK
docs(onboarding): add PlatformIO Windows setup steps
test(kinematics): add FK/IK round-trip tests for all legs
```

Rules:
- Summary is imperative tense, lowercase, no period
- Keep the summary under 72 characters
- Add a body if the reason for the change is not obvious

---

## Pull Request Process

1. **Open an issue first** for anything non-trivial. PRs without a linked issue will be returned.
2. **Create a branch** from `develop` following naming conventions above.
3. **Write tests** for any new logic (see Testing Requirements).
4. **Self-review** your diff before requesting review, read every line.
5. **Open the PR** against `develop`, fill in the PR template completely.
6. **Request review** from the Software Lead + at least one other team member.
7. **Address all review comments** before merging. Resolve conversations only after the reviewer confirms.
8. **Squash and merge** to keep `develop` history clean.

No one merges their own PR. No exceptions.

---

## Code Review Standards

**As a reviewer:**
- Review within 48 hours of being assigned.
- Comment on logic, correctness, and clarity, not style (CI handles style).
- Use GitHub's suggestion feature for small fixes.
- Approve only when you would be comfortable owning that code.
- Be specific: "this will fail if velocity is negative because..." not "this looks wrong".

**As an author:**
- Respond to every comment, even if just "done" or "disagree because...".
- Do not push unrelated changes after review starts.

---

## Code Style

**Python:**
- Formatter: `ruff format` (enforced by CI)
- Linter: `ruff check` (enforced by CI)
- Type hints on all public functions
- No commented-out code in PRs

**C++ (Firmware):**
- Formatter: `clang-format` with `.clang-format` in repo root
- Naming: `snake_case` for variables/functions, `PascalCase` for classes, `SCREAMING_SNAKE_CASE` for constants
- No dynamic memory allocation (`new`/`malloc`) in control loops

---

## Testing Requirements

| Code type | Minimum requirement |
|---|---|
| Python math/algorithms (kinematics, control) | Unit tests with `pytest` |
| Python comms/serial | Integration test or manual test documented in PR |
| ESP32 firmware | PlatformIO unit test OR manual bench test documented in PR |
| Simulation scripts | Run to completion without error |

Tests live in `tests/` and mirror the `src/` structure.

---

## Reporting Bugs

Use the Bug Report issue template. Include:
- What you expected to happen
- What actually happened
- Steps to reproduce (exact commands, exact hardware state)
- Relevant logs or error messages

Do not open a bug report for something you haven't tried to reproduce.
