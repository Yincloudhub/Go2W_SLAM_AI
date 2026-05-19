# Git Agent Workflow

This repository uses short-lived agent branches for robot/LLM integration work.

## Branches

- `main`: stable project baseline.
- `agent/llm-on-robot`: current Codex agent branch for local LLM planner, robot-side deployment notes, eval data, and guardrail work.

Future agent branches should use:

```text
agent/<scope>
```

Examples:

```text
agent/cpp-plan-executor
agent/robot-llm-runtime
agent/map-topology-data
```

## Commit Rules

- Keep model weights, recovered backups, logs, eval run artifacts, build directories, and cache files out of git.
- Commit source, schemas, prompts, configs, docs, small eval/SFT datasets, and tests.
- Use clear commit messages:

```text
Add robot-side LLM runtime checklist
Harden local planner guardrails
Add floorplan v4 auto weak-network eval cases
```

## Push

Remote:

```text
origin https://github.com/Yincloudhub/Go2W_SLAM_AI.git
```

Push current branch:

```bash
git push -u origin agent/llm-on-robot
```

If HTTPS authentication is not configured on the machine, log in with Git Credential Manager or switch the remote to SSH after adding an SSH key to GitHub.

## Merge

Open a pull request from `agent/llm-on-robot` into `main` after:

- tests pass locally;
- robot-side dry-run is documented;
- large generated artifacts are not included;
- deployment notes are up to date.
