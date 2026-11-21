# Sokoban Solver

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/vnz347/sokoban-solver/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/vnz347/sokoban-solver/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

Push-optimal Sokoban solver for the [make.fun bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121). Every level is a hallway with rooms off it, and a box pushed out of a room is lost, so a level's optimum is the sum of its rooms': each room is solved on its own, push-optimally, with A* over push-states, a Hungarian box-to-goal assignment heuristic, dead-square and frozen-2x2 pruning, and a push-BFS fallback. Deterministic: all budgets are work counts, never the clock. Practice set: 1,000,000 (every instance at P*).

## Run it

```sh
node main.ts
```

Reads `/input/<id>/level.txt`, writes `/output/<id>/solution.txt` (player moves in LURD). `MAKEFUN_INPUT_DIR` / `MAKEFUN_OUTPUT_DIR` override the paths. Node 22, no dependencies.

<sub>Entered on [make.fun](https://make.fun/?ref=qa6gng8g&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
