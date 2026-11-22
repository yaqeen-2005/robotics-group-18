# Training A* weights

## Current data

`training_data/factory_busy_v1.zip` is a reproducible compressed dataset for one 40 x 30 busy warehouse with storage aisles, synthetic human traffic, synthetic robot traffic, confidence values, and 5,000 reachable start/goal tasks.

The archive contains:

- `map.txt`: static occupancy map
- `attributes.txt`: four risk values per cell
- `metadata.json`: generated human and robot trajectories
- `tasks.jsonl`: start, goal, and unweighted shortest distance
- `TRAINING_DATA.md`: archive schema summary

Rebuild it with:

```bash
python -m ml.prepare_training_data --tasks 5000 --seed 18
```

## Train profiles

The trainer uses only the Python standard library. It samples candidate A* weights, plans routes on the archived tasks, measures route distance, blockage, human exposure, uncertainty, other-robot exposure, and turns, then keeps the lowest-scoring candidate for each objective.

```bash
python -m ml.train_weights
```

Generated editable files:

- `weights/balanced.json`: balanced distance and risk objective
- `weights/human_safe.json`: stronger preference for avoiding human activity
- `weights/training_report.json`: dataset, seed, task count, trials, and scores

The files are ordinary JSON and can be edited manually. Every profile must define exactly:

```text
distance
blockage
human
uncertainty
robot_encounter
turn
heuristic
```

Without a profile, A* uses `DEFAULT_WEIGHTS` at the top of `astar/a_star.py`. Currently those defaults match the balanced baseline. Select a profile explicitly during testing:

```bash
python -m astar.run tests/maps/demo_map.txt \
  --attributes tests/maps/demo_attributes.txt \
  --weights ml/weights/human_safe.json \
  --no-visual
```

## Limitations and future work

These profiles are trained on synthetic data and are starting values, not final experimental results. After integrated robot trials, replace or extend `tasks.jsonl` with measured delivery time, collision, waiting, rerouting, and human-distance outcomes, then run the trainer again.

Automatic selection of the best profile after each run is intentionally not implemented yet. A future selector can compare completed-run metrics and pass the chosen JSON profile to the planner without changing A*.
