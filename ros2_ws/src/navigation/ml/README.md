# Map generation for weight experiments

Generate all initial map templates from the `navigation` directory:

```bash
python -m ml.generate_maps
```

Generate one reproducible template with a selected seed:

```bash
python -m ml.generate_maps --template large_aisles_human_busy --seed 12
```

Each template directory contains:

- `map.txt`: obstacles, start, and goal in `MAP v1` format
- `attributes.txt`: confidence, blockage, human, and robot probabilities in `ATTRIBUTES v2` format
- `metadata.json`: layout factors plus the generated human and robot trajectories

The initial templates vary layout, map size, human traffic, robot traffic, and map quality. A linear weight model is intentionally not implemented yet because it needs a labelled route score, such as delivery time plus penalties for collisions, human proximity, blockage, and rerouting.
