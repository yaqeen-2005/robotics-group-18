# Map generation and A* weight experiments

Generate all reproducible map templates:

```bash
python -m ml.generate_maps
```

Generate only the larger 40 x 30 busy factory map:

```bash
python -m ml.generate_maps --template training_factory_busy --seed 18
```

Each map contains `map.txt`, `attributes.txt`, and `metadata.json`. The four attributes are confidence, historical blockage probability, human presence probability, and other-robot encounter probability.

The prepared training archive and editable profiles are described in [TRAINING.md](TRAINING.md). The current profiles are deliberately selected by the caller; automatic profile selection after a run is future work.
