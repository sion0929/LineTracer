# Randomized line-track samples

Ten domain-randomized copies of `../decoy1000/part0/maps/decoy_0000.sdf`
generated with `experiments/worlds/randomize_world.py` (default preset):

```bash
for s in 0 1 2 3 4 5 6 7 8 9; do
  python3 experiments/worlds/randomize_world.py \
    experiments/worlds/suites/decoy1000/part0/maps/decoy_0000.sdf \
    experiments/worlds/suites/rand_sample/rand_$(printf '%04d' $s).sdf --seed $s
done
```

Each `rand_XXXX.sdf` is accompanied by `rand_XXXX.json` (source sidecar plus a
`randomization` object with every drawn value), and `decoy_0000.stl` is copied
beside them so the relative mesh URI `./decoy_0000.stl` still resolves.
In `default` mode 70% of runs use small jitter around the original look and
30% use the full randomization ranges; the table records which mode was drawn.

| seed | mode | sun elev° | sun az° | diffuse× | temp K | shadows | extra lights | ambient | background | mat RGB | mat spec | line black | ground |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | full | 41.3 | 282.2 | 0.76 | 5406 | yes | 0 (–) | 0.65 | 0.47,0.34,0.53 | 0.82,0.69,0.58 | 0.19 | 0.027 | grey |
| 1 | near | 48.8 | 89.8 | 0.99 | 6152 | yes | 0 (–) | 0.43 | 0.73,0.73,0.73 | 0.83,0.70,0.53 | 0.04 | 0.010 | grey |
| 2 | full | 62.8 | 241.1 | 0.77 | 5924 | yes | 2 (P, P) | 0.77 | 0.19,0.21,0.78 | 0.77,0.72,0.53 | 0.15 | 0.057 | tile |
| 3 | near | 54.1 | 95.0 | 0.91 | 5513 | yes | 0 (–) | 0.40 | 0.69,0.69,0.69 | 0.84,0.71,0.54 | 0.05 | 0.011 | grey |
| 4 | near | 47.3 | 72.7 | 0.98 | 6418 | yes | 0 (–) | 0.36 | 0.69,0.69,0.69 | 0.83,0.73,0.54 | 0.03 | 0.008 | grey |
| 5 | near | 59.1 | 99.6 | 1.08 | 5529 | yes | 0 (–) | 0.42 | 0.73,0.73,0.73 | 0.82,0.74,0.55 | 0.05 | 0.005 | grey |
| 6 | full | 58.1 | 169.3 | 1.31 | 4993 | yes | 1 (S) | 0.69 | 0.51,0.35,0.15 | 0.87,0.75,0.59 | 0.10 | 0.012 | grey |
| 7 | near | 46.1 | 91.4 | 0.97 | 5558 | yes | 0 (–) | 0.37 | 0.72,0.72,0.72 | 0.82,0.70,0.55 | 0.00 | 0.005 | grey |
| 8 | near | 55.6 | 73.4 | 0.95 | 6499 | yes | 0 (–) | 0.45 | 0.66,0.66,0.66 | 0.81,0.73,0.55 | 0.02 | 0.011 | grey |
| 9 | near | 58.0 | 70.3 | 1.00 | 6398 | yes | 0 (–) | 0.39 | 0.66,0.66,0.66 | 0.80,0.72,0.55 | 0.00 | 0.009 | grey |

All ten outputs were parsed back with `xml.etree.ElementTree` and their mesh
URI was checked to resolve to an existing file. The `gz sdf -k` check was
skipped because `gz` is not installed in this environment (it is expected in
the lab container).
