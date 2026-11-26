# Isaac Sim tile-map conversion

All ten `decoy_0000`–`decoy_0009` JSON sidecars from `experiments/worlds/suites/decoy1000/part0/maps/` were converted to `sample_usd/` and reopened with Isaac Sim's bundled OpenUSD. Tile JSON is the source even when a sibling SDF exists. The ramp uses three Gazebo-style boxes for the rise, deck, and fall. The converter uses the authored ramp width when supplied and defaults to 0.66 m.

| Map | Marking polylines | Paint samples | Max paint centerline error (mm) | Max paint height error (mm) | Ramp top samples | Max ramp top height error (mm) | Result |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| decoy_0000 | 3 | 3395 | 0.0001 | <0.0001 | 105 | 0.000002 | PASS |
| decoy_0001 | 4 | 3425 | 0.0001 | <0.0001 | 105 | 0.000002 | PASS |
| decoy_0002 | 3 | 2880 | 0.0001 | <0.0001 | 105 | 0.000002 | PASS |
| decoy_0003 | 2 | 2685 | 0.0001 | <0.0001 | 105 | 0.000002 | PASS |
| decoy_0004 | 3 | 2865 | 0.0001 | <0.0001 | 105 | 0.000002 | PASS |
| decoy_0005 | 3 | 3585 | 0.0001 | <0.0001 | 105 | 0.000002 | PASS |
| decoy_0006 | 4 | 3085 | 0.0001 | <0.0001 | 105 | 0.000002 | PASS |
| decoy_0007 | 3 | 3275 | 0.0001 | <0.0001 | 0 | — | PASS |
| decoy_0008 | 3 | 3915 | 0.0001 | <0.0001 | 105 | 0.000002 | PASS |
| decoy_0009 | 4 | 3300 | 0.0001 | <0.0001 | 0 | — | PASS |

## Readback checks

- **Paint, 5 mm limit.** Verified two USD triangles per JSON segment across `painted` and `markings`, including start and finish T bars and decoy lines. Recovered each stripe's centerline from the two opposing edge vertices at either end. Compared five equally spaced samples per segment in world coordinates to corresponding points on the JSON polyline in 3D, after removing the authored 1 mm paint lift. Compared the same unlifted samples to the analytic ramp profile for the reported paint height error. The centerline check accounts for the 20 mm paint width, whose straight edges are 10 mm from the centerline.
- **Ramps, 2 mm limit.** Read each USD cube's size and local-to-world transform. Sampled the top face at 1%, 10%, 25%, 50%, 75%, 90%, and 99% of its length and at five lateral positions spanning 98% of the width. Compared the resulting XY coordinates with the JSON footprint, requiring error under 2 µm, and compared Z with `gen_cases.ramp_height`. For the two flat maps, verified zero ramp prims.
- **Legacy compatibility.** Compared USDA files from the original and updated converters byte for byte. Both the official track and a seed-42 hard `gen_world` JSON with two hills and zero obstacles matched.

The checks cover sampled geometry on these ten maps. Exact ramp seams, render appearance, and physics were outside the readback check. Isaac's shell wrapper printed a conda environment warning and conversion and verification completed without failures.

## Reproduce conversion

Run from the project root.

```bash
isaac=/home/rvlab/isaacsim
usd_lib=("$isaac"/extscache/omni.usd.libs-*)
export PYTHONPATH="${usd_lib[0]}" LD_LIBRARY_PATH="${usd_lib[0]}/bin"
export PYTHONDONTWRITEBYTECODE=1

for n in {0000..0009}; do
  "$isaac/python.sh" experiments/isaac/json_to_usd.py \
    "experiments/worlds/suites/decoy1000/part0/maps/decoy_${n}.json" \
    "experiments/isaac/sample_usd/decoy_${n}.usdc" || break
done
```
