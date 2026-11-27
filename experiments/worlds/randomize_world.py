#!/usr/bin/env python3
"""Create a domain-randomized copy of a generated line-track SDF world.

Usage:
    python3 experiments/worlds/randomize_world.py SRC.sdf OUT.sdf \
        --seed N [--preset default|wide]

The source SDF, STL and JSON are never modified.  OUT.sdf is written together
with OUT.json (a copy of the source sidecar plus a "randomization" object that
records every drawn value) and a copy of any relative STL mesh beside OUT, so
the relative mesh URI in the SDF still resolves.  All randomness comes from
``random.Random(seed)`` so a sample is exactly reproducible.

Presets: "default" draws 70% of samples close to the original look (small
jitter) and 30% from the full ranges; "wide" always uses the full ranges.
"""

import argparse
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from random import Random

MAT_CENTER = (1.65, 1.65)
MAT_SIZE = 3.3
GROUND_PALETTE = {
    "grey": (0.60, 0.63, 0.68),
    "wood": (0.45, 0.31, 0.19),
    "tile": (0.78, 0.72, 0.60),
}


def clamp01(v):
    return min(1.0, max(0.0, v))


def temp_to_rgb(kelvin):
    """Approximate black-body colour (Tanner Helland), RGB in 0..1."""
    t = kelvin / 100.0
    if t <= 66:
        r = 255.0
        g = 99.4708025861 * math.log(t) - 161.1195681661
    else:
        r = 329.698727446 * (t - 60.0) ** -0.1332047592
        g = 288.1221695283 * (t - 60.0) ** -0.0755148492
    if t >= 66:
        b = 255.0
    elif t <= 19:
        b = 0.0
    else:
        b = 138.5177312231 * math.log(t - 10.0) - 305.0447927307
    return [clamp01(r / 255.0), clamp01(g / 255.0), clamp01(b / 255.0)]


def fmt(values):
    return " ".join(f"{v:.6g}" for v in values)


def clamp_rgb(values):
    """SDF colours must stay in [0, 1]; brightness goes into <intensity> instead."""
    return [min(1.0, max(0.0, v)) for v in values]


def child(parent, tag):
    el = parent.find(tag)
    if el is None:
        el = ET.SubElement(parent, tag)
    return el


def set_text(parent, tag, values):
    child(parent, tag).text = fmt(values)


def material_of(visual):
    return child(visual, "material")


def set_material(mat, rgb, specular=0.0):
    set_text(mat, "ambient", [*rgb, 1.0])
    set_text(mat, "diffuse", [*clamp_rgb(rgb), 1.0])
    set_text(mat, "specular", [specular, specular, specular, 1.0])


def find_model(world, name):
    return next((m for m in world.findall("model") if m.get("name") == name), None)


def pick(mode, full_range, near_range):
    lo, hi = near_range if mode == "near" else full_range
    return rng.uniform(lo, hi)


def jitter(rgb, amount):
    return [clamp01(c * (1.0 + rng.uniform(-amount, amount))) for c in rgb]


def draw_scene(world, mode, rec):
    scene = child(world, "scene")
    ambient = pick(mode, (0.2, 0.8), (0.35, 0.45))
    background = (
        [pick(mode, (0.15, 0.9), (0.65, 0.75)) for _ in range(3)]
        if mode == "full"
        else [pick(mode, (0.15, 0.9), (0.65, 0.75))] * 3
    )
    set_text(scene, "ambient", [ambient] * 3 + [1.0])
    set_text(scene, "background", [*background, 1.0])
    rec["scene"] = {"ambient_grey": ambient, "background_rgb": background}


def draw_sun(world, mode, rec):
    sun = next((l for l in world.findall("light") if l.get("name") == "sun"), None)
    if sun is None:
        sun = next((l for l in world.findall("light")
                    if l.get("type") == "directional"), None)
    if sun is None:
        sun = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
        set_text(sun, "pose", [0, 0, 500, 0, 0, 0])

    elevation = pick(mode, (15.0, 80.0), (45.0, 60.0))
    azimuth = pick(mode, (0.0, 360.0), (70.0, 110.0))
    el, az = math.radians(elevation), math.radians(azimuth)
    direction = [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az),
                 -math.sin(el)]
    scale = pick(mode, (0.4, 1.6), (0.9, 1.1))
    kelvin = pick(mode, (3500.0, 7500.0), (5500.0, 6500.0))
    rgb = temp_to_rgb(kelvin)
    diffuse = clamp_rgb(rgb)                   # colour only; the scale is the light's <intensity>
    set_text(sun, "intensity", [scale])
    shadows = True if mode == "near" else rng.random() < 0.8

    set_text(sun, "direction", direction)
    set_text(sun, "diffuse", [*diffuse, 1.0])
    set_text(sun, "specular", [0.25 * scale] * 3 + [1.0])
    child(sun, "cast_shadows").text = "true" if shadows else "false"
    rec["sun"] = {
        "elevation_deg": elevation, "azimuth_deg": azimuth,
        "direction": direction, "diffuse_scale": scale,
        "colour_temperature_k": kelvin, "colour_rgb": rgb,
        "diffuse": diffuse, "cast_shadows": shadows,
    }
    return shadows


def draw_extra_lights(world, mode, rec):
    count = 0 if mode == "near" else rng.randint(0, 2)
    lights = []
    for i in range(count):
        kind = rng.choice(["point", "spot"])
        x = MAT_CENTER[0] + rng.uniform(-MAT_SIZE / 2, MAT_SIZE / 2)
        y = MAT_CENTER[1] + rng.uniform(-MAT_SIZE / 2, MAT_SIZE / 2)
        z = rng.uniform(0.8, 2.0)
        intensity = rng.uniform(0.3, 2.0)
        light_range = rng.uniform(1.0, 5.0)
        rgb = temp_to_rgb(rng.uniform(3000.0, 7000.0))
        rgb = [c * rng.uniform(0.6, 1.0) for c in rgb]

        light = ET.SubElement(world, "light", {"name": f"extra_light_{i}", "type": kind})
        set_text(light, "pose", [x, y, z, 0, 0, 0])
        child(light, "cast_shadows").text = "false"
        set_text(light, "intensity", [intensity])
        set_text(light, "diffuse", [*clamp_rgb(rgb), 1.0])
        set_text(light, "specular", [0.2, 0.2, 0.2, 1.0])
        att = child(light, "attenuation")
        set_text(att, "range", [light_range])
        set_text(att, "constant", [1.0])
        set_text(att, "linear", [0.1])
        set_text(att, "quadratic", [0.01])
        if kind == "spot":
            set_text(light, "direction", [0.0, 0.0, -1.0])
            spot = child(light, "spot")
            set_text(spot, "inner_angle", [0.3])
            set_text(spot, "outer_angle", [0.7])
            set_text(spot, "falloff", [1.0])
        lights.append({"name": f"extra_light_{i}", "type": kind,
                       "pose": [x, y, z, 0, 0, 0], "intensity": intensity,
                       "range": light_range, "diffuse": rgb})
    rec["extra_lights"] = lights


def draw_mat(world, mode, rec):
    model = find_model(world, "track")
    link = next((l for l in model.findall("link") if l.get("name") == "surface"),
                None) if model is not None else None
    if link is None:
        rec["mat"] = None
        return
    visuals = link.findall("visual")
    base = [0.82, 0.72, 0.55]
    first = material_of(visuals[0]).find("diffuse")
    if first is not None and first.text:
        base = [float(v) for v in first.text.split()[:3]]
    amount = 0.03 if mode == "near" else 0.10
    rgb = jitter(base, amount)
    specular = pick(mode, (0.0, 0.3), (0.0, 0.05))
    for visual in visuals:
        set_material(material_of(visual), rgb, specular)
    rec["mat"] = {"base_rgb": base, "diffuse_rgb": rgb,
                  "jitter_fraction": amount, "specular": specular}


def draw_line(world, mode, rec):
    model = find_model(world, "track")
    link = next((l for l in model.findall("link") if l.get("name") == "line"),
                None) if model is not None else None
    if link is None:
        rec["line"] = None
        return
    level = pick(mode, (0.003, 0.10), (0.003, 0.02))
    tint = [clamp01(level * (1.0 + rng.uniform(-0.2, 0.2))) for _ in range(3)]
    for visual in link.findall("visual"):
        set_material(material_of(visual), tint, 0.1)
    rec["line"] = {"black_level": level, "diffuse_rgb": tint}


def draw_ground(world, mode, rec):
    model = find_model(world, "room_floor")
    link = next(iter(model.findall("link")), None) if model is not None else None
    if link is None:
        rec["ground"] = None
        return
    name = rng.choice(list(GROUND_PALETTE)) if mode == "full" else "grey"
    base = GROUND_PALETTE[name]
    rgb = jitter(base, 0.03 if mode == "near" else 0.10)
    for visual in link.findall("visual"):
        set_material(material_of(visual), rgb, 0.0)
    rec["ground"] = {"kind": name, "base_rgb": base, "diffuse_rgb": rgb}


def copy_meshes(src, out):
    """Copy relative STL meshes beside OUT so relative URIs keep resolving."""
    tree = ET.parse(src)
    copied = []
    for mesh in tree.getroot().iter("mesh"):
        uri_el = mesh.find("uri")
        if uri_el is None:
            continue
        uri = (uri_el.text or "").strip()
        if not uri or "://" in uri:
            continue
        rel = Path(uri)
        if rel.is_absolute():
            continue
        source = src.parent / rel
        target = out.parent / rel
        if source.exists() and source.resolve() != target.resolve():
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            copied.append(str(target))
    return copied


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("src", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--preset", choices=["default", "wide"], default="default")
    args = parser.parse_args()

    global rng
    rng = Random(args.seed)
    src, out = args.src, args.out
    if out.resolve() == src.resolve():
        parser.error("OUT must differ from SRC (the source is never modified)")
    out.parent.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(src)
    world = tree.getroot().find("world")
    if world is None:
        parser.error(f"{src} has no <world> element")

    mode = "near" if args.preset == "default" and rng.random() < 0.7 else "full"
    rec = {"seed": args.seed, "preset": args.preset, "mode": mode,
           "source_sdf": str(src), "output_sdf": str(out)}

    draw_scene(world, mode, rec)
    shadows = draw_sun(world, mode, rec)
    draw_extra_lights(world, mode, rec)
    draw_mat(world, mode, rec)
    draw_line(world, mode, rec)
    draw_ground(world, mode, rec)
    scene = child(world, "scene")
    child(scene, "shadows").text = "true" if shadows else "false"

    copied = copy_meshes(src, out)
    rec["copied_meshes"] = copied

    ET.indent(tree, space="  ")
    tree.write(out, encoding="utf-8", xml_declaration=True)

    sidecar = src.with_suffix(".json")
    data = json.loads(sidecar.read_text()) if sidecar.exists() else {}
    data["randomization"] = rec
    out.with_suffix(".json").write_text(json.dumps(data, indent=2) + "\n")
    print(f"wrote {out} ({mode} mode), {out.with_suffix('.json')}")


if __name__ == "__main__":
    main()
