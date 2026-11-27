#!/usr/bin/env python3
"""Convert generated track JSON or Gazebo SDF to a standalone USD asset."""

import argparse
import json
import math
import re
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "worlds"))
from gen_world import BRINGUP, LINE_LIFT, LINE_WIDTH, MAT_COLOR, write_line_mesh
from gen_cases import ramp_height


TILE_RAMP_WIDTH = 0.66


def numbers(value):
    return tuple(map(float, value.split()))


def place(prim, pose):
    transform = UsdGeom.XformCommonAPI(prim)
    transform.SetTranslate(Gf.Vec3d(*pose[:3]))
    transform.SetRotate(Gf.Vec3f(*map(math.degrees, pose[3:])))


def material(stage, shape, rgba):
    # Gazebo diffuse values are used as authored. Photometric matching is done separately.
    key = "_".join(str(round(channel * 1000000)) for channel in rgba)
    path = f"/World/Materials/C_{key}"
    surface = UsdShade.Material.Get(stage, path)
    if not surface:
        surface = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, f"{path}/Preview")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgba[:3]))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(rgba[3])
        surface.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(surface)


def box(stage, path, size, pose, rgba):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.0)
    place(shape, pose)
    UsdGeom.XformCommonAPI(shape).SetScale(Gf.Vec3f(*size))
    material(stage, shape, rgba)


def line(stage, source, rgba=(0, 0, 0, 1)):
    with source.open("rb") as stream:
        _, count = struct.unpack("<80sI", stream.read(84))
        triangles = struct.iter_unpack("<12fH", stream.read())
        points = [
            triangle[offset:offset + 3]
            for triangle in triangles
            for offset in (3, 6, 9)
        ]
    if len(points) != 3 * count or not points:
        raise ValueError(f"Invalid binary STL: {source}")
    mesh = UsdGeom.Mesh.Define(stage, "/World/Track/Line")
    mesh.CreatePointsAttr(points)
    mesh.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(mesh.GetPointsAttr().Get()))
    mesh.CreateFaceVertexCountsAttr([3] * count)
    mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    material(stage, mesh, rgba)


def pose_value(value):
    return numbers(value) if value else (0.0,) * 6


def pose_matrix(pose):
    rotation = (Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(pose[3])) *
                Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(pose[4])) *
                Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(pose[5])))
    matrix = Gf.Matrix4d().SetRotate(rotation)
    matrix.SetTranslateOnly(Gf.Vec3d(*pose[:3]))
    return matrix


def box_matrix(stage, path, size, pose, rgba, parent=None):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.0)
    matrix = pose_matrix(pose)
    if parent is not None:
        matrix = matrix * parent
    scale = Gf.Matrix4d().SetScale(Gf.Vec3d(*size))
    UsdGeom.Xformable(shape).MakeMatrixXform().Set(scale * matrix)
    material(stage, shape, rgba)


def visual_name(name, index):
    if name.startswith("mat"):
        return "Mat"
    if name.startswith("ramp_"):
        return "Ramp_" + name.split("_")[1]
    return re.sub(r"[^A-Za-z0-9_]+", "_", name).strip("_") or f"box_{index}"


def generated_sdf(stage, source):
    world = ET.parse(source).getroot().find("world")
    room = world.find("model[@name='room_floor']")
    if room is not None:
        room_floor(stage, world)
    track = world.find("model[@name='track']")
    if track is None:
        raise ValueError(f"No inline track model: {source}")
    track_prim = UsdGeom.Xform.Define(stage, "/World/Track")
    place(track_prim, pose_value(track.findtext("pose")))
    for link in track.findall("link"):
        link_matrix = pose_matrix(pose_value(link.findtext("pose")))
        for index, visual in enumerate(link.findall("./visual[geometry]")):
            size = visual.findtext("geometry/box/size")
            name = visual.get("name") or f"visual_{index}"
            rgba = numbers(visual.findtext("material/diffuse") or "0.5 0.5 0.5 1")
            if size:
                box_matrix(stage, f"/World/Track/{visual_name(name, index)}", numbers(size),
                           pose_value(visual.findtext("pose")), rgba, link_matrix)
                continue
            uri = visual.findtext("geometry/mesh/uri")
            if uri:
                line(stage, source.parent / uri, rgba)
                line_prim = stage.GetPrimAtPath("/World/Track/Line")
                if line_prim:
                    visual_matrix = pose_matrix(pose_value(visual.findtext("pose")))
                    UsdGeom.Xformable(line_prim).MakeMatrixXform().Set(visual_matrix * link_matrix)
    robot = next((item for item in world.findall("include")
                  if item.findtext("uri") == "model://turtlebot3_waffle_pi"), None)
    if robot is None:
        raise ValueError(f"No robot include: {source}")
    return numbers(robot.findtext("pose"))


def rotated_ramp(stage, index, ramp, color):
    heading = ramp["heading"]
    cosine, sine = math.cos(heading), math.sin(heading)
    rise = ramp["height"]
    thickness = rise + 0.015
    width = ramp["width"]
    start = 0.0
    for part, (length, z0, z1) in enumerate((
        (ramp["up"], 0, rise),
        (ramp["deck"], rise, rise),
        (ramp["down"], rise, 0),
    )):
        if length <= 0:
            continue
        slope = math.atan2(z1 - z0, length)
        # Offset the box center so its top face meets the profile breakpoints.
        center = start + (length + math.sin(slope) * thickness) / 2
        pose = (ramp["x0"] + center * cosine, ramp["y0"] + center * sine,
                (z0 + z1 - math.cos(slope) * thickness) / 2, 0, -slope, heading)
        size = (math.hypot(length, z1 - z0), width, thickness)
        box(stage, f"/World/Track/Ramp_{index}_{part}", size, pose, color)
        start += length


def generated(stage, source, reference_line=None):
    data = json.loads(source.read_text())
    if data["obstacles"]:
        raise ValueError("Line-only scene conversion requires zero obstacles")
    if reference_line is not None:
        line(stage, reference_line)
    for key, expected in (("line_width_m", LINE_WIDTH), ("line_lift_m", LINE_LIFT)):
        if not math.isclose(data[key], expected):
            raise ValueError(f"Unsupported {key}: {data[key]}")
    ramps = data["ramps"] if "ramps" in data else [data["ramp"]]
    ramps = [
        dict(ramp, height=ramp["height_m"], width=ramp.get("width", TILE_RAMP_WIDTH))
        if "heading" in ramp else ramp
        for ramp in ramps
    ]
    color = numbers(MAT_COLOR)
    x0, y0, x1, y1 = data["mat_bounds"]
    box(stage, "/World/Track/Mat", (x1 - x0, y1 - y0, 0.1),
        ((x0 + x1) / 2, (y0 + y1) / 2, -0.05, 0, 0, 0), color)

    for index, ramp in enumerate(ramps):
        if "heading" in ramp:
            rotated_ramp(stage, index, ramp, color)
            continue
        xs, (low, high), rise = ramp["x"], ramp["y"], ramp["height_m"]
        thickness = rise + 0.015
        for part, (start, end, z0, z1) in enumerate(zip(
            xs, xs[1:], (0, rise, rise), (rise, rise, 0)
        )):
            slope = math.atan2(z1 - z0, end - start)
            size = (math.hypot(end - start, z1 - z0), high - low, thickness)
            pose = ((start + end + math.sin(slope) * thickness) / 2, (low + high) / 2,
                    (z0 + z1 - math.cos(slope) * thickness) / 2, 0, -slope, 0)
            box(stage, f"/World/Track/Ramp_{index}_{part}", size, pose, color)

    def height(x, y):
        result = 0.0
        for ramp in ramps:
            if "heading" in ramp:
                result = max(result, ramp_height(ramp, x, y))
                continue
            a, b, c, d = ramp["x"]
            if ramp["y"][0] <= y <= ramp["y"][1]:
                result = max(result, ramp["height_m"] *
                             max(0.0, min(1.0, (x - a) / (b - a), (d - x) / (d - c))))
        return result

    paths = data["painted"] if "painted" in data else [data["centerline"]]
    paths = [[tuple(point[:2]) for point in path] for path in (*paths, *data["markings"])]
    if reference_line is not None:
        return data["start_pose"]
    with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
        mesh = Path(directory) / "line.stl"
        write_line_mesh(mesh, paths, height)
        line(stage, mesh)
    return data["start_pose"]


def sdf_boxes(stage, model, parent):
    for index, visual in enumerate(model.findall("./link/visual[geometry]")):
        size = visual.findtext("geometry/box/size")
        if size:
            box(stage, f"{parent}/{visual.attrib['name']}_{index}", numbers(size),
                numbers(visual.findtext("pose", "0 0 0 0 0 0")),
                numbers(visual.findtext("material/diffuse")))


def room_floor(stage, world):
    floor = world.find("model[@name='room_floor']")
    place(UsdGeom.Xform.Define(stage, "/World/Floor"), numbers(floor.findtext("pose")))
    sdf_boxes(stage, floor, "/World/Floor")


def official(stage, source):
    world = ET.parse(source).getroot().find("world")
    room_floor(stage, world)
    include = next(item for item in world.findall("include") if item.findtext("uri") == "model://track")
    place(UsdGeom.Xform.Define(stage, "/World/Track"), numbers(include.findtext("pose")))
    models = source.parent.parent / "models"
    track = ET.parse(models / "track/model.sdf").getroot().find("model")
    sdf_boxes(stage, track, "/World/Track")
    uri = track.findtext("link[@name='line']/visual/geometry/mesh/uri")
    line(stage, models / uri.removeprefix("model://"))
    robot = next(item for item in world.findall("include")
                 if item.findtext("uri") == "model://turtlebot3_waffle_pi")
    return numbers(robot.findtext("pose"))


def convert(source, output, *, reference_line=None):
    stage = Usd.Stage.CreateInMemory()
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World/Track")
    if source.suffix == ".json":
        data = json.loads(source.read_text())
        ramps = data.get("ramps", [data.get("ramp", {})])
        tile_map = data.get("case", {}).get("group") == "tiles"
        has_rotated_ramps = any("heading" in ramp for ramp in ramps)
        if ("case" in data and not tile_map and not has_rotated_ramps
                and source.with_suffix(".sdf").is_file()):
            start_pose = generated_sdf(stage, source.with_suffix(".sdf"))
        else:
            template = ET.parse(BRINGUP / "worlds/line_track.sdf").getroot()
            room_floor(stage, template.find("world"))
            start_pose = generated(stage, source, reference_line)
    elif ET.parse(source).getroot().find("world/model[@name='track']") is not None:
        start_pose = generated_sdf(stage, source)
    else:
        start_pose = official(stage, source)
    stage.GetRootLayer().customLayerData = {"source": source.name}
    place(UsdGeom.Xform.Define(stage, "/World/StartPose"), start_pose)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not stage.GetRootLayer().Export(str(output)):
        raise OSError(f"Could not write {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="generated track JSON, Gazebo SDF, or 'official'")
    parser.add_argument("output", type=Path, help="USD output (.usd, .usda, or .usdc)")
    args = parser.parse_args()
    source = BRINGUP / "worlds/line_track.sdf" if args.source == "official" else Path(args.source)
    if source.suffix not in {".json", ".sdf"}:
        parser.error("source must be JSON or SDF")
    if args.output.suffix not in {".usd", ".usda", ".usdc"}:
        parser.error("output must end in .usd, .usda, or .usdc")
    for directory in (BRINGUP / "worlds", BRINGUP / "models"):
        if args.output.resolve().is_relative_to(directory):
            parser.error("output must be outside the original worlds and models")
    convert(source, args.output)


if __name__ == "__main__":
    main()
