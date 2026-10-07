"""Load a robot CAD export into a tree of named nodes with world-space geometry.

STEP (and IGES) go through OpenCASCADE via cascadio, which keeps part names and
can report analytic cylinders (wheel and gear radii, axle directions). Mesh
formats (STL, OBJ, GLB/glTF, PLY, 3MF, OFF) go through trimesh; an STL is one
anonymous mesh, so the studio has to work from shape alone.

Everything comes out in inches, in the CAD's own axes. frame.py works out which
way is up and which way is forward.
"""
from __future__ import annotations

import json
import pathlib
import re
import struct
import tempfile
from dataclasses import dataclass, field

import numpy as np
import trimesh

M_TO_IN = 39.37007874015748
STEP_EXT = {".step", ".stp"}
IGES_EXT = {".iges", ".igs"}
MESH_EXT = {".stl", ".obj", ".glb", ".gltf", ".ply", ".3mf", ".off"}
SUPPORTED = sorted(STEP_EXT | IGES_EXT | MESH_EXT)


@dataclass
class Cylinder:
    radius: float                    # in
    axis: np.ndarray                 # unit vector, world
    center: np.ndarray               # a point on the axis, world (in)


@dataclass
class Node:
    id: str
    name: str
    parent: str | None
    children: list[str] = field(default_factory=list)
    vertices: np.ndarray | None = None   # (n, 3) world inches, leaves only
    faces: np.ndarray | None = None
    cylinders: list[Cylinder] = field(default_factory=list)

    @property
    def is_leaf(self) -> bool:
        return self.vertices is not None


@dataclass
class Model:
    source: str
    format: str                          # "step", "iges", "mesh"
    nodes: dict[str, Node]
    root: str
    named: bool                          # did the file carry part names at all?

    def leaves(self, under: str | None = None):
        stack = [under or self.root]
        while stack:
            n = self.nodes[stack.pop()]
            if n.is_leaf:
                yield n
            stack.extend(reversed(n.children))

    def path(self, node_id: str) -> list[str]:
        out = []
        n = self.nodes.get(node_id)
        while n is not None and n.parent is not None:
            out.append(n.name)
            n = self.nodes.get(n.parent)
        return list(reversed(out))

    def bounds(self, under: str | None = None) -> np.ndarray:
        pts = [n.vertices for n in self.leaves(under) if len(n.vertices)]
        if not pts:
            return np.zeros((2, 3))
        allv = np.vstack(pts)
        return np.array([allv.min(0), allv.max(0)])

    def all_vertices(self) -> np.ndarray:
        pts = [n.vertices for n in self.leaves() if len(n.vertices)]
        return np.vstack(pts) if pts else np.zeros((0, 3))

    def triangle_count(self) -> int:
        return int(sum(len(n.faces) for n in self.leaves()))


# ---------------------------------------------------------------- STEP names
_ENTITY = re.compile(r"#(\d+)\s*=\s*([A-Z_0-9]+)\s*\((.*?)\)\s*;", re.S)
_STR = re.compile(r"'((?:[^']|'')*)'")
_REF = re.compile(r"#(\d+)")


def step_product_tree(text: str) -> dict:
    """Instance id/name -> product name, read straight from the STEP text.

    OpenCASCADE names a component after its NEXT_ASSEMBLY_USAGE_OCCURRENCE;
    when that has no name (common for sub-assemblies) the node comes out as a
    bare id like "3" and the product's real name is lost. This recovers it.
    """
    products, formations, defs, nauo = {}, {}, {}, []
    for m in _ENTITY.finditer(text):
        num, kind, args = int(m.group(1)), m.group(2), m.group(3)
        if kind == "PRODUCT":
            s = _STR.findall(args)
            products[num] = (s[1] if len(s) > 1 and s[1] else s[0] if s else "").replace("''", "'")
        elif kind.startswith("PRODUCT_DEFINITION_FORMATION"):
            refs = _REF.findall(args)
            if refs:
                formations[num] = int(refs[-1])
        elif kind == "PRODUCT_DEFINITION":
            refs = _REF.findall(args)
            if refs:
                defs[num] = int(refs[0])
        elif kind == "NEXT_ASSEMBLY_USAGE_OCCURRENCE":
            s = _STR.findall(args)
            refs = _REF.findall(args)
            if len(refs) >= 2:
                nauo.append((s[0] if s else "", s[1] if len(s) > 1 else "", int(refs[0]), int(refs[1])))
    by_id = {}
    for inst_id, inst_name, _parent, child in nauo:
        pname = products.get(formations.get(defs.get(child, -1), -1), "")
        by_id[inst_id] = pname
        if inst_name:
            by_id.setdefault(inst_name, pname)
    return {"by_id": by_id, "products": sorted(set(products.values()))}


# ---------------------------------------------------------------- loading
_CT = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
_NC = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def _read_glb(data: bytes) -> tuple[dict, bytes]:
    if data[:4] != b"glTF":
        raise ValueError("not a GLB file")
    jlen = struct.unpack("<I", data[12:16])[0]
    gj = json.loads(data[20:20 + jlen])
    rest = data[20 + jlen:]
    binary = rest[8:8 + struct.unpack("<I", rest[:4])[0]] if len(rest) >= 8 else b""
    return gj, binary


def _accessor(gj: dict, binary: bytes, idx: int) -> np.ndarray:
    a = gj["accessors"][idx]
    bv = gj["bufferViews"][a["bufferView"]]
    dt = np.dtype(_CT[a["componentType"]])
    n = _NC[a["type"]]
    off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
    stride = bv.get("byteStride", 0)
    if stride and stride != dt.itemsize * n:
        raw = np.frombuffer(binary, dtype=np.uint8, count=stride * a["count"], offset=off).reshape(a["count"], stride)
        return raw[:, :dt.itemsize * n].copy().view(dt).reshape(a["count"], n)
    return np.frombuffer(binary, dtype=dt, count=a["count"] * n, offset=off).reshape(a["count"], n) if n > 1 \
        else np.frombuffer(binary, dtype=dt, count=a["count"], offset=off)


def _node_matrix(nd: dict) -> np.ndarray:
    if "matrix" in nd:
        return np.array(nd["matrix"], dtype=float).reshape(4, 4).T     # glTF is column-major
    M = np.eye(4)
    if "scale" in nd:
        M = np.diag(list(nd["scale"]) + [1.0]) @ M
    if "rotation" in nd:
        x, y, z, w = nd["rotation"]
        M = trimesh.transformations.quaternion_matrix([w, x, y, z]) @ M
    if "translation" in nd:
        M[:3, 3] += nd["translation"]
    return M


def _from_glb(data: bytes, source: str, fmt: str, names: dict, scale: float) -> Model:
    """Our own GLB walk: names, hierarchy and the TM_brep_faces cylinder data
    cascadio writes all survive, duplicates and all (a robot has six identical
    "Motor Housing" parts, and they all matter)."""
    gj, binary = _read_glb(data)
    nodes: dict[str, Node] = {}
    root = "root"
    nodes[root] = Node(id=root, name=pathlib.Path(source).stem, parent=None)
    any_named = False

    def display(raw: str) -> str:
        if (not raw or re.fullmatch(r"\d+", raw)) and raw in names.get("by_id", {}):
            return names["by_id"][raw] or raw
        return raw

    def visit(i: int, parent: str, T_parent: np.ndarray):
        nonlocal any_named
        nd = gj["nodes"][i]
        T = T_parent @ _node_matrix(nd)
        nid = f"n{i}"
        nm = display(nd.get("name", ""))
        if nm and not re.fullmatch(r"\d*|mesh\d*|node\d*", nm):
            any_named = True
        node = Node(id=nid, name=nm or f"part {i}", parent=parent)
        nodes[nid] = node
        nodes[parent].children.append(nid)
        if "mesh" in nd:
            verts, faces, cyls = [], [], []
            base = 0
            for prim in gj["meshes"][nd["mesh"]].get("primitives", []):
                if prim.get("mode", 4) != 4 or "POSITION" not in prim.get("attributes", {}):
                    continue
                v = _accessor(gj, binary, prim["attributes"]["POSITION"]).astype(np.float64)
                f = _accessor(gj, binary, prim["indices"]).astype(np.int64).reshape(-1, 3) if "indices" in prim \
                    else np.arange(len(v)).reshape(-1, 3)
                verts.append(v)
                faces.append(f + base)
                base += len(v)
                ext = (prim.get("extensions") or {}).get("TM_brep_faces") or {}
                cyls += [c for c in ext.get("faces", []) if c and c.get("type") == "cylinder"]
            if verts:
                V = np.vstack(verts)
                node.vertices = trimesh.transformations.transform_points(V, T) * scale
                node.faces = np.vstack(faces)
                R = T[:3, :3]
                for c in cyls:
                    ax = R @ np.asarray(c["axis"], dtype=float)
                    n = np.linalg.norm(ax)
                    if n == 0:
                        continue
                    o = trimesh.transformations.transform_points(np.asarray([c["origin"]], dtype=float), T)[0]
                    node.cylinders.append(Cylinder(radius=float(c["radius"]) * scale * np.cbrt(abs(np.linalg.det(R))),
                                                   axis=ax / n, center=o * scale))
        for ch in nd.get("children", []):
            visit(ch, nid, T)

    scene = gj.get("scenes", [{}])[gj.get("scene", 0)] if gj.get("scenes") else {"nodes": list(range(len(gj["nodes"])))}
    for i in scene.get("nodes", []):
        visit(i, root, np.eye(4))
    return Model(source=source, format=fmt, nodes=nodes, root=root, named=any_named)


def load(path: str | pathlib.Path, linear_tol_mm: float = 0.4) -> Model:
    path = pathlib.Path(path)
    ext = path.suffix.lower()
    if ext not in SUPPORTED:
        raise ValueError(f"{path.name}: unsupported format; use one of {', '.join(SUPPORTED)}")

    if ext in STEP_EXT | IGES_EXT:
        import cascadio  # only needed for B-rep formats

        kwargs = dict(tol_linear=linear_tol_mm, tol_angular=0.5, include_brep=True, brep_types={"cylinder"})
        with tempfile.TemporaryDirectory() as d:
            glb = pathlib.Path(d) / "model.glb"
            if ext in IGES_EXT:
                if not hasattr(cascadio, "to_glb_bytes"):
                    raise ValueError("this cascadio build can't read IGES; export STEP instead")
                glb.write_bytes(cascadio.to_glb_bytes(str(path), file_type=cascadio.FileType.IGES, **kwargs))
            else:
                cascadio.step_to_glb(str(path), str(glb), **kwargs)
            data = glb.read_bytes()
        names = step_product_tree(path.read_text(errors="replace")) if ext in STEP_EXT else {}
        return _from_glb(data, str(path), "step" if ext in STEP_EXT else "iges", names, scale=M_TO_IN)

    if ext == ".glb":
        return _from_glb(path.read_bytes(), str(path), "mesh", {}, scale=M_TO_IN)

    # everything else: let trimesh read it, guess the units from the size
    scene = trimesh.load(str(path), force="scene")
    ext_m = float(np.max(scene.extents)) if len(scene.geometry) else 0.0
    units = getattr(scene, "units", None)
    scale = {"meters": M_TO_IN, "m": M_TO_IN, "millimeters": 1 / 25.4, "mm": 1 / 25.4,
             "inches": 1.0, "in": 1.0}.get(units) or _guess_scale(ext_m)
    nodes: dict[str, Node] = {"root": Node(id="root", name=path.stem, parent=None)}
    named = False
    k = 0
    for gname in scene.graph.nodes_geometry:
        T, geom = scene.graph[gname]
        mesh = scene.geometry[geom]
        if not hasattr(mesh, "faces"):
            continue
        nm = str(gname)
        is_named = not re.fullmatch(r"geometry_\d+|[\w.-]*\.(stl|obj|ply|off|3mf)|\d+", nm, re.I)
        named = named or is_named
        mesh = mesh.copy()
        mesh.apply_transform(T)
        # An STL is usually the whole robot as one mesh. Its separate solids
        # (each wheel, each motor) are still separate pieces of surface, so
        # split them back out; shape recognition needs them one at a time.
        bodies = [(np.asarray(mesh.vertices), np.asarray(mesh.faces))]
        if not is_named and len(mesh.faces) < 2_000_000:
            parts = split_bodies(np.asarray(mesh.vertices), np.asarray(mesh.faces))
            if 1 < len(parts) <= 5000:
                bodies = parts
        for v, f in bodies:
            nid = f"n{k}"
            k += 1
            label = nm if is_named and len(bodies) == 1 else f"body {k}"
            nodes[nid] = Node(id=nid, name=label, parent="root",
                              vertices=np.asarray(v, dtype=np.float64) * scale, faces=np.asarray(f, dtype=np.int64))
            nodes["root"].children.append(nid)
    return Model(source=str(path), format="mesh", nodes=nodes, root="root", named=named)


def split_bodies(vertices: np.ndarray, faces: np.ndarray) -> list[tuple[np.ndarray, np.ndarray]]:
    """Connected pieces of a triangle soup (faces joined by shared vertices),
    each re-indexed. Union-find, so no scipy/networkx needed."""
    parent = list(range(len(vertices)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, c in faces.tolist():
        ra, rb, rc = find(a), find(b), find(c)
        if ra != rb:
            parent[rb] = ra
        if ra != rc:
            parent[find(rc)] = find(ra)
    roots = np.array([find(f[0]) for f in faces.tolist()]) if len(faces) else np.zeros(0, int)
    out = []
    for r in np.unique(roots):
        fs = faces[roots == r]
        used, inv = np.unique(fs.ravel(), return_inverse=True)
        out.append((vertices[used], inv.reshape(-1, 3)))
    return out


def _guess_scale(extent: float) -> float:
    """Model units -> inches for formats that don't say (STL, OBJ...). A VRC
    robot is 12-24 in on its longest side, whatever the file thinks."""
    if extent <= 0:
        return 1.0
    if extent < 1.5:
        return M_TO_IN            # metres
    if extent <= 30:
        return 1.0                # inches
    if extent <= 80:
        return 1 / 2.54           # centimetres
    return 1 / 25.4               # millimetres


def vertex_normals(v: np.ndarray, f: np.ndarray) -> np.ndarray:
    """Area-weighted vertex normals (trimesh's own wants scipy)."""
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    vn = np.zeros_like(v, dtype=np.float64)
    for k in range(3):
        np.add.at(vn, f[:, k], fn)
    n = np.linalg.norm(vn, axis=1, keepdims=True)
    n[n == 0] = 1
    return vn / n


def export_glb(model: Model, path: str | pathlib.Path, transform: np.ndarray | None = None,
               colors: dict[str, tuple] | None = None) -> None:
    """Write the model as a GLB in metres (glTF's unit), optionally moved into
    the robot frame, one mesh per leaf named by node id so a viewer can map
    clicks back to parts."""
    scene = trimesh.Scene()
    for n in model.leaves():
        if n.vertices is None or not len(n.faces):
            continue
        v = n.vertices
        if transform is not None:
            v = trimesh.transformations.transform_points(v, transform)
        m = trimesh.Trimesh(vertices=v / M_TO_IN, faces=n.faces, vertex_normals=vertex_normals(v, n.faces), process=False)
        if colors and n.id in colors:
            m.visual.face_colors = colors[n.id]
        scene.add_geometry(m, node_name=str(n.id), geom_name=str(n.id))
    scene.export(str(path), include_normals=True)
