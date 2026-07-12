"""Render the README hero image from the exported CAD geometry.

Rasterises the exterior part STLs in ``out/cad/stl/parts/`` with a small
self-contained z-buffer software renderer (numpy + Pillow — no OpenGL
context required, so it runs headless anywhere the STLs exist) and writes
``assets/vbat_render.png``: the vehicle standing tail-down in its VTOL
attitude, with a soft projected ground shadow.

Regenerate after a design change once ``out/cad/`` is fresh:

    python scripts/render_readme_cad.py

All numbers below are presentation choices (camera, palette, lighting),
not physics — the geometry itself comes exclusively from the STLs.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh
from PIL import Image, ImageDraw, ImageFilter

REPO = Path(__file__).resolve().parents[1]
PARTS_DIR = REPO / "out" / "cad" / "stl" / "parts"
OUT_PNG = REPO / "assets" / "vbat_render.png"

# Exterior parts only (internal frame/equipment are occluded anyway) with
# a flat-shaded base colour each: airframe skin in light grey, control
# surfaces in accent orange, propulsion/landing hardware in graphite.
SKIN = (232, 234, 237)
LID = (219, 223, 229)
ACCENT = (226, 88, 54)
GRAPHITE = (72, 77, 84)
DARK = (48, 51, 56)
METAL = (128, 133, 141)

PART_COLOURS: dict[str, tuple[int, int, int]] = {
    "fuselage_lower": SKIN,
    "fuselage_lid": LID,
    "wing_L": SKIN,
    "wing_R": SKIN,
    "aileron_L": ACCENT,
    "aileron_R": ACCENT,
    "duct": GRAPHITE,
    "centerbody": METAL,
    "strut_1": METAL,
    "strut_2": METAL,
    "strut_3": METAL,
    "strut_4": METAL,
    "vane_T": ACCENT,
    "vane_B": ACCENT,
    "vane_L": ACCENT,
    "vane_R": ACCENT,
    "leg_1": DARK,
    "leg_2": DARK,
    "leg_3": DARK,
    "leg_4": DARK,
    "prop_rotor": METAL,
    "servo_aileron_L": GRAPHITE,
    "servo_aileron_R": GRAPHITE,
    "battery_vents": GRAPHITE,
}

# Output raster: final size and supersampling factor for antialiasing.
WIDTH, HEIGHT = 1400, 1580
SS = 2

# Camera (world frame: Z up, vehicle standing on the Z=0 ground plane).
# The projection is auto-fitted to the canvas afterwards, so only the view
# direction and the perspective strength (distance factor) matter here.
CAM_AZIMUTH_DEG = 145.0  # around Z, measured from world +X
CAM_ELEVATION_DEG = 13.0
CAM_DISTANCE_FACTOR = 4.0  # of the vehicle bounding radius
FIT_MARGIN = 0.05  # canvas fraction kept clear around the fitted model

# Lighting: key + fill directions (world frame), Lambert weights, ambient.
KEY_LIGHT = np.array([-0.45, 0.55, 0.75])
FILL_LIGHT = np.array([0.7, -0.3, 0.25])
KEY_WEIGHT, FILL_WEIGHT, AMBIENT = 0.62, 0.22, 0.30
GAMMA = 1.9

SHADOW_ALPHA = 70  # 0-255
SHADOW_BLUR_PX = 9


def body_to_world(v: np.ndarray) -> np.ndarray:
    """Body FRD (x fwd out the nose, y right, z down) -> render world.

    The tail-sitter stands nose-up: world up is body +x. Right-handed
    world basis: Xw = y_body, Yw = z_body, Zw = x_body.
    """
    return np.column_stack([v[:, 1], v[:, 2], v[:, 0]])


def load_scene() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return (vertices per face (n,3,3), face normals (n,3), colours (n,3))."""
    tri_list, col_list = [], []
    for name, colour in PART_COLOURS.items():
        mesh = trimesh.load(PARTS_DIR / f"{name}.stl")
        tris = body_to_world(mesh.vertices.view(np.ndarray))[mesh.faces]
        tri_list.append(tris)
        col_list.append(np.tile(np.array(colour, dtype=float), (len(mesh.faces), 1)))
    tris = np.concatenate(tri_list)
    colours = np.concatenate(col_list)
    # Rest the landing legs on the ground plane Z=0.
    tris[:, :, 2] -= tris[:, :, 2].min()
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    normals = np.cross(e1, e2)
    norm = np.linalg.norm(normals, axis=1, keepdims=True)
    normals /= np.maximum(norm, 1e-12)
    return tris, normals, colours


def camera(tris: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (eye position, world->camera rotation rows)."""
    centre = (tris.reshape(-1, 3).min(0) + tris.reshape(-1, 3).max(0)) / 2.0
    radius = np.linalg.norm(tris.reshape(-1, 3) - centre, axis=1).max()
    az, el = np.radians(CAM_AZIMUTH_DEG), np.radians(CAM_ELEVATION_DEG)
    direction = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    eye = centre + CAM_DISTANCE_FACTOR * radius * direction
    fwd = (centre - eye) / np.linalg.norm(centre - eye)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    rot = np.stack([right, up, fwd])
    return eye, rot


def project(points: np.ndarray, eye: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Perspective-project world points to (x, y, depth) in unfitted units."""
    cam = (points - eye) @ rot.T
    x = cam[..., 0] / cam[..., 2]
    y = -cam[..., 1] / cam[..., 2]
    return np.stack([x, y, cam[..., 2]], axis=-1)


def fit_transform(*screens: np.ndarray) -> tuple[float, np.ndarray]:
    """Scale+offset mapping the combined projected extents onto the canvas."""
    pts = np.concatenate([s.reshape(-1, 3)[:, :2] for s in screens])
    lo, hi = pts.min(0), pts.max(0)
    w, h = WIDTH * SS, HEIGHT * SS
    scale = (1.0 - 2.0 * FIT_MARGIN) * min(w / (hi[0] - lo[0]), h / (hi[1] - lo[1]))
    offset = np.array([w, h]) / 2.0 - scale * (lo + hi) / 2.0
    return scale, offset


def apply_fit(screen: np.ndarray, scale: float, offset: np.ndarray) -> np.ndarray:
    out = screen.copy()
    out[..., :2] = screen[..., :2] * scale + offset
    return out


def shade(normals: np.ndarray, colours: np.ndarray) -> np.ndarray:
    """Two-light Lambert shading with gamma-correct blending, per face."""
    key = KEY_LIGHT / np.linalg.norm(KEY_LIGHT)
    fill = FILL_LIGHT / np.linalg.norm(FILL_LIGHT)
    lam = AMBIENT + KEY_WEIGHT * np.maximum(normals @ key, 0.0) + FILL_WEIGHT * np.maximum(normals @ fill, 0.0)
    lin = (colours / 255.0) ** GAMMA * lam[:, None]
    return np.clip(lin ** (1.0 / GAMMA) * 255.0, 0.0, 255.0)


def rasterise(screen: np.ndarray, shaded: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Z-buffer rasterisation. Returns (RGB float image, coverage mask)."""
    w, h = WIDTH * SS, HEIGHT * SS
    zbuf = np.full((h, w), np.inf)
    img = np.zeros((h, w, 3))
    order = np.argsort(-screen[:, :, 2].min(axis=1))  # far-to-near cuts overdraw cost
    for i in order:
        p = screen[i]
        x0, x1 = int(np.floor(p[:, 0].min())), int(np.ceil(p[:, 0].max())) + 1
        y0, y1 = int(np.floor(p[:, 1].min())), int(np.ceil(p[:, 1].max())) + 1
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(x1, w), min(y1, h)
        if x0 >= x1 or y0 >= y1:
            continue
        xs = np.arange(x0, x1) + 0.5
        ys = np.arange(y0, y1) + 0.5
        gx, gy = np.meshgrid(xs, ys)
        d = (p[1, 0] - p[0, 0]) * (p[2, 1] - p[0, 1]) - (p[2, 0] - p[0, 0]) * (p[1, 1] - p[0, 1])
        if abs(d) < 1e-9:
            continue
        w1 = ((gx - p[0, 0]) * (p[2, 1] - p[0, 1]) - (p[2, 0] - p[0, 0]) * (gy - p[0, 1])) / d
        w2 = ((p[1, 0] - p[0, 0]) * (gy - p[0, 1]) - (gx - p[0, 0]) * (p[1, 1] - p[0, 1])) / d
        w0 = 1.0 - w1 - w2
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not inside.any():
            continue
        depth = w0 * p[0, 2] + w1 * p[1, 2] + w2 * p[2, 2]
        tile = zbuf[y0:y1, x0:x1]
        closer = inside & (depth < tile)
        tile[closer] = depth[closer]
        img[y0:y1, x0:x1][closer] = shaded[i]
    return img, np.isfinite(zbuf)


def shadow_points(tris: np.ndarray) -> np.ndarray:
    """Triangle vertices dropped onto the Z=0 plane along the key light."""
    key = KEY_LIGHT / np.linalg.norm(KEY_LIGHT)
    flat = tris.reshape(-1, 3).copy()
    t = flat[:, 2] / key[2]
    return (flat - t[:, None] * key).reshape(-1, 3, 3)


def ground_shadow(screen: np.ndarray) -> Image.Image:
    """Soft shadow mask from the fitted, projected shadow triangles."""
    mask = Image.new("L", (WIDTH * SS, HEIGHT * SS), 0)
    draw = ImageDraw.Draw(mask)
    for p in screen:
        draw.polygon([(p[0, 0], p[0, 1]), (p[1, 0], p[1, 1]), (p[2, 0], p[2, 1])], fill=SHADOW_ALPHA)
    return mask.filter(ImageFilter.GaussianBlur(SHADOW_BLUR_PX * SS))


def main() -> None:
    tris, normals, colours = load_scene()
    print(f"loaded {len(tris)} triangles from {len(PART_COLOURS)} parts")
    eye, rot = camera(tris)
    shaded = shade(normals, colours)
    model_screen = project(tris, eye, rot)
    shadow_screen = project(shadow_points(tris), eye, rot)
    scale, offset = fit_transform(model_screen, shadow_screen)
    img, covered = rasterise(apply_fit(model_screen, scale, offset), shaded)
    print("rasterised")

    canvas = Image.new("RGBA", (WIDTH * SS, HEIGHT * SS), (0, 0, 0, 0))
    canvas.putalpha(ground_shadow(apply_fit(shadow_screen, scale, offset)))
    model = np.concatenate([img, np.where(covered, 255.0, 0.0)[..., None]], axis=-1)
    canvas = Image.alpha_composite(canvas, Image.fromarray(model.astype(np.uint8), "RGBA"))
    canvas = canvas.resize((WIDTH, HEIGHT), Image.LANCZOS)

    bbox = canvas.getbbox()
    if bbox:  # trim empty margins, keep a small border
        pad = 24
        canvas = canvas.crop((max(bbox[0] - pad, 0), max(bbox[1] - pad, 0),
                              min(bbox[2] + pad, WIDTH), min(bbox[3] + pad, HEIGHT)))
    OUT_PNG.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(OUT_PNG)
    print(f"wrote {OUT_PNG} ({canvas.width}x{canvas.height})")


if __name__ == "__main__":
    main()
