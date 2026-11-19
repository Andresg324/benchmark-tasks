"""gripkit: measured YAM grip settings for a yam_arena task file (see GRIP-ENVELOPE.md for the numbers behind them).

What it does:
- `use_rubber_pads()` gives both YAM finger pads (links tip_left / tip_right, both arms) a rubber material at run
  time. The stock YAM pads have no physics material, so PhysX uses its default 0.5 / 0.5; real YAM pads are
  rubber-faced. The binding goes through the PhysX tensor API (`root_view.set_material_properties`, the same call
  Isaac Lab's material randomizer uses), so it works however the arm USD is instanced, and it is applied on the first
  control step, before any grasp. PhysX averages the two touching materials, so the pad-object friction becomes
  (pad + object) / 2.
- `grasp_tcp_z(...)`, `lift_ok(...)`, `max_hold_g(...)`: the grasp height and the measured limits, as plain numbers,
  for demo planning and gating ideas offline.

Import it from task.py next to yamkit (copy this file into the task folder):

    import gripkit
    gripkit.use_rubber_pads()          # before the simulator builds the scene; idempotent

Plain Python at import time; touches Isaac only inside the simulator, through yam_arena.scene's object_states hook.
"""

import importlib.abc
import importlib.util
import os
import sys

# -- materials ---------------------------------------------------------------------------------------------------

DEFAULT_PAD = (0.5, 0.5)  # PhysX default material: what the stock YAM pads get (no material bound)
RUBBER_PAD = (1.0, 0.8)  # static, dynamic: rubber pad faces (rubber on card/plastic/wood 0.6-1.2)
TIP_LINKS = ("tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")

_STATE = {"pads": None, "bound": 0, "hooked": False}


def pad_shape_ranges(link_paths, shapes_per_link, tips=TIP_LINKS):
    """(start, end) shape-index ranges of the fingertip links in an articulation's material buffer. Shapes are
    stored link by link in `link_paths` order, `shapes_per_link[i]` per link (same layout Isaac Lab's
    randomize_rigid_body_material relies on)."""
    out, start = [], 0
    for path, n in zip(link_paths, shapes_per_link):
        if str(path).rstrip("/").rsplit("/", 1)[-1] in tips:
            out.append((start, start + int(n)))
        start += int(n)
    return out


def _bind(env) -> int:
    """Set the pad material on every fingertip shape of both arms. Returns how many shapes were set."""
    import torch
    import warp as wp

    static, dynamic = _STATE["pads"]
    base = env.unwrapped
    n = 0
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError:
            continue
        view = art.root_view
        counts = [art._physics_sim_view.create_rigid_body_view(p).max_shapes for p in view.link_paths[0]]
        mats = wp.to_torch(view.get_material_properties()).clone()
        for a, b in pad_shape_ranges(view.link_paths[0], counts):
            mats[:, a:b, 0] = static
            mats[:, a:b, 1] = dynamic
            mats[:, a:b, 2] = 0.0
            n += b - a
        ids = torch.arange(mats.shape[0], dtype=torch.int32)
        view.set_material_properties(wp.from_torch(mats.contiguous(), dtype=wp.float32), wp.from_torch(ids, dtype=wp.int32))
    return n


def _patch(scene) -> None:
    if getattr(scene, "_gripkit", False):
        return
    scene._gripkit = True
    inner = scene.object_states

    def object_states(env, names):
        if _STATE["pads"] is not None and not _STATE["bound"]:
            _STATE["bound"] = _bind(env) or -1
            if os.environ.get("GRIPKIT_LOG"):  # probe log only
                print(f"gripkit: pad material {_STATE['pads']} bound to {_STATE['bound']} fingertip shapes", flush=True)
        return inner(env, names)

    scene.object_states = object_states


class _Hook(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None:
            return None
        self._inner = spec.loader
        spec.loader = self
        return spec

    def create_module(self, spec):
        return self._inner.create_module(spec)

    def exec_module(self, module):
        self._inner.exec_module(module)
        _patch_late(module)


def _patch_late(module) -> None:
    # yamkit (if imported) replaces object_states in its own hook; wrap after it so both run.
    yk = sys.modules.get("yamkit")
    if yk is not None and not getattr(module, "_yamkit", False):
        yk._patch(module)
    _patch(module)


def use_pads(static: float, dynamic: float) -> None:
    """Bind (static, dynamic) friction to both arms' finger pads on the first control step."""
    _STATE["pads"] = (float(static), float(dynamic))
    if _STATE["hooked"]:
        return
    _STATE["hooked"] = True
    if "yam_arena.scene" in sys.modules:
        _patch_late(sys.modules["yam_arena.scene"])
    else:
        sys.meta_path.insert(0, _Hook())


def use_rubber_pads() -> None:
    """The recommended setting: rubber pad faces (1.0 / 0.8)."""
    use_pads(*RUBBER_PAD)


def pads_from_env(var: str = "GRIP_PADS") -> str:
    """'rubber' (default) or 'default' (stock pads) from an environment variable; applies it and returns the name."""
    name = os.environ.get(var, "rubber")
    if name == "rubber":
        use_rubber_pads()
    elif name != "default":
        raise ValueError(f"{var} must be 'rubber' or 'default', got {name!r}")
    return name


# -- planning numbers (measured; GRIP-ENVELOPE.md) ---------------------------------------------------------------

PAD_TIP_PAST_TCP = 0.013  # yam_arena.constants
MIN_TIP_CLEARANCE = 0.0005  # pad tips this far above the table at the lowest grasp (0.5 mm worked on 1 mm plates)
GRIP_BELOW_TOP = 0.025  # tall objects: put the pad tips 2.5 cm below the top


def grasp_tcp_z(table_top: float, height: float) -> float:
    """TCP height for a top-down grasp of an object `height` tall standing on `table_top`: pad tips at mid-height
    (low objects) or 2.5 cm below the top (tall ones), never closer than 0.5 mm to the table."""
    tips = table_top + max(MIN_TIP_CLEARANCE, height / 2, height - GRIP_BELOW_TOP)
    return tips + PAD_TIP_PAST_TCP


# Hold limit (g) by grasp width (mm) for a 20 cm carry plus shake, filled from the probe (GRIP-ENVELOPE.md).
# Each entry: (width_mm, stock_pads_g, rubber_pads_g): the heaviest mass that held with < 5 mm slip.
# Only widths measured with BOTH pad settings and pads that did not sink in; > 20 mm is not trusted (see the doc).
HOLD_LIMIT_G = (
    (3, 300, 500),  # 3 mm fin: stock slips 8.4 mm / 21 deg at 500 g; rubber holds 500 g (1.9 mm), slips at 800
    (8, 800, 800),  # 8 mm: no slip at any tested mass (top of the sweep, not a limit)
    (20, 800, 800),  # 20 mm: same
)
MIN_MASS_G = 40  # 5 g parts 20 mm and wider were squeezed out by the fully closed grip (both pad settings)
MAX_TRUSTED_WIDTH_MM = 20


def max_hold_g(width_mm: float, rubber: bool = True) -> float:
    """Heaviest measured mass (g) that holds at this grasp width (linear between probed widths, 0 below the
    narrowest that held). Returns nan if the table is empty."""
    rows = sorted(HOLD_LIMIT_G)
    if not rows:
        return float("nan")
    k = 2 if rubber else 1
    if width_mm <= rows[0][0]:
        return rows[0][k] if width_mm == rows[0][0] else 0.0
    for a, b in zip(rows, rows[1:]):
        if width_mm <= b[0]:
            f = (width_mm - a[0]) / (b[0] - a[0])
            return min(a[k], b[k]) if f not in (0.0, 1.0) else (a[k] if f == 0.0 else b[k])
    return rows[-1][k]


def lift_ok(mass_g: float, width_mm: float, rubber: bool = True) -> bool:
    """True if the probe held this mass at this width (conservative: the lighter limit of the neighbours)."""
    if width_mm > MAX_TRUSTED_WIDTH_MM or (width_mm >= 20 and mass_g < MIN_MASS_G):
        return False
    lim = max_hold_g(width_mm, rubber)
    return lim == lim and mass_g <= lim


# -- fitted squeeze ------------------------------------------------------------------------------------------------

FINGER_OPEN = -0.04695  # yam_arena.constants: finger joint at full open (pads 93.9 mm apart); 0 = pads touching
GRIPPER_STIFFNESS = 5000.0  # N/m per finger drive (yam_arena.constants.GRIPPER); force capped at 50 N


def squeeze_target(width_mm: float, squeeze_mm: float = 4.0) -> float:
    """Finger joint target (m, negative = open) that closes the pads to `width_mm - squeeze_mm`, instead of fully
    closed. Each finger then pushes with about GRIPPER_STIFFNESS x squeeze/2 (4 mm -> 10 N per finger) instead of
    the 50 N cap, which drove the pads through wide objects and squeezed 5 g parts out (GRIP-ENVELOPE.md)."""
    gap = max(0.0, (width_mm - squeeze_mm) / 1000.0)
    return max(FINGER_OPEN, -gap / 2.0)


def squeeze_force_n(squeeze_mm: float) -> float:
    """Approximate pad force per finger (N) for a fitted squeeze."""
    return min(50.0, GRIPPER_STIFFNESS * squeeze_mm / 2000.0)
