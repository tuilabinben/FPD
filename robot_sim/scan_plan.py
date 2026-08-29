"""Scan parameters: operator gives four numbers, this derives the rest.

    t = P / f                 seconds per slice
    w = sweep / t             deg/s RM must turn at
    step = sweep / P          degrees between samples
    lift = gap * (N - 1)      total ZM travel

50 Hz, 50 points, 330 deg sweep = 330 deg in 1 s. 100 points = 2 s.
Pure arithmetic — panel, log and tests read one source.
"""

from . import config as C


class ScanPlanError(ValueError):
    """Parameter outside what board accepts, with reason."""


def slice_seconds(points_per_slice, sample_hz):
    """Sensor delivers P points at f Hz."""
    return points_per_slice / sample_hz


def rot_speed_deg_s(sweep_deg, points_per_slice, sample_hz):
    """Speed that spends exactly that long on one slice."""
    return sweep_deg / slice_seconds(points_per_slice, sample_hz)


def deg_step(sweep_deg, points_per_slice):
    """Degrees between samples. P steps, so P+1 points — see point_count().

    NOT P-1: that makes the count exact and the speed wrong, and speed is
    what the sensor must keep up with.
    """
    return sweep_deg / points_per_slice


def point_count(points_per_slice):
    """P plus the sample taken at the start angle, before RM moves."""
    return int(points_per_slice) + 1


def z_travel_mm(slices, gap_mm):
    """Lift moves only BETWEEN slices: N slices, N-1 steps."""
    return gap_mm * max(int(slices) - 1, 0)


def machine_rot_speed_deg_s(settings):
    """RM's real speed now. Mirrors applyMotionParams(), ceiling included,
    so the panel warns with the number the board clamps to."""
    pct = float(settings.get("rot_pct", C.DEFAULT_ROT_PCT))
    return min(C.rot_speed_deg_s(C.MASTER_RPM, pct), C.ROT_VEL_MAX_DEG_S)


def plan(sample_hz, points_per_slice, slices, gap_mm, sweep_deg,
         settings=None, start_z_mm=0.0):
    """Derivation dict, or ScanPlanError naming the bad field.

    `warnings` do NOT stop a scan — ZM ceiling and speed clamp are the
    operator's own limits. Refusals here are only what the board refuses.
    """
    def number(value, label):
        try:
            return float(str(value).strip().replace(",", "."))
        except (TypeError, ValueError, AttributeError):
            raise ScanPlanError(f"“{label}” has to be a number.")

    sample_hz = number(sample_hz, "Sample rate")
    points = number(points_per_slice, "Points per slice")
    slices = number(slices, "Slices")
    gap_mm = number(gap_mm, "Slice spacing")
    sweep_deg = number(sweep_deg, "Sweep")

    if not (C.SCAN_SAMPLE_HZ_MIN <= sample_hz <= C.SCAN_SAMPLE_HZ_MAX):
        raise ScanPlanError(
            f"The sample rate must be between {C.SCAN_SAMPLE_HZ_MIN:g} and "
            f"{C.SCAN_SAMPLE_HZ_MAX:g} Hz — you entered {sample_hz:g}.")
    if points != int(points) or not (C.SCAN_POINTS_MIN <= points <= C.SCAN_POINTS_MAX):
        raise ScanPlanError(
            f"Points per slice must be a whole number between "
            f"{C.SCAN_POINTS_MIN} and {C.SCAN_POINTS_MAX} — you entered "
            f"{points:g}.")
    if slices != int(slices) or not (C.SCAN_SLICES_MIN <= slices <= C.SCAN_SLICES_MAX):
        raise ScanPlanError(
            f"Slices must be a whole number between {C.SCAN_SLICES_MIN} and "
            f"{C.SCAN_SLICES_MAX} — you entered {slices:g}.")
    if gap_mm < C.SCAN_SLICE_GAP_MIN_MM:
        raise ScanPlanError(
            f"The slice spacing must be at least {C.SCAN_SLICE_GAP_MIN_MM:g} mm "
            f"— you entered {gap_mm:g} mm.")
    if not (C.SCAN_SWEEP_MIN_DEG <= sweep_deg <= C.SCAN_SWEEP_MAX_DEG):
        raise ScanPlanError(
            f"The sweep must be between {C.SCAN_SWEEP_MIN_DEG:g} and "
            f"{C.SCAN_SWEEP_MAX_DEG:g}° — the whole travel of the turntable "
            f"— you entered {sweep_deg:g}°.")

    points, slices = int(points), int(slices)
    step = deg_step(sweep_deg, points)
    if not (C.SCAN_DEG_STEP_MIN <= step <= C.SCAN_DEG_STEP_MAX):
        raise ScanPlanError(
            f"{points} points over {sweep_deg:g}° is a {step:.3f}° step, and "
            f"the board accepts {C.SCAN_DEG_STEP_MIN:g}–{C.SCAN_DEG_STEP_MAX:g}°. "
            f"Change the point count or the sweep.")

    seconds = slice_seconds(points, sample_hz)
    speed = rot_speed_deg_s(sweep_deg, points, sample_hz)
    lift = z_travel_mm(slices, gap_mm)
    settings = settings or {}
    max_z = float(settings.get(C.SCAN_MAX_Z_KEY, C.DEFAULT_SCAN_MAX_Z_MM))
    machine_speed = machine_rot_speed_deg_s(settings)

    warnings = []
    if lift > max_z:
        warnings.append(
            f"{slices} slices {gap_mm:g} mm apart move ZM {lift:.1f} mm, past "
            f"the {max_z:g} mm ceiling set in Settings → Scan. Lower the "
            f"spacing or the slice count, or raise the ceiling.")
    top_z = start_z_mm + lift
    if top_z > C.D1_MAX_MM:
        warnings.append(
            f"Starting at Z {start_z_mm:.1f} mm, the last slice sits at "
            f"{top_z:.1f} mm — past the {C.D1_MAX_MM:g} mm stroke. The board "
            f"will refuse this.")
    if speed > machine_speed:
        achievable = machine_speed / step
        warnings.append(
            f"{speed:.1f}°/s is faster than RM's {machine_speed:.1f}°/s at the "
            f"current speed percentages, so the board will clamp it: the "
            f"points land at the same angles but the sensor is only asked for "
            f"{achievable:.1f} Hz, and each slice takes "
            f"{sweep_deg / machine_speed:.1f} s instead of {seconds:.1f} s.")

    return {
        "sample_hz": sample_hz,
        "points_per_slice": points,
        "points_in_slice": point_count(points),
        "slices": slices,
        "gap_mm": gap_mm,
        "sweep_deg": sweep_deg,
        "deg_step": step,
        "slice_seconds": seconds,
        "rot_deg_s": speed,
        "machine_rot_deg_s": machine_speed,
        "z_travel_mm": lift,
        "z_travel_max_mm": max_z,
        "z_travel_over": lift > max_z,
        "top_z_mm": top_z,
        # Sweeps only: seek and lifts depend on where machine starts.
        "sweep_seconds_total": seconds * slices,
        "total_points": point_count(points) * slices,
        "warnings": warnings,
    }


def summary(p):
    """One line for the log."""
    return (f"{p['slices']} slices · {p['points_in_slice']} points each · "
            f"{p['deg_step']:.2f}° step over {p['sweep_deg']:g}° · "
            f"RM {p['rot_deg_s']:.1f}°/s · {p['slice_seconds']:.2f} s a slice · "
            f"ZM {p['z_travel_mm']:.1f} mm total")

