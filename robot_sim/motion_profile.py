"""Angular motion profiles — trapezoidal, S-curve, pure S-curve.

A port of `Compare_Angular_Motion_Profiles.m`, kept faithful to it on
purpose: the .m is the reference the profiles were designed against, and
the two must agree or the graph in Settings is drawing something the
report does not describe. Symbol names mirror that file for eyeball
diffing — `Theta`, `omegaMax`, `alphaMax`, `Vp`, `tJ`, `tA`, `tV`, `J`.

Plain maths, no tkinter, like kinematics.py — so the settings dialog, the
tests and anything else can import it without pulling in a GUI.

WHAT THE THREE ARE, and why anyone cares:

* TRAPEZOIDAL ramps at a constant acceleration, holds speed, ramps down.
  Acceleration therefore STEPS between three values, and a step in
  acceleration is infinite jerk. That is what a stepper hears as a bang,
  and on an open-loop drive with no encoder it is where steps get lost.
* S-CURVE spends `rS` of the ramp easing the acceleration in and out, so
  jerk is bounded. Seven phases: +J, hold, -J, cruise, -J, hold, +J.
* PURE S-CURVE is the same generator with rS = 1, where the constant
  acceleration phase disappears entirely and the ramp is nothing but the
  two eases. Smoothest, and the slowest of the three for a given limit.

The trade is time: a bounded jerk cannot reach the same average
acceleration, so a smoother profile takes longer over the same angle.
The summary each generator returns carries the total, so the panel can
say so rather than implying smoothness is free.
"""

import math

# Shape parameter for the NORMAL S-curve, from the .m. It is the fraction
# of the acceleration ramp spent easing:
#
#   rS -> 0    approaches trapezoidal
#   rS = 0.5   the .m's worked example, and the default here
#   rS = 1     pure S-curve, which is its own profile below
#
# Deliberately not a field. It is one number whose two interesting values
# are already two of the three menu entries, and a third box that silently
# turns one profile into another is a worse question than the menu.
SCURVE_RATIO = 0.5


class Profile:
    """One generated profile. `t/s/v/a/j` are the sampled curves."""

    def __init__(self, t, s, v, a, j, **summary):
        self.t = t
        self.s = s
        self.v = v
        self.a = a
        self.j = j
        self.__dict__.update(summary)

    @property
    def peak_accel(self):
        return max(abs(x) for x in self.a) if self.a else 0.0

    @property
    def peak_jerk(self):
        """None for trapezoidal, where it is infinite rather than large."""
        if self.j is None:
            return None
        return max(abs(x) for x in self.j) if self.j else 0.0


def _time_grid(total, samples):
    """`samples` points across [0, total], like the .m's linspace."""
    if total <= 0 or samples < 2:
        return [0.0]
    step = total / (samples - 1)
    return [k * step for k in range(samples)]


def trapezoidal(theta, omega_max, alpha_max, samples=400):
    """generateTrapezoidalAngular(). Triangular when the angle is short."""
    theta = abs(float(theta))
    if theta <= 0 or omega_max <= 0 or alpha_max <= 0:
        raise ValueError("theta, omegaMax and alphaMax must all be positive")

    # Angle needed to reach omegaMax and come back down again. Below it the
    # profile never gets there and is a triangle, not a trapezium.
    theta_min = omega_max * omega_max / alpha_max
    if theta >= theta_min:
        vp = omega_max
        ta = vp / alpha_max
        tv = theta / vp - ta
    else:
        vp = math.sqrt(theta * alpha_max)
        ta = vp / alpha_max
        tv = 0.0

    total = 2.0 * ta + tv
    theta_a = 0.5 * alpha_max * ta * ta      # angle covered by the ramp up

    t = _time_grid(total, samples)
    s, v, a = [], [], []
    for tk in t:
        if tk <= ta:
            a.append(alpha_max)
            v.append(alpha_max * tk)
            s.append(0.5 * alpha_max * tk * tk)
        elif tk <= ta + tv:
            tau = tk - ta
            a.append(0.0)
            v.append(vp)
            s.append(theta_a + vp * tau)
        else:
            tau = tk - (ta + tv)
            a.append(-alpha_max)
            v.append(vp - alpha_max * tau)
            s.append(theta_a + vp * tv + vp * tau - 0.5 * alpha_max * tau * tau)

    # The .m pins the final sample rather than leaving the round-off in it.
    if len(t) > 1:
        s[-1], v[-1], a[-1] = theta, 0.0, 0.0

    # j is None, NOT a list of zeros: trapezoidal jerk is infinite at the
    # three corners, and zeros would be a quieter lie than no answer.
    return Profile(t, s, v, a, None, Vp=vp, ta=ta, tv=tv, T=total,
                   kind="TRAPEZOIDAL")


def s_curve(theta, omega_max, alpha_max, r=SCURVE_RATIO, samples=400):
    """generateSCurveAngular(). r = 1 is the pure S-curve."""
    theta = abs(float(theta))
    if theta <= 0 or omega_max <= 0 or alpha_max <= 0:
        raise ValueError("theta, omegaMax and alphaMax must all be positive")
    if not (0.0 < r <= 1.0):
        raise ValueError("the S-curve ratio must satisfy 0 < r <= 1")

    # Can omegaMax be reached at all? The ramp is longer than a trapezoid's
    # by the factor (1+r), which is the whole cost of bounding the jerk.
    ta_max = (1.0 + r) * omega_max / alpha_max
    theta_min = omega_max * ta_max
    vp = omega_max if theta >= theta_min else math.sqrt(theta * alpha_max / (1.0 + r))

    t_j = r * vp / alpha_max                 # easing, jerk = +/-J
    t_a = (1.0 - r) * vp / alpha_max         # constant acceleration; 0 when r=1
    ramp = 2.0 * t_j + t_a

    t_v = theta / vp - ramp                  # cruise
    if t_v < 1e-12:
        t_v = 0.0

    jerk_mag = alpha_max / t_j if t_j > 0 else 0.0

    # The seven phases, exactly as the .m orders them.
    durations = [t_j, t_a, t_j, t_v, t_j, t_a, t_j]
    jerks = [jerk_mag, 0.0, -jerk_mag, 0.0, -jerk_mag, 0.0, jerk_mag]

    total = sum(durations)

    # State at the START of each phase, integrated forward once.
    n = len(durations)
    s0, v0, a0 = [0.0] * n, [0.0] * n, [0.0] * n
    for k in range(1, n):
        h, jk = durations[k - 1], jerks[k - 1]
        a0[k] = a0[k - 1] + jk * h
        v0[k] = v0[k - 1] + a0[k - 1] * h + 0.5 * jk * h * h
        s0[k] = (s0[k - 1] + v0[k - 1] * h + 0.5 * a0[k - 1] * h * h
                 + jk * h * h * h / 6.0)

    t_end, run = [], 0.0
    for d in durations:
        run += d
        t_end.append(run)
    t_start = [0.0] + t_end[:-1]

    t = _time_grid(total, samples)
    s, v, a, j = [], [], [], []
    for tn in t:
        # First phase that both CONTAINS this instant and actually exists.
        # The length test matters at r = 1, where the two constant-accel
        # phases are zero-length and would otherwise swallow the sample.
        k = next((i for i in range(n)
                  if tn <= t_end[i] + 1e-12 and durations[i] > 1e-14), n - 1)
        tau = min(max(tn - t_start[k], 0.0), durations[k])
        jk = jerks[k]
        j.append(jk)
        a.append(a0[k] + jk * tau)
        v.append(v0[k] + a0[k] * tau + 0.5 * jk * tau * tau)
        s.append(s0[k] + v0[k] * tau + 0.5 * a0[k] * tau * tau
                 + jk * tau * tau * tau / 6.0)

    if len(t) > 1:
        s[-1], v[-1], a[-1], j[-1] = theta, 0.0, 0.0, 0.0

    return Profile(t, s, v, a, j, Vp=vp, tJ=t_j, tA=t_a, tV=t_v, J=jerk_mag,
                   T=total, kind="PURE_SCURVE" if r >= 1.0 else "SCURVE")


def generate(kind, theta, omega_max, alpha_max, samples=400):
    """One entry point, keyed by the names the settings file stores."""
    if kind == "TRAPEZOIDAL":
        return trapezoidal(theta, omega_max, alpha_max, samples)
    if kind == "SCURVE":
        return s_curve(theta, omega_max, alpha_max, SCURVE_RATIO, samples)
    if kind == "PURE_SCURVE":
        return s_curve(theta, omega_max, alpha_max, 1.0, samples)
    raise ValueError(f"unknown motion profile {kind!r}")
