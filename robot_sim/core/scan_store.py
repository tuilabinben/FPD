"""The points one scan produced, and the CSV writer.

Copy of Scan/scanner/store.py, not an import: Scan/ is a separate program
with its own launcher, off this package's path. Same CSV columns on
purpose — either program's file drops into the same plot.
"""

import csv
import math

from ..config import SCAN_MISS_MM


class ScanStore:
    def __init__(self):
        self.clear()

    def clear(self):
        self.points = []          # (layer, deg, mm) -- mm < 0 means a miss
        self.layer_z = {}         # layer -> height in mm, from [SCAN_LAYER]
        self.range_mm = None      # frozen plot scale, see config

    # -- filling ------------------------------------------------------
    def add(self, layer, deg, mm):
        self.points.append((layer, deg, mm))

    def set_layer_z(self, layer, z_mm):
        self.layer_z[layer] = z_mm

    # -- reading back -------------------------------------------------
    def layer_points(self, layer):
        """HITS only. A miss drawn at radius 0 is a false wall."""
        return [(d, r) for (n, d, r) in self.points if n == layer and r >= 0]

    def layers(self):
        return sorted({n for (n, _d, _r) in self.points})

    @property
    def total(self):
        return len(self.points)

    @property
    def misses(self):
        return sum(1 for (_n, _d, r) in self.points if r < 0)

    def max_radius(self):
        hits = [r for (_n, _d, r) in self.points if r >= 0]
        return max(hits) if hits else 0.0

    # -- output -------------------------------------------------------
    def to_csv(self, path):
        """One row per point, misses marked. A miss is a finding — a file
        that omits it looks like a scan that never looked there."""
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["layer", "z_mm", "angle_deg", "distance_mm",
                        "x_mm", "y_mm", "hit"])
            for layer, deg, mm in self.points:
                z = self.layer_z.get(layer, "")
                hit = mm >= 0
                if hit:
                    rad = math.radians(deg)
                    x = f"{mm * math.cos(rad):.3f}"
                    y = f"{mm * math.sin(rad):.3f}"
                else:
                    x = y = ""
                w.writerow([layer, z, f"{deg:.2f}",
                            f"{mm:.2f}" if hit else f"{SCAN_MISS_MM:.2f}",
                            x, y, 1 if hit else 0])
        return len(self.points)
