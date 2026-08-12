class Font:
    def __init__(self, root=None, **kw):
        self.kw = kw
    def measure(self, text):
        return int(len(text) * abs(self.kw.get("size", 10)) * 0.6)
    def metrics(self, what=None):
        m = {"linespace": int(abs(self.kw.get("size", 10)) * 1.4), "ascent": 10, "descent": 3}
        return m[what] if what else m
    def actual(self, opt=None):
        return self.kw.get(opt) if opt else dict(self.kw)
    def cget(self, k): return self.kw.get(k)
    def config(self, **kw): self.kw.update(kw)
def nametofont(name): return Font(size=10)
def families(root=None): return ("Segoe UI", "Consolas", "Arial", "Courier New")
