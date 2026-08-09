from . import Widget, Frame, Label, Button, Entry, Scrollbar  # noqa: F401
class Style(Widget):
    def configure(self, *a, **k): pass
    def theme_use(self, *a): pass
    def map(self, *a, **k): pass
    def layout(self, *a, **k): return []
class Notebook(Widget):
    def add(self, child, **kw): pass
    def select(self, *a): return ""
    def tab(self, *a, **k): pass
class Combobox(Entry):
    def set(self, v): self.opts["_text"] = v
    def current(self, i=None): return 0
class Progressbar(Widget):
    def start(self, *a): pass
    def stop(self): pass
    def step(self, *a): pass
class Separator(Widget): pass
class Treeview(Widget):
    def insert(self, *a, **k): return "I001"
    def delete(self, *a): pass
    def get_children(self, *a): return []
