"""Headless Tk stand-in, so the GUI's LOGIC can be tested without a display.

It is not a renderer. Widgets accept any option and record their tree;
StringVars really hold values; bindings are recorded. That is enough to
exercise every code path in the dialogs, which is where the bugs live.

Deliberate details, each one paid for by a real bug:
  * __getitem__ raises KeyError for unknown options, so `"x" in widget`
    terminates instead of looping forever on the iteration fallback.
  * PhotoImage exists, because the rounded-corner renderer holds refs.
  * winfo_ismapped()/place()/pack_info() exist, because the tab logic asks.
"""


class TclError(Exception):
    pass


CALLS = []


class Variable:
    def __init__(self, master=None, value=None, name=None):
        self._v = value if value is not None else self._default
    def get(self):
        return self._v
    def set(self, v):
        self._v = v
    def trace_add(self, *a, **k):
        pass
    trace = trace_add


class StringVar(Variable):
    _default = ""
class BooleanVar(Variable):
    _default = False
class IntVar(Variable):
    _default = 0
class DoubleVar(Variable):
    _default = 0.0


class Widget:
    def __init__(self, master=None, **kw):
        self.master = master
        self.opts = dict(kw)
        self.children = []
        self.bindings = {}
        self._mapped = True
        if master is not None and hasattr(master, "children"):
            master.children.append(self)

    # options
    def config(self, **kw):
        self.opts.update(kw)
    configure = config
    def cget(self, k):
        return self.opts.get(k)
    def __getitem__(self, k):
        # MUST raise for unknown keys: `"x" in widget` falls back to
        # iteration via __getitem__, and a version that returned None
        # never raised IndexError and hung forever.
        if k not in self.opts:
            raise KeyError(k)
        return self.opts[k]
    def __setitem__(self, k, v):
        self.opts[k] = v

    # geometry
    def pack(self, **kw): self._geo = ("pack", kw)
    def grid(self, **kw): self._geo = ("grid", kw)
    def place(self, **kw): self._geo = ("place", kw)
    def pack_forget(self): self._mapped = False
    def grid_forget(self): self._mapped = False
    def place_forget(self): self._mapped = False
    def pack_info(self): return getattr(self, "_geo", ("pack", {}))[1]
    def grid_columnconfigure(self, *a, **k): pass
    def grid_rowconfigure(self, *a, **k): pass
    def columnconfigure(self, *a, **k): pass
    def rowconfigure(self, *a, **k): pass

    # events / lifecycle
    def bind(self, seq, fn=None, add=None): self.bindings[seq] = fn
    def bind_all(self, seq, fn=None, add=None): self.bindings[seq] = fn
    def unbind(self, seq, funcid=None): self.bindings.pop(seq, None)
    def unbind_all(self, seq): self.bindings.pop(seq, None)
    def after(self, ms, fn=None, *a): return "after#1"
    def after_idle(self, fn=None, *a):
        # Run it now. Deferring would mean the tests never exercise the
        # very code the app defers, which is where the tab-body wiring is.
        return fn(*a) if fn else None
    def after_cancel(self, i): pass
    def update(self): pass
    def update_idletasks(self): pass
    def destroy(self): self._mapped = False
    def focus_set(self): pass
    def winfo_ismapped(self): return self._mapped
    def winfo_children(self): return list(self.children)
    def winfo_width(self): return 800
    def winfo_height(self): return 600
    def winfo_reqwidth(self): return 100
    def winfo_reqheight(self): return 30
    def winfo_exists(self): return True
    def winfo_toplevel(self): return self
    def winfo_fpixels(self, s): return 96.0


class Misc(Widget): pass
class Tk(Widget):
    def title(self, *a): pass
    def geometry(self, *a): pass
    def minsize(self, *a): pass
    def maxsize(self, *a): pass
    def resizable(self, *a): pass
    def protocol(self, *a): pass
    def mainloop(self): pass
    def iconphoto(self, *a): pass
    def call(self, *a): return ""
    def tk_setPalette(self, *a): pass
    def wm_attributes(self, *a): pass
    attributes = wm_attributes
    def state(self, *a): return "normal"
    def deiconify(self): pass
    def lift(self): pass
    def grab_set(self): pass
    def transient(self, *a): pass
    # Not a real focus manager -- tests set `root._focus` directly to
    # simulate "the cursor is in this Entry" for the gating checks in
    # core/keyboard.py (_jog_keys_enabled / _run_key_enabled).
    def focus_get(self): return getattr(self, "_focus", None)


class Toplevel(Tk): pass
class Frame(Widget): pass
class LabelFrame(Widget): pass
class Label(Widget): pass
class Button(Widget):
    def invoke(self):
        fn = self.opts.get("command")
        return fn() if fn else None
class Entry(Widget):
    def get(self): return self.opts.get("_text", "")
    # Real Text.insert takes trailing tag names; accept and ignore them.
    def insert(self, i, t, *tags): self.opts["_text"] = self.opts.get("_text", "") + t
    def delete(self, a, b=None): self.opts["_text"] = ""
    def icursor(self, *a): pass
    def selection_range(self, *a): pass
class Spinbox(Entry): pass
class Text(Entry):
    def see(self, *a): pass
    def tag_config(self, *a, **k): pass
    tag_configure = tag_config
    def yview(self, *a): pass
    # The log trims itself by line count, so index() has to return
    # something parseable as "<line>.<col>".
    def index(self, spec):
        return "%d.0" % (self.opts.get("_text", "").count("\n") + 1)
    def delete(self, a, b=None):
        if a == "1.0" and b and b != "end":
            keep = self.opts.get("_text", "").split("\n")
            try:
                n = int(str(b).split(".")[0]) - 1
            except ValueError:
                n = 0
            self.opts["_text"] = "\n".join(keep[n:])
        else:
            self.opts["_text"] = ""
class Listbox(Widget):
    def __init__(self, master=None, **kw):
        super().__init__(master, **kw); self.items = []
    def insert(self, i, t, *tags): self.items.append(t)
    def delete(self, a, b=None): self.items = []
    def curselection(self): return getattr(self, "_sel", ())
    def get(self, i, j=None): return self.items[i] if j is None else self.items[i:j]
    def size(self): return len(self.items)
    def selection_set(self, i): self._sel = (i,)
    def see(self, *a): pass
    def yview(self, *a): pass
class Scrollbar(Widget):
    def set(self, *a): pass
class Canvas(Widget):
    def __init__(self, master=None, **kw):
        super().__init__(master, **kw); self.items = []
    def create_rectangle(self, *a, **k): self.items.append(("rect", a, k)); return len(self.items)
    def create_oval(self, *a, **k): self.items.append(("oval", a, k)); return len(self.items)
    def create_polygon(self, *a, **k): self.items.append(("poly", a, k)); return len(self.items)
    def create_line(self, *a, **k): self.items.append(("line", a, k)); return len(self.items)
    def create_text(self, *a, **k): self.items.append(("text", a, k)); return len(self.items)
    def create_arc(self, *a, **k): self.items.append(("arc", a, k)); return len(self.items)
    def create_image(self, *a, **k): self.items.append(("image", a, k)); return len(self.items)
    def create_window(self, *a, **k): self.items.append(("window", a, k)); return len(self.items)
    def itemconfig(self, *a, **k): pass
    itemconfigure = itemconfig
    def coords(self, *a): return [0, 0, 0, 0]
    def delete(self, *a): self.items = []
    def bbox(self, *a): return (0, 0, 100, 100)
    def yview(self, *a): pass
    def yview_moveto(self, *a): pass
    def yview_scroll(self, *a): pass
    def xview(self, *a): pass
    def tag_bind(self, *a, **k): pass


class _TkCall:
    """Stands in for the Tcl interpreter handle. Pillow's ImageTk reaches
    through PhotoImage.tk to blit pixels, so this has to exist even though
    nothing is drawn."""
    def call(self, *a, **k): return ""
    def eval(self, *a): return ""
    def getint(self, v): return int(v)
    def globalgetvar(self, *a): return ""
    def createcommand(self, *a): return ""
    def deletecommand(self, *a): return ""


class PhotoImage:
    _n = 0
    def __init__(self, *a, **k):
        self._d = k
        self.tk = _TkCall()
        PhotoImage._n += 1
        self.name = "pyimage%d" % PhotoImage._n
    def __str__(self): return self.name
    def width(self): return int(self._d.get("width", 1))
    def height(self): return int(self._d.get("height", 1))
    def paste(self, *a, **k): pass
    def put(self, *a, **k): pass
    def blank(self): pass


# constants
(LEFT, RIGHT, TOP, BOTTOM, BOTH, X, Y, NONE, N, S, E, W,
 NE, NW, SE, SW, CENTER, HORIZONTAL, VERTICAL, END, INSERT,
 NORMAL, DISABLED, ACTIVE, FLAT, RAISED, SUNKEN, GROOVE, RIDGE, SOLID,
 SINGLE, BROWSE, WORD, CHAR) = (
    "left", "right", "top", "bottom", "both", "x", "y", "none",
    "n", "s", "e", "w", "ne", "nw", "se", "sw", "center",
    "horizontal", "vertical", "end", "insert",
    "normal", "disabled", "active", "flat", "raised", "sunken",
    "groove", "ridge", "solid", "single", "browse", "word", "char")
