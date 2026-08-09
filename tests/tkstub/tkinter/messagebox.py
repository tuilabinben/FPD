CALLS = []
ANSWER = True
def showinfo(t="", m="", **k):    CALLS.append(("info", t, m));  return "ok"
def showwarning(t="", m="", **k): CALLS.append(("warn", t, m));  return "ok"
def showerror(t="", m="", **k):   CALLS.append(("error", t, m)); return "ok"
def askyesno(t="", m="", **k):    CALLS.append(("ask", t, m));   return ANSWER
def askokcancel(t="", m="", **k): CALLS.append(("ask", t, m));   return ANSWER
def askretrycancel(t="", m="", **k): CALLS.append(("ask", t, m)); return ANSWER
