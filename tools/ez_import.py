#!/usr/bin/env python3
"""Turn EZ-Template autons in autons.cpp into .auton files the sim can run.

    tools/ez_import.py v5/src/autons.cpp --func worldsMogoRush --bind isBlue=true
    tools/ez_import.py v5/src/autons.cpp v5/src/skills.cpp --all -o autons/

It reads the C++ directly, so you don't hand-copy routines (and don't make
copying mistakes). It understands the things autons are actually made of:

  * EZ calls: pid_drive_set, pid_turn_set, pid_swing_set, pid_odom_set,
    pid_wait / _until / _quick / _quick_chain, pid_speed_max_set,
    pid_drive_chain_constant_set, odom_xyt_set, drive_set
  * pros::delay
  * wrapper functions like our set_drive (see ALIASES below)
  * if/else on things it can evaluate (isBlue, a sgn variable, ...)
  * timed loops: while (pros::millis() - start < 1000) { chassis.drive_set(...); }
  * calls into other functions defined in the same files (inlined)
  * everything else (intake.move, mogoClamp.toggle, ChangeLBState(...)) is kept
    as an action: logged with its timestamp in the sim, not simulated

Anything it can't evaluate (a sensor read in an if, a for loop, a turn to a
point) is reported with its line number and written into the file as a comment,
so a person can see exactly what's missing. It does not guess silently.

No dependencies beyond the Python standard library.
"""
from __future__ import annotations

import argparse
import ast
import math
import os
import re
import sys
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Team-specific wrappers. Most teams wrap pid_drive_set in a helper; say how
# yours maps onto EZ here. Our set_drive(inches, time, minSpeed, maxSpeed) calls
# chassis.pid_drive_set(inches, maxSpeed). Its default arguments live in a
# header that isn't in this repo, so the default speed below is an assumption.
# ---------------------------------------------------------------------------
ALIASES = {
    # name: (EZ call, {ez_arg: source_arg_index}, {ez_arg: default})
    "set_drive": ("pid_drive_set", {"inches": 0, "speed": 3}, {"speed": 127}),
    "setDrive": ("drive_set", {"left": 0, "right": 1}, {}),
}

# Calls that change nothing the drive sim models. Dropped with a note.
IGNORED = {
    "std::cout", "printf", "chassis.pid_targets_reset", "chassis.drive_sensor_reset",
    "chassis.drive_imu_reset", "master.rumble", "pros::lcd::print",
}

# Calls that change drive behaviour in ways this port doesn't model yet.
UNSUPPORTED_DRIVE: dict[str, str] = {}

# Functions from EZ-Template's example project. Nearly every EZ team still has
# them in autons.cpp; --all skips them unless --include-examples is given.
EZ_EXAMPLES = {
    "drive_example", "turn_example", "drive_and_turn", "wait_until_change_speed", "swing_example",
    "motion_chaining", "combining_movements", "tug", "interfered_example", "odom_drive_example",
    "odom_pure_pursuit_example", "odom_pure_pursuit_wait_until_example", "odom_boomerang_example",
    "odom_boomerang_injected_pure_pursuit_example", "measure_offsets", "default_constants",
}

UNIT_SUFFIX = re.compile(r"^([0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?)_(in|deg|ms|s|cm|mm|ft|rad)$")
UNIT_SCALE = {"in": 1.0, "deg": 1.0, "ms": 1.0, "s": 1000.0, "cm": 1 / 2.54, "mm": 1 / 25.4,
              "ft": 12.0, "rad": 180.0 / math.pi}


class Unresolved(Exception):
    """An expression depends on something only known on the robot."""


# ---------------------------------------------------------------------------
# Tokenizer
# ---------------------------------------------------------------------------
@dataclass
class Tok:
    kind: str   # id, num, str, op
    text: str
    line: int


TOKEN_RE = re.compile(r"""
    (?P<ws>[ \t\r\f\v]+) |
    (?P<nl>\n) |
    (?P<lc>//[^\n]*) |
    (?P<bc>/\*.*?\*/) |
    (?P<str>"(?:\\.|[^"\\])*") |
    (?P<chr>'(?:\\.|[^'\\])') |
    (?P<num>[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?(?:_[a-z]+)?[fFlLuU]*) |
    (?P<id>[A-Za-z_][A-Za-z0-9_]*(?:::[A-Za-z_][A-Za-z0-9_]*)*) |
    (?P<op><<=|>>=|->|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\||[-+*/%<>=!?:;,.(){}\[\]&|^~])
""", re.S | re.X)


def tokenize(src: str) -> list[Tok]:
    toks, line, pos = [], 1, 0
    while pos < len(src):
        m = TOKEN_RE.match(src, pos)
        if not m:
            pos += 1  # stray character (#include etc. never reach here)
            continue
        kind, text = m.lastgroup, m.group()
        if kind == "nl":
            line += 1
        elif kind in ("lc", "bc"):
            line += text.count("\n")
        elif kind != "ws":
            if kind == "chr":
                kind = "str"
            if kind == "num":
                text = text.rstrip("fFlLuU")
            toks.append(Tok(kind, text, line))
        pos = m.end()
    return toks


def join(toks: list[Tok]) -> str:
    """Re-render tokens compactly (for labels and comments)."""
    binary = {"+", "-", "*", "/", "%", "<", ">", "<=", ">=", "==", "!=", "&&", "||", "?", ":", "=",
              "+=", "-=", "<<", ">>"}
    out, prev = "", None
    for t in toks:
        # '-' and '+' are binary only after something that ends an operand
        is_bin = t.text in binary and not (
            t.text in ("-", "+") and (prev is None or (prev.kind == "op" and prev.text not in (")", "]"))))
        if prev is not None:
            if is_bin or (prev.kind != "op" and t.kind != "op"):
                out += " "
            elif prev.text == ",":
                out += " "
            elif prev.text in binary and prev_bin:
                out += " "
        out += t.text
        prev, prev_bin = t, is_bin
    return out


# ---------------------------------------------------------------------------
# Finding functions
# ---------------------------------------------------------------------------
@dataclass
class Func:
    name: str
    params: list[tuple[str, str]]   # (type, name)
    body: list[Tok]
    file: str
    line: int


TYPE_WORDS = {"void", "int", "double", "float", "long", "bool", "auto", "short", "unsigned",
              "const", "char", "std::string", "static", "inline"}


def find_functions(toks: list[Tok], file: str) -> dict[str, Func]:
    funcs, i = {}, 0
    while i < len(toks) - 3:
        t = toks[i]
        if t.kind == "id" and t.text in TYPE_WORDS and toks[i + 1].kind == "id" and toks[i + 2].text == "(":
            name = toks[i + 1].text
            j, depth = i + 3, 1
            while j < len(toks) and depth:
                depth += {"(": 1, ")": -1}.get(toks[j].text, 0)
                j += 1
            if j < len(toks) and toks[j].text == "{":
                params = parse_params(toks[i + 3:j - 1])
                k, depth = j + 1, 1
                while k < len(toks) and depth:
                    depth += {"{": 1, "}": -1}.get(toks[k].text, 0)
                    k += 1
                funcs[name] = Func(name, params, toks[j + 1:k - 1], file, t.line)
                i = k
                continue
        i += 1
    return funcs


def parse_params(toks: list[Tok]) -> list[tuple[str, str]]:
    params, cur = [], []
    for t in toks + [Tok("op", ",", 0)]:
        if t.text == ",":
            ids = [x.text for x in cur if x.kind == "id"]
            if ids and ids != ["void"]:
                params.append((" ".join(ids[:-1]), ids[-1]))
            cur = []
        elif t.text == "=":
            cur.append(Tok("op", "=", 0))
            break  # default arguments: rare in .cpp definitions; stop at '='
        else:
            cur.append(t)
    return params


# ---------------------------------------------------------------------------
# Expressions: C++ tokens -> a Python expression evaluated over known values
# ---------------------------------------------------------------------------
MILLIS = object()   # marker for "a pros::millis() snapshot"
SAFE_FUNCS = {"fabs": abs, "abs": abs, "std::abs": abs, "sqrt": math.sqrt, "std::fabs": abs,
              "sin": math.sin, "cos": math.cos, "atan2": math.atan2}
SAFE_NODES = (ast.Expression, ast.BinOp, ast.UnaryOp, ast.BoolOp, ast.Compare, ast.IfExp, ast.Constant,
              ast.Name, ast.Load, ast.Call, ast.Add, ast.Sub, ast.Mult, ast.Div, ast.Mod, ast.USub, ast.UAdd,
              ast.Not, ast.And, ast.Or, ast.Eq, ast.NotEq, ast.Lt, ast.LtE, ast.Gt, ast.GtE)


def to_python(toks: list[Tok], env: dict) -> str:
    """Translate a C++ expression to Python, resolving names from env."""
    # peel parentheses that wrap the whole expression, e.g. (!isBlue ? a : b)
    while len(toks) >= 2 and toks[0].text == "(" and match(toks, 0, "(", ")") == len(toks) - 1:
        toks = toks[1:-1]
    # ternary: split on the top-level '?' and its matching ':'
    depth, q = 0, -1
    for i, t in enumerate(toks):
        depth += {"(": 1, ")": -1}.get(t.text, 0)
        if t.text == "?" and depth == 0:
            q = i
            break
    if q >= 0:
        depth, nest = 0, 0
        for j in range(q + 1, len(toks)):
            t = toks[j]
            depth += {"(": 1, ")": -1}.get(t.text, 0)
            if depth == 0 and t.text == "?":
                nest += 1
            if depth == 0 and t.text == ":":
                if nest == 0:
                    c, a, b = toks[:q], toks[q + 1:j], toks[j + 1:]
                    return f"(({to_python(a, env)}) if ({to_python(c, env)}) else ({to_python(b, env)}))"
                nest -= 1
        raise Unresolved("unbalanced ?:")

    out, i = [], 0
    while i < len(toks):
        t = toks[i]
        nxt = toks[i + 1].text if i + 1 < len(toks) else ""
        if t.kind == "num":
            m = UNIT_SUFFIX.match(t.text)
            out.append(repr(float(m.group(1)) * UNIT_SCALE[m.group(2)]) if m else repr(float(t.text)))
        elif t.kind == "id":
            # C-style cast like (int) or static_cast<...>: drop the type
            if t.text in ("int", "double", "float", "long") and out and out[-1] == "(" and nxt == ")":
                out.pop()
                i += 2
                continue
            if nxt == "(":
                if t.text in SAFE_FUNCS:
                    out.append(t.text.replace("std::", ""))
                elif t.text == "pros::millis":
                    raise Unresolved("pros::millis()")
                else:
                    raise Unresolved(f"call to {t.text}()")
            elif t.text == "true":
                out.append("True")
            elif t.text == "false":
                out.append("False")
            elif t.text in ("M_PI",):
                out.append(repr(math.pi))
            elif t.text in env:
                v = env[t.text]
                if v is MILLIS or v is None:
                    raise Unresolved(t.text)
                out.append(repr(v))
            elif t.text in EZ_ENUMS:
                out.append(repr(EZ_ENUMS[t.text]))
            else:
                raise Unresolved(t.text)
        elif t.kind == "str":
            raise Unresolved("string")
        else:
            op = {"&&": " and ", "||": " or ", "!": " not "}.get(t.text, t.text)
            if t.text in (".", "->", "[", "]", "&", "|", "^", "~", "<<", ">>", "=", "++", "--"):
                raise Unresolved(f"operator {t.text}")
            out.append(op)
        i += 1
    return "".join(out)


def evaluate(toks: list[Tok], env: dict):
    if not toks:
        raise Unresolved("empty expression")
    src = to_python(toks, env).strip()
    try:
        tree = ast.parse(src, mode="eval")
    except SyntaxError as e:
        raise Unresolved(f"can't parse '{join(toks)}'") from e
    for node in ast.walk(tree):
        if not isinstance(node, SAFE_NODES):
            raise Unresolved(f"unsupported syntax in '{join(toks)}'")
        if isinstance(node, ast.Call) and not (isinstance(node.func, ast.Name) and node.func.id in
                                               {k.replace("std::", "") for k in SAFE_FUNCS}):
            raise Unresolved("call")
    funcs = {k.replace("std::", ""): v for k, v in SAFE_FUNCS.items()}
    return eval(compile(tree, "<expr>", "eval"), {"__builtins__": {}}, funcs)  # noqa: S307 (whitelisted AST)


def split_args(toks: list[Tok]) -> list[list[Tok]]:
    args, cur, depth = [], [], 0
    for t in toks:
        if t.text in "({[":
            depth += 1
        elif t.text in ")}]":
            depth -= 1
        if t.text == "," and depth == 0:
            args.append(cur)
            cur = []
        else:
            cur.append(t)
    if cur or args:
        args.append(cur)
    return args


def fmt(v) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    v = float(v)
    return str(int(v)) if v == int(v) else f"{v:.4g}"


# ---------------------------------------------------------------------------
# Walking the statements of an auton
# ---------------------------------------------------------------------------
@dataclass
class Line:
    text: str        # the .auton instruction, or "" for a pure comment
    src: int         # source line
    note: str = ""   # trailing comment


@dataclass
class Result:
    name: str
    bindings: dict
    lines: list[Line] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    start: tuple | None = None
    source: str = ""


class Importer:
    def __init__(self, funcs: dict[str, Func], consts: dict | None = None):
        self.funcs = funcs
        self.consts = consts or {}

    # --- entry ---------------------------------------------------------------
    def run(self, name: str, bindings: dict) -> Result:
        f = self.funcs[name]
        res = Result(name, dict(bindings), source=f"{os.path.basename(f.file)}:{f.line}")
        env = dict(self.consts)
        for _, p in f.params:
            env[p] = bindings.get(p)
        self.res, self.depth, self.saw_motion = res, 0, False
        self.turn_behavior = "shortest"
        try:
            self.block(f.body, env)
        except _Return:
            pass
        return res

    def warn(self, line: int, msg: str, text: str = ""):
        self.res.warnings.append(f"line {line}: {msg}" + (f"  [{text}]" if text else ""))
        self.res.lines.append(Line("", line, f"NOT IMPORTED: {msg}" + (f": {text}" if text else "")))

    def emit(self, text: str, line: int, note: str = ""):
        head = text.split("(")[0]
        if head not in ("action", "delay", "pid_speed_max_set", "pid_drive_chain_constant_set",
                        "pid_turn_chain_constant_set", "slew_drive_constants_set", "odom_look_ahead_set"):
            self.saw_motion = True
        self.res.lines.append(Line(text, line, note))

    # --- statements ------------------------------------------------------------
    def block(self, toks: list[Tok], env: dict):
        i = 0
        while i < len(toks):
            i = self.statement(toks, i, env)

    def statement(self, toks: list[Tok], i: int, env: dict) -> int:
        t = toks[i]
        if t.text == ";":
            return i + 1
        if t.text == "{":
            end = match(toks, i, "{", "}")
            self.block(toks[i + 1:end], dict(env) | {})  # scoped copy is fine: locals rarely escape
            return end + 1
        if t.text == "if":
            return self.if_stmt(toks, i, env)
        if t.text == "while":
            return self.while_stmt(toks, i, env)
        if t.text in ("for", "switch", "do"):
            end = self.skip_statement(toks, i)
            self.warn(t.line, f"'{t.text}' loops/switches aren't imported", join(toks[i:min(end, i + 12)]) + " ...")
            return end
        if t.text == "return":
            end = find(toks, i, ";")
            raise _Return()
        end = find(toks, i, ";")
        self.simple(toks[i:end], env)
        return end + 1

    def skip_statement(self, toks, i):
        """Skip a for/while/switch/do statement including its body."""
        j = i + 1
        if j < len(toks) and toks[j].text == "(":
            j = match(toks, j, "(", ")") + 1
        if j < len(toks) and toks[j].text == "{":
            return match(toks, j, "{", "}") + 1
        return find(toks, j, ";") + 1

    def body_after(self, toks, j):
        """Return (body_tokens, next_index) for the statement starting at j."""
        if toks[j].text == "{":
            end = match(toks, j, "{", "}")
            return toks[j + 1:end], end + 1
        if toks[j].text in ("if", "while", "for"):
            # nested control statement without braces
            k = self.skip_statement(toks, j) if toks[j].text != "if" else self._if_end(toks, j)
            return toks[j:k], k
        end = find(toks, j, ";")
        return toks[j:end + 1], end + 1

    def _if_end(self, toks, i):
        j = match(toks, i + 1, "(", ")") + 1
        _, j = self.body_after(toks, j)
        if j < len(toks) and toks[j].text == "else":
            _, j = self.body_after(toks, j + 1)
        return j

    def if_stmt(self, toks, i, env):
        line = toks[i].line
        close = match(toks, i + 1, "(", ")")
        cond = toks[i + 2:close]
        then_body, j = self.body_after(toks, close + 1)
        else_body = None
        if j < len(toks) and toks[j].text == "else":
            else_body, j = self.body_after(toks, j + 1)
        try:
            taken = bool(evaluate(cond, env))
        except Unresolved as e:
            self.warn(line, f"can't evaluate if-condition ({e}); imported the if-branch", f"if ({join(cond)})")
            taken = True
        body = then_body if taken else else_body
        if body:
            self.block(body, env)
        return j

    def while_stmt(self, toks, i, env):
        line = toks[i].line
        close = match(toks, i + 1, "(", ")")
        cond = toks[i + 2:close]
        body, j = self.body_after(toks, close + 1)
        # The one loop shape autons use: drive for a fixed time.
        #   while (pros::millis() - start < T) { chassis.drive_set(a, b); pros::delay(10); }
        ms = self.timed_loop_duration(cond, env)
        if ms is None:
            self.warn(line, "while loop on a runtime condition isn't imported", f"while ({join(cond)})")
            return j
        inner = Importer(self.funcs, self.consts)
        inner.res = Result("loop", {})
        inner.saw_motion, inner.depth, inner.turn_behavior = True, self.depth, self.turn_behavior
        inner.block(body, env)
        for ln in inner.res.lines:
            if ln.text.startswith("delay("):
                continue  # the loop's own tick delay; the whole loop becomes one delay
            self.res.lines.append(ln)
        self.res.warnings += inner.res.warnings
        self.emit(f"delay({fmt(ms)})", line, f"while (pros::millis() - ... < {fmt(ms)})")
        return j

    def timed_loop_duration(self, cond, env):
        text = [t.text for t in cond]
        if "pros::millis" not in text:
            return None
        lt = next((k for k, t in enumerate(cond) if t.text in ("<", "<=")), None)
        if lt is None:
            return None
        lhs, rhs = cond[:lt], cond[lt + 1:]
        # lhs must be `pros::millis() - <snapshot>`
        names = [t.text for t in lhs if t.kind == "id" and t.text != "pros::millis"]
        if len(names) != 1 or env.get(names[0]) is not MILLIS:
            return None
        try:
            return float(evaluate(rhs, env))
        except Unresolved:
            return None

    # --- simple statements -----------------------------------------------------
    def simple(self, toks: list[Tok], env: dict):
        if not toks:
            return
        line = toks[0].line
        text = join(toks)

        # declaration: [const] type name = expr
        k = 0
        while k < len(toks) and toks[k].kind == "id" and toks[k].text in TYPE_WORDS | {"ez::pose", "std::string"}:
            k += 1
        if 0 < k < len(toks) - 1 and toks[k].kind == "id" and toks[k + 1].text == "=":
            name, expr = toks[k].text, toks[k + 2:]
            if [t.text for t in expr[:3]] == ["pros::millis", "(", ")"] and len(expr) == 3:
                env[name] = MILLIS
                return
            try:
                env[name] = evaluate(expr, env)
            except Unresolved:
                env[name] = None  # known to exist, value only known on the robot
            return

        # assignment to a local: x = expr / x += expr
        if len(toks) == 5 and toks[0].kind == "id" and toks[1].text == "=" and \
                [t.text for t in toks[2:]] == ["pros::millis", "(", ")"]:
            env[toks[0].text] = MILLIS
            return
        if len(toks) >= 3 and toks[0].kind == "id" and toks[0].text in env and toks[1].text in ("=", "+=", "-="):
            try:
                v = evaluate(toks[2:], env)
                env[toks[0].text] = v if toks[1].text == "=" else env[toks[0].text] + (v if toks[1].text == "+=" else -v)
            except (Unresolved, TypeError):
                env[toks[0].text] = None
            return

        # a call: qualified.name( args )
        if toks[0].kind == "id" and len(toks) >= 3 and toks[1].text in (".", "(") :
            callee, p = toks[0].text, 1
            while p + 1 < len(toks) and toks[p].text == "." and toks[p + 1].kind == "id":
                callee += "." + toks[p + 1].text
                p += 2
            if p < len(toks) and toks[p].text == "(":
                close = match(toks, p, "(", ")")
                if close == len(toks) - 1:
                    self.call(callee, split_args(toks[p + 1:close]), env, line, text)
                    return

        if toks[0].text in ("std::cout",):
            return
        # anything else (x = y on a global, x++): keep as an action so the timing is visible
        self.emit(f'action("{esc(text)}")', line)

    def call(self, callee: str, args: list[list[Tok]], env: dict, line: int, text: str):
        def num(a, what="argument"):
            try:
                v = evaluate(a, env)
            except Unresolved as e:
                raise _Skip(f"{what} depends on the robot at runtime ({e})")
            if isinstance(v, bool):
                return float(v)
            return float(v)

        try:
            if callee in IGNORED:
                return
            if callee in UNSUPPORTED_DRIVE:
                self.warn(line, UNSUPPORTED_DRIVE[callee], text)
                return
            if callee == "pros::delay":
                self.emit(f"delay({fmt(num(args[0]))})", line)
                return
            if callee in ALIASES:
                ez, mapping, defaults = ALIASES[callee]
                vals = {}
                for key, idx in mapping.items():
                    if idx < len(args):
                        vals[key] = num(args[idx], key)
                    elif key in defaults:
                        vals[key] = defaults[key]
                    else:
                        raise _Skip(f"{callee}: missing {key}")
                if ez == "pid_drive_set":
                    self.emit(f"pid_drive_set({fmt(vals['inches'])}, {fmt(vals['speed'])})", line, text)
                else:
                    self.emit(f"drive_set({fmt(vals['left'])}, {fmt(vals['right'])})", line, text)
                return
            if callee.startswith("chassis."):
                self.ez_call(callee[len("chassis."):], args, env, line, text, num)
                return
            if callee in self.funcs and self.depth < 4:
                self.inline(callee, args, env, line, text)
                return
            # a mechanism or anything else: keep it as a timed action
            self.emit(f'action("{esc(text)}")', line)
        except _Skip as e:
            self.warn(line, str(e), text)

    def evaluate_word(self, toks, env):
        """An enum-ish argument (ez::LEFT_SWING, fwd, a ternary of them) -> 'left' etc."""
        try:
            v = evaluate(toks, env)
            if isinstance(v, str):
                return v
        except Unresolved:
            pass
        return toks[-1].text.split("::")[-1].lower() if toks else ""

    def ez_call(self, m, args, env, line, text, num):
        n = len(args)
        if m == "pid_drive_set":
            if n < 2:
                raise _Skip("pid_drive_set needs a speed")
            slew = ""
            if n >= 3:
                try:
                    slew = "" if evaluate(args[2], env) else ", false"
                except Unresolved:
                    pass
            self.emit(f"pid_drive_set({fmt(num(args[0]))}, {fmt(num(args[1]))}{slew})", line, note(text))
        elif m in ("pid_turn_set",):
            if args and args[0] and args[0][0].text == "{":
                # pid_turn_set({x, y}, fwd|rev, speed): face a point
                pt = split_args(args[0][1:-1])
                if len(pt) != 2 or n < 3:
                    raise _Skip("unrecognised turn-to-point form")
                d = "rev" if self.evaluate_word(args[1], env).startswith("rev") else "fwd"
                extra = ", rev" if d == "rev" else ""
                self.emit(f"pid_turn_to_point({fmt(num(pt[0]))}, {fmt(num(pt[1]))}, {fmt(num(args[2]))}{extra})",
                          line, note(text))
                return
            if n < 2:
                raise _Skip("pid_turn_set needs a speed")
            beh = self.turn_behavior
            if n >= 3:
                k = args[2][-1].text if args[2] else ""
                beh = {"shortest": "shortest", "longest": "longest", "cw": "cw", "ccw": "ccw",
                       "raw": "raw", "left_turn": "ccw", "right_turn": "cw"}.get(k.split("::")[-1], beh)
            extra = "" if beh == "shortest" else f", {beh}"
            self.emit(f"pid_turn_set({fmt(num(args[0]))}, {fmt(num(args[1]))}{extra})", line, note(text))
        elif m == "pid_turn_behavior_set":
            k = args[0][-1].text.split("::")[-1] if args and args[0] else ""
            self.turn_behavior = {"left_turn": "ccw", "right_turn": "cw"}.get(k, k or "shortest")
        elif m == "pid_swing_set":
            if n < 3:
                raise _Skip("pid_swing_set needs side, angle, speed")
            side = self.evaluate_word(args[0], env)
            side = "left" if "left" in side else "right" if "right" in side else None
            if side is None:
                raise _Skip("unknown swing side")
            opp = f", {fmt(num(args[3]))}" if n >= 4 else ""
            self.emit(f"pid_swing_set({side}, {fmt(num(args[1]))}, {fmt(num(args[2]))}{opp})", line, note(text))
        elif m == "pid_odom_set":
            self.odom_set(args, line, text, num)
        elif m == "pid_wait":
            self.emit("pid_wait()", line)
        elif m == "pid_wait_until":
            if args and args[0] and args[0][0].text == "{":
                raise _Skip("waiting on a point isn't supported yet")
            self.emit(f"pid_wait_until({fmt(num(args[0]))})", line, note(text))
        elif m == "pid_wait_quick":
            self.emit("pid_wait_quick()", line)
        elif m == "pid_wait_quick_chain":
            self.emit("pid_wait_quick_chain()", line)
        elif m == "pid_turn_relative_set":
            self.emit(f"pid_turn_relative_set({fmt(num(args[0]))}, {fmt(num(args[1]))})", line, note(text))
        elif m == "pid_turn_chain_constant_set":
            self.emit(f"pid_turn_chain_constant_set({fmt(num(args[0]))})", line)
        elif m == "slew_drive_constants_set":
            self.emit(f"slew_drive_constants_set({fmt(num(args[0]))}, {fmt(num(args[1]))})", line, note(text))
        elif m == "odom_look_ahead_set":
            self.emit(f"odom_look_ahead_set({fmt(num(args[0]))})", line, note(text))
        elif m == "pid_speed_max_set":
            self.emit(f"pid_speed_max_set({fmt(num(args[0]))})", line)
        elif m == "pid_drive_chain_constant_set":
            self.emit(f"pid_drive_chain_constant_set({fmt(num(args[0]))})", line)
        elif m == "odom_xyt_set":
            if n != 3:
                raise _Skip("odom_xyt_set with a pose object isn't supported")
            x, y, th = (num(a) for a in args)
            if not self.saw_motion and self.res.start is None:
                self.res.start = (x, y, th)
            self.emit(f"odom_xyt_set({fmt(x)}, {fmt(y)}, {fmt(th)})", line, note(text))
        elif m == "drive_set":
            self.emit(f"drive_set({fmt(num(args[0]))}, {fmt(num(args[1]))})", line, note(text))
        elif m.endswith("_get") or m.startswith("odom_") or m.startswith("drive_"):
            return  # getters and sensor reads with no effect on their own
        else:
            raise _Skip(f"chassis.{m} isn't supported")

    def odom_set(self, args, line, text, num):
        # pid_odom_set(distance, speed[, slew]): EZ's odom-backed straight drive.
        # The sim can't know the pose statically, so this becomes a plain
        # pid_drive_set. Same distance and speed; EZ steers it with odom.
        if args and args[0] and args[0][0].text != "{":
            if len(args) < 2:
                raise _Skip("pid_odom_set needs a speed")
            self.emit(f"pid_drive_set({fmt(num(args[0]))}, {fmt(num(args[1]))})", line,
                      f"{text}  (approximated as a straight pid_drive_set)")
            return
        # pid_odom_set({{x, y}, fwd, speed}) or pid_odom_set({{x, y}, fwd, speed}, slew)
        if not args or not args[0] or args[0][0].text != "{":
            raise _Skip("unrecognised pid_odom_set form")
        inner = args[0][1:-1]
        parts = split_args(inner)
        if len(parts) < 3 or not parts[0] or parts[0][0].text != "{" or parts[0][1].text == "{":
            raise _Skip("multi-point paths / pure pursuit aren't supported yet")
        pt = split_args(parts[0][1:-1])
        if len(pt) != 2:
            raise _Skip("boomerang targets (x, y, theta) aren't supported yet")
        d = parts[1][-1].text.split("::")[-1].lower() if parts[1] else "fwd"
        d = "rev" if d.startswith("rev") else "fwd"
        extra = ", rev" if d == "rev" else ""
        self.emit(f"pid_odom_set({fmt(num(pt[0]))}, {fmt(num(pt[1]))}, {fmt(num(parts[2]))}{extra})", line,
                  note(text))

    def inline(self, callee, args, env, line, text):
        f = self.funcs[callee]
        sub = dict(self.consts)
        for (ptype, pname), a in zip(f.params, args + [None] * len(f.params)):
            if a is None:
                raise _Skip(f"{callee}: argument '{pname}' has a default this tool can't see")
            try:
                sub[pname] = evaluate(a, env)
            except Unresolved:
                sub[pname] = None
        self.res.lines.append(Line("", line, f"--- {text} (inlined from {os.path.basename(f.file)}:{f.line}) ---"))
        self.depth += 1
        try:
            self.block(f.body, sub)
        except _Return:
            pass
        finally:
            self.depth -= 1


class _Return(Exception):
    pass


class _Skip(Exception):
    pass


def note(text: str) -> str:
    """Keep the original call as a comment when it isn't a trivial chassis call."""
    return text if not re.match(r"^chassis\.\w+\([-0-9., ]*\)$", text) else ""


def esc(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"')


def find(toks, i, text):
    depth = 0
    for j in range(i, len(toks)):
        if toks[j].text in "({[":
            depth += 1
        elif toks[j].text in ")}]":
            depth -= 1
        if toks[j].text == text and depth == 0:
            return j
    return len(toks)


def match(toks, i, open_, close):
    depth = 0
    for j in range(i, len(toks)):
        if toks[j].text == open_:
            depth += 1
        elif toks[j].text == close:
            depth -= 1
            if depth == 0:
                return j
    return len(toks) - 1


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
def render(res: Result, out_name: str) -> str:
    lines = [f"# @name {out_name}",
             f"# imported from {res.source} by tools/ez_import.py",
             "# bindings: " + (", ".join(f"{k}={fmt(v)}" for k, v in res.bindings.items()) or "none")]
    if res.warnings:
        lines.append(f"# {len(res.warnings)} line(s) could not be imported; search for NOT IMPORTED")
    if res.start is None:
        lines.append("# no odom_xyt_set before the first motion; starting at (0, 0, 0)")
    lines.append("")
    width = max((len(l.text) for l in res.lines if l.text), default=0)
    for l in res.lines:
        if not l.text:
            lines.append(f"# {l.note}  @{l.src}" if l.note.startswith("NOT") else f"# {l.note}")
            continue
        s = f"{l.text:<{width}}  @{l.src}"
        if l.note:
            s += f"  # {l.note}"
        lines.append(s)
    return "\n".join(lines) + "\n"


def snake(name: str) -> str:
    return re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", name).lower()


def parse_bindings(pairs: list[str]) -> dict:
    out = {}
    for p in pairs:
        k, _, v = p.partition("=")
        v = v.strip()
        out[k.strip()] = {"true": True, "false": False}.get(v.lower(), None)
        if out[k.strip()] is None:
            out[k.strip()] = float(v)
    return out


DEFINE_RE = re.compile(r"^[ \t]*#[ \t]*define[ \t]+([A-Za-z_]\w*)[ \t]+([^\n]+)$", re.M)


def file_constants(src: str, toks: list[Tok], env: dict) -> None:
    """Values every function in the file can see: `const int DRIVE_SPEED = 110;`
    and `#define DRIVE_SLEW 0.02`. Every EZ-Template project ships DRIVE_SPEED,
    TURN_SPEED and SWING_SPEED this way."""
    for name, value in DEFINE_RE.findall(src):
        try:
            env[name] = evaluate(tokenize(value.split("//")[0]), env)
        except Unresolved:
            pass
    depth = 0
    for i, t in enumerate(toks):
        depth += {"{": 1, "}": -1}.get(t.text, 0)
        if depth or t.text not in ("const", "constexpr"):
            continue
        j = i + 1
        while j < len(toks) and toks[j].kind == "id" and toks[j].text in TYPE_WORDS | {"constexpr"}:
            j += 1
        if j + 1 < len(toks) and toks[j].kind == "id" and toks[j + 1].text == "=":
            end = find(toks, j + 2, ";")
            try:
                env[toks[j].text] = evaluate(toks[j + 2:end], env)
            except Unresolved:
                pass


EZ_ENUMS = {
    "ez::LEFT_SWING": "left", "ez::RIGHT_SWING": "right", "LEFT_SWING": "left", "RIGHT_SWING": "right",
    "ez::fwd": "fwd", "ez::rev": "rev", "fwd": "fwd", "rev": "rev",
    "ez::shortest": "shortest", "ez::longest": "longest", "ez::cw": "cw", "ez::ccw": "ccw", "ez::raw": "raw",
    "ez::left_turn": "ccw", "ez::right_turn": "cw",
}


def load(paths: list[str]) -> tuple[dict[str, Func], dict]:
    funcs, consts = {}, dict(EZ_ENUMS)
    for p in paths:
        with open(p, encoding="utf-8") as f:
            src = f.read()
        toks = tokenize(src)
        file_constants(src, toks, consts)
        funcs.update(find_functions(toks, p))
    return funcs, consts


def is_auton(f: Func) -> bool:
    """Heuristic: an auton is a void function that sets at least one chassis motion."""
    motions = {"pid_drive_set", "pid_turn_set", "pid_swing_set", "pid_odom_set"}
    ids = [t.text for t in f.body]
    uses_chassis = any(a == "chassis" and b == "." and c in motions for a, b, c in zip(ids, ids[1:], ids[2:]))
    return uses_chassis or "set_drive" in ids


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("sources", nargs="+", help="C++ files to read (autons.cpp, skills.cpp, ...)")
    ap.add_argument("--func", action="append", default=[], help="function to import (repeatable)")
    ap.add_argument("--all", action="store_true", help="import every function that looks like an auton")
    ap.add_argument("--bind", action="append", default=[], metavar="NAME=VALUE",
                    help="value for a parameter, e.g. isBlue=true (default for --all: isBlue=true)")
    ap.add_argument("-o", "--out", help="output file (one --func) or directory (--all / several)")
    ap.add_argument("--list", action="store_true", help="list functions that look like autons and exit")
    ap.add_argument("--include-examples", action="store_true",
                    help="with --all, also import EZ-Template's bundled example functions")
    a = ap.parse_args(argv)

    funcs, consts = load(a.sources)
    autons = [n for n, f in funcs.items() if is_auton(f) and n not in ALIASES]
    if a.list:
        for n in autons:
            f = funcs[n]
            print(f"{n:28s} {os.path.basename(f.file)}:{f.line}  params: {', '.join(p for _, p in f.params) or '-'}")
        return 0

    names = [n for n in autons if a.include_examples or n not in EZ_EXAMPLES] if a.all else a.func
    if not names:
        ap.error("give --func NAME, --all, or --list")
    missing = [n for n in names if n not in funcs]
    if missing:
        ap.error(f"not found: {', '.join(missing)}")
    binds = parse_bindings(a.bind) if a.bind else {}
    to_dir = a.out and (a.all or len(names) > 1 or os.path.isdir(a.out))
    if to_dir:
        os.makedirs(a.out, exist_ok=True)

    imp = Importer(funcs, consts)
    rows, rc, written = [], 0, {}
    for n in names:
        f = funcs[n]
        b = {p: binds[p] for _, p in f.params if p in binds}
        if a.all and "isBlue" in [p for _, p in f.params] and "isBlue" not in b:
            b["isBlue"] = True
        unbound = [p for _, p in f.params if p not in b]
        if unbound:
            rows.append((n, 0, f"skipped: needs --bind {', '.join(p + '=...' for p in unbound)}"))
            continue
        res = imp.run(n, b)
        # ".blue"/".red" rather than "_blue": teams often also have a separate
        # fooBlue() function, and snake-casing that must not collide with this.
        suffix = "" if "isBlue" not in b else (".blue" if b["isBlue"] else ".red")
        out_name = snake(n) + suffix
        if out_name in written:
            ap.error(f"{n} and {written[out_name]} would both be written as {out_name}.auton")
        written[out_name] = n
        text = render(res, out_name)
        count = sum(1 for l in res.lines if l.text)
        if not a.out:
            sys.stdout.write(text)
        else:
            path = os.path.join(a.out, out_name + ".auton") if to_dir else a.out
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(text)
        rows.append((n, count, f"{len(res.warnings)} not imported" if res.warnings else "clean"))
        for w in res.warnings:
            print(f"  {n}: {w}", file=sys.stderr)

    if a.out:
        w = max(len(r[0]) for r in rows)
        for n, c, s in rows:
            print(f"{n:<{w}}  {c:4d} instructions  {s}")
        clean = sum(1 for r in rows if r[2] == "clean")
        print(f"\n{len(rows)} autons, {clean} imported cleanly")
    return rc


if __name__ == "__main__":
    sys.exit(main())
