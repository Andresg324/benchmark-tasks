"""Per-trial memory for rules scored over the run: latched events (debounced), their order, seconds held, counts,
peaks. Resets when yamkit's kit.episode() changes; updates once per control step though success() and score() both
call it (keyed on yamkit's step index in the sim, offline on the objects dict). Call _watch(o) atop both:
    T = tracekit.Trace()
    def _watch(o):
        if T.new_step(o):
            T.track("tipped", tilt_deg(o["plank"]) > 10, steps=3)  # 3 in a row: a one-step PhysX pop doesn't count
            T.peak("ext", ext(o))
    then T.ever("tipped"), T.before("strapped", "cap_off"), T.total_s(n), T.run_s(n), T.count(n), T.peak("ext")"""
import yamkit as kit


class Trace:
    def __init__(self, dt: float = 0.05):
        self.dt, self._ep, self._key, self._objs = dt, None, None, None
        self._fresh()

    def _fresh(self):
        self.step, self._run, self._total, self._first, self._count, self._peak = 0, {}, {}, {}, {}, {}

    def new_step(self, objects) -> bool:
        """True the first time it is called in a control step (and starts a new trial's record when one began)."""
        ep, n = kit.episode(), kit._EPISODE.get("last")
        if ep != self._ep:
            self._ep, self._key, self._objs = ep, None, None
            self._fresh()
        key = (ep, n) if n is not None else None
        if (key is not None and key == self._key) or (key is None and objects is self._objs):
            return False
        self._key, self._objs = key, objects  # holding `objects` keeps its id from being reused
        self.step += 1
        return True

    def track(self, name: str, cond: bool, steps: int = 1) -> bool:
        """Feed this step's condition. It counts once true for `steps` steps in a row; returns whether it counts now."""
        run = self._run[name] = self._run.get(name, 0) + 1 if cond else 0
        on = run >= steps
        if on:
            self._total[name] = self._total.get(name, 0) + (steps if run == steps else 1)
        if run == steps:  # a confirmed onset: latch the first, count each
            self._first.setdefault(name, self.step - steps + 1)
            self._count[name] = self._count.get(name, 0) + 1
        return on

    def peak(self, name: str, value: float | None = None) -> float:
        """The largest value fed this trial (feed it with `value`); -inf if none yet."""
        if value is not None:
            self._peak[name] = max(self._peak.get(name, float("-inf")), float(value))
        return self._peak.get(name, float("-inf"))

    def when(self, name): return self._first.get(name)  # step of the first confirmed onset (1 = first step), or None
    def ever(self, name): return name in self._first  # latched: stays True for the rest of the trial
    def count(self, name): return self._count.get(name, 0)  # confirmed onsets (separate episodes of the condition)
    def total_s(self, name): return self._total.get(name, 0) * self.dt  # seconds it counted, in all
    def run_s(self, name): return self._run.get(name, 0) * self.dt  # seconds true without a break, up to now

    def before(self, a: str, b: str) -> bool:
        """`a` happened, and `b` either never did or first happened strictly later."""
        return self.ever(a) and (not self.ever(b) or self.when(a) < self.when(b))
