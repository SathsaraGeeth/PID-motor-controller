from pid_spice import *
import numpy as np
from scipy.optimize import minimize


class PIDFactory:

    _FIXED_DEFAULTS = dict(
        TSTEP      = 1000e-6,
        TSTOP      = 1000e-3,
        DEFAULT_SP = 1.0,
        TEMP       = 25,
        VCC        = 15.0,
        VEE        = -15.0,
        OPAMP      = "ua741",
        R_MOT      = 5.5,
        L_MOT      = 1e-3,
        KEMF_MOT   = 0.02,
        J_MOT      = 10e-6,
        RLOSS_MOT  = 200.0,
        KTRQ_MOT   = 0.02,
    )

    TUNABLE_BOUNDS = dict(
        RIN_PROP   = (1e3,    1e6),
        RFB_PROP   = (1e3,    1e6),
        RIN_INT    = (1e3,    1e6),
        RLIM_INT   = (1e3,    1e6),
        C_INT      = (100e-9, 100e-6),
        R1_DIFF    = (1e3,    1e6),
        C_DIFF     = (1e-9,   10e-6),
        RDAMP_DIFF = (10,     10e3),
        RIN_SUM    = (10e3,   1e6),
        RFB_SUM    = (10e3,   1e6),
        R_ERR      = (100,    100e3),
    )

    _TUNABLE_DEFAULTS = dict(
        RIN_PROP   = 10e3,
        RFB_PROP   = 100e3,
        RIN_INT    = 10e3,
        RLIM_INT   = 10e3,
        C_INT      = 10e-6,
        R1_DIFF    = 10e3,
        C_DIFF     = 100e-9,
        RDAMP_DIFF = 100.0,
        RIN_SUM    = 100e3,
        RFB_SUM    = 100e3,
        R_ERR      = 1e3,
    )

    def __init__(self, **fixed_overrides):
        self._fixed = dict(self._FIXED_DEFAULTS)
        for k, v in fixed_overrides.items():
            if k not in self._FIXED_DEFAULTS:
                raise ValueError(f"'{k}' is not a fixed parameter.")
            self._fixed[k] = v

    def build(self, **tunable) -> "PID":
        t = self._apply_defaults(tunable)
        self._check_bounds(t)
        cfg = self._make_config(t)
        pid = PID(cfg)
        pid._system()
        return pid

    def default_tunables(self) -> dict:
        return dict(self._TUNABLE_DEFAULTS)

    def _apply_defaults(self, tunable: dict) -> dict:
        out = dict(self._TUNABLE_DEFAULTS)
        for k, v in tunable.items():
            if k not in self.TUNABLE_BOUNDS:
                raise ValueError(f"'{k}' is not a tunable parameter.")
            out[k] = v
        return out

    def _check_bounds(self, t: dict):
        for k, v in t.items():
            lo, hi = self.TUNABLE_BOUNDS[k]
            if not (lo <= v <= hi):
                raise ValueError(f"{k}={v:.3e} is outside [{lo:.3e}, {hi:.3e}]")

    def _make_config(self, t: dict) -> PIDConfig:
        cfg = PIDConfig()

        cfg.TSTEP      = self._fixed["TSTEP"]     @ u_s
        cfg.TSTOP      = self._fixed["TSTOP"]     @ u_s
        cfg.DEFAULT_SP = self._fixed["DEFAULT_SP"]
        cfg.TEMP       = self._fixed["TEMP"]
        cfg.VCC        = self._fixed["VCC"]       @ u_V
        cfg.VEE        = self._fixed["VEE"]       @ u_V
        cfg.OPAMP      = self._fixed["OPAMP"]
        cfg.R_MOT      = self._fixed["R_MOT"]     @ u_Ω
        cfg.L_MOT      = self._fixed["L_MOT"]     @ u_H
        cfg.KEMF_MOT   = self._fixed["KEMF_MOT"]
        cfg.J_MOT      = self._fixed["J_MOT"]
        cfg.RLOSS_MOT  = self._fixed["RLOSS_MOT"] @ u_Ω
        cfg.KTRQ_MOT   = self._fixed["KTRQ_MOT"]

        cfg.RIN_PROP   = t["RIN_PROP"]   @ u_Ω
        cfg.RFB_PROP   = t["RFB_PROP"]   @ u_Ω
        cfg.RIN_INT    = t["RIN_INT"]    @ u_Ω
        cfg.RLIM_INT   = t["RLIM_INT"]   @ u_Ω
        cfg.C_INT      = t["C_INT"]      @ u_F
        cfg.R1_DIFF    = t["R1_DIFF"]    @ u_Ω
        cfg.C_DIFF     = t["C_DIFF"]     @ u_F
        cfg.RDAMP_DIFF = t["RDAMP_DIFF"] @ u_Ω

        cfg.RIN1_SUM = cfg.RIN2_SUM = cfg.RIN3_SUM = t["RIN_SUM"] @ u_Ω
        cfg.RFB_SUM  = t["RFB_SUM"] @ u_Ω
        cfg.R1_ERR   = cfg.R2_ERR = cfg.R3_ERR = cfg.R4_ERR = t["R_ERR"] @ u_Ω

        return cfg


class Tuner:

    METRIC_KEYS = ("overshoot_%", "rise_time_s", "settling_time_s", "steady_state_error")

    def __init__(self, factory: PIDFactory, setpoint: float = 1.0, drive_kwargs: dict = None):
        self.factory      = factory
        self.setpoint     = setpoint
        self.drive_kwargs = drive_kwargs or {"kind": "step", "amplitude": 1.0, "t0": 1e-3}
        self._best_params : dict = {}
        self._best_stats  : dict = {}
        self._history     : list = []

    def run(
        self,
        weights  : dict  = None,
        x0       : dict  = None,
        max_iter : int   = 500,
        tol      : float = 1e-3,
        verbose  : bool  = True,
    ) -> dict:
        weights = self._parse_weights(weights)
        x0_dict = x0 or self.factory.default_tunables()

        keys       = list(x0_dict.keys())
        x0_log     = np.log10([x0_dict[k] for k in keys])
        bounds_log = np.array([
            (np.log10(self.factory.TUNABLE_BOUNDS[k][0]),
             np.log10(self.factory.TUNABLE_BOUNDS[k][1]))
            for k in keys
        ])
        lo_log = bounds_log[:, 0]
        hi_log = bounds_log[:, 1]

        if verbose:
            print("── Tuner: evaluating x0 ─────────────────────────────────")
        ref_stats = self._simulate(x0_dict)
        ref       = self._reference(ref_stats, weights)

        if verbose:
            print("── Tuner starting ───────────────────────────────────────")
            print(f"   weights  : {weights}")
            print(f"   max_iter : {max_iter}  |  tol: {tol}")
            print(f"   x0 stats : OS={ref_stats['overshoot_%']:.2f}%  "
                  f"Tr={ref_stats['rise_time_s']*1e3:.2f}ms  "
                  f"Ts={ref_stats['settling_time_s']*1e3:.2f}ms  "
                  f"SSE={ref_stats['steady_state_error']:.4f}")
            print("─────────────────────────────────────────────────────────")

        self._iter    = 0
        self._history = []

        def objective(x_log):
            x_log  = np.clip(x_log, lo_log, hi_log)
            params = dict(zip(keys, 10 ** x_log))
            try:
                stats = self._simulate(params)
            except Exception as exc:
                if verbose:
                    print(f"   [iter {self._iter:>4d}] FAILED: {exc}")
                return 1e6

            cost = self._cost(stats, weights, ref)
            self._history.append({
                "iter"   : self._iter,
                "cost"   : cost,
                "params" : params.copy(),
                "stats"  : stats,
            })
            self._iter += 1

            if verbose:
                print(
                    f"   [iter {self._iter:>4d}] cost={cost:.5f} | "
                    f"OS={stats['overshoot_%']:6.2f}%  "
                    f"Tr={stats['rise_time_s']*1e3:.2f}ms  "
                    f"Ts={stats['settling_time_s']*1e3:.2f}ms  "
                    f"SSE={stats['steady_state_error']:.4f}"
                )
            return cost

        result = minimize(
            objective,
            x0_log,
            method  = "Nelder-Mead",
            options = {"maxiter": max_iter, "xatol": tol, "fatol": tol, "adaptive": True},
        )

        self._best_params = dict(zip(keys, 10 ** np.clip(result.x, lo_log, hi_log)))
        self._best_stats  = self._simulate(self._best_params)

        if verbose:
            print("\n── Tuner result ─────────────────────────────────────────")
            print(f"   success  : {result.success}  ({result.message})")
            print(f"   iters    : {result.nit}  |  evals: {result.nfev}")
            self._print_params(self._best_params)
            self._print_stats(self._best_stats)
            print(self.build_best().configure())
            print("─────────────────────────────────────────────────────────")

        return {
            "best_params" : self._best_params,
            "best_stats"  : self._best_stats,
            "history"     : self._history,
            "result"      : result,
        }

    def best_config(self) -> PIDConfig:
        if not self._best_params:
            raise RuntimeError("Call run() before best_config().")
        return self.factory._make_config(self.factory._apply_defaults(self._best_params))

    def build_best(self) -> PID:
        if not self._best_params:
            raise RuntimeError("Call run() before build_best().")
        return self.factory.build(**self._best_params)

    def plot_history(self):
        if not self._history:
            print("No history — call run() first.")
            return
        iters = [h["iter"] for h in self._history]
        costs = [h["cost"] for h in self._history]
        plt.figure(figsize=(8, 4))
        plt.plot(iters, costs, marker="o", markersize=2, linewidth=0.8)
        plt.xlabel("Iteration")
        plt.ylabel("Weighted cost (normalised)")
        plt.title("Tuner convergence")
        plt.grid(True)
        plt.tight_layout()
        plt.show()

    def plot_best(self):
        pid = self.build_best()
        pid.drive(**self.drive_kwargs)
        pid.plot()

    def _simulate(self, params: dict) -> dict:
        pid       = self.factory.build(**params)
        pid.drive(**self.drive_kwargs)
        simulator = pid.circuit.simulator(
            temperature=pid.c.TEMP, nominal_temperature=pid.c.TEMP
        )
        analysis  = simulator.transient(step_time=pid.c.TSTEP, end_time=pid.c.TSTOP)
        stats     = pid._stats(analysis)
        stats["steady_state_error"] = (
            abs(stats["final_value"] - self.setpoint) / (abs(self.setpoint) or 1.0)
        )
        return stats

    @staticmethod
    def _reference(stats: dict, weights: dict) -> dict:
        ref = {}
        for key in Tuner.METRIC_KEYS:
            if weights.get(key, 0.0) > 0:
                ref[key] = max(abs(stats.get(key, 1.0)), 1e-12)
        return ref

    @staticmethod
    def _cost(stats: dict, weights: dict, ref: dict) -> float:
        cost = 0.0
        for key in Tuner.METRIC_KEYS:
            w = weights.get(key, 0.0)
            if w > 0.0:
                cost += w * max(stats.get(key, 0.0), 0.0) / ref.get(key, 1.0)
        return cost

    @staticmethod
    def _parse_weights(weights) -> dict:
        if weights is None:
            weights = {}
        unknown = set(weights) - set(Tuner.METRIC_KEYS)
        if unknown:
            raise ValueError(f"Unknown metric keys: {unknown}\nValid: {Tuner.METRIC_KEYS}")
        return {k: float(w) for k, w in weights.items()}

    @staticmethod
    def _print_params(p: dict):
        print("   best params:")
        for k, v in p.items():
            print(f"      {k:<14s} = {v:.4e}")

    @staticmethod
    def _print_stats(s: dict):
        print("   best stats:")
        print(f"      overshoot      = {s['overshoot_%']:.2f} %")
        print(f"      rise time      = {s['rise_time_s']*1e3:.3f} ms")
        print(f"      settling time  = {s['settling_time_s']*1e3:.3f} ms")
        print(f"      steady-state ε = {s['steady_state_error']:.5f}")