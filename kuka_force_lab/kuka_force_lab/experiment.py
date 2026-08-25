import numpy as np

from kuka_control_lab import trajectories as traj_mod
from kuka_forward_kinematics.kinematics import forward_kinematics

from . import environment as env_mod
from . import force_controllers as fc

Z_SURFACE_DEFAULT = 0.96
OVERSHOOT_DEFAULT = 0.02
APPROACH_TIME_DEFAULT = 3.0


def build_scene(
    posture="pick",
    z_surface=Z_SURFACE_DEFAULT,
    overshoot=OVERSHOOT_DEFAULT,
    approach_time=APPROACH_TIME_DEFAULT,
    material="madeira",
    surface="plane",
    stiffness=None,
    lateral=0.0,
    lateral_time=None,
):

    q0 = traj_mod.get_posture(posture)
    x0 = forward_kinematics(q0)[0][:3, 3]
    x_goal = np.array([x0[0], x0[1] + lateral, z_surface - overshoot])

    ref_joint = traj_mod.JointTrajectory(q0, q0, kind="step")  # postura fixa
    if lateral != 0.0 and lateral_time:
        ref_cart = _TwoPhase(
            x0, x_goal, z_surface - overshoot, approach_time, float(lateral_time)
        )
    else:
        ref_cart = traj_mod.CartesianTrajectory(
            x0, x_goal, duration=approach_time, kind="quintic"
        )

    kw = {}
    if stiffness is not None:
        kw["stiffness"] = float(stiffness)
        kw["damping"] = 6.0 * np.sqrt(float(stiffness))
    env = (
        env_mod.build(
            surface,
            material,
            origin=(0.0, 0.0, z_surface),
            normal=(0.0, 0.0, 1.0),
            **kw,
        )
        if surface in ("plane", "moving")
        else env_mod.build(surface, material, **kw)
    )

    info = dict(
        x_start=[float(v) for v in x0],
        x_goal=[float(v) for v in x_goal],
        z_surface=float(z_surface),
        overshoot_mm=float(overshoot * 1000.0),
        material=material,
        stiffness=float(getattr(env, "K", 0.0)),
    )
    return ref_joint, ref_cart, q0, env, info


class _TwoPhase:

    def __init__(self, x0, x_goal, z_target, t_down, t_slide):
        self.a = traj_mod.CartesianTrajectory(
            x0, np.array([x0[0], x0[1], z_target]), duration=t_down, kind="quintic"
        )
        self.x_mid = np.array([x0[0], x0[1], z_target])
        self.b = traj_mod.CartesianTrajectory(
            self.x_mid, x_goal, duration=t_slide, kind="quintic"
        )
        self.t_down = float(t_down)

    def __call__(self, t):
        if t <= self.t_down:
            return self.a(t)
        return self.b(t - self.t_down)


def build_controller(kind, f_des=20.0, **kw):
    if kind == "position_only":
        return fc.PurePositionController(
            wn=float(kw.get("wn", 30.0)), zeta=float(kw.get("zeta", 1.0))
        )
    if kind == "impedance":
        return fc.ImpedanceController(
            K_d=kw.get("K_d", (2000.0, 2000.0, 1000.0)),
            B_d=kw.get("B_d", (400.0, 400.0, 250.0)),
            M_d=kw.get("M_d", (20.0, 20.0, 20.0)),
            inertia_shaping=bool(kw.get("inertia_shaping", False)),
        )
    if kind == "admittance":
        return fc.AdmittanceController(
            M_d=kw.get("M_d", (50.0, 50.0, 50.0)),
            B_d=kw.get("B_d", (1000.0, 1000.0, 1000.0)),
            K_d=kw.get("K_d", (0.0, 0.0, 0.0)),
            F_des=(0.0, 0.0, float(f_des)),
            selection=(0.0, 0.0, 1.0),
            dt=1.0 / float(kw.get("control_rate", 200.0)),
            position_servo_wn=float(kw.get("servo_wn", 30.0)),
        )
    if kind == "hybrid":
        return fc.HybridForcePositionController(
            mode=kw.get("mode", "surface_polish"),
            F_des=float(f_des),
            Kf=float(kw.get("Kf", 0.8)),
            Kfi=float(kw.get("Kfi", 6.0)),
            Kv=float(kw.get("Kv", 400.0)),
            wn=float(kw.get("wn", 8.0)),
            dt=1.0 / float(kw.get("control_rate", 200.0)),
        )
    raise ValueError(
        f"controlador desconhecido: {kind} " f"(use um de {list(fc.CONTROLLERS)})"
    )


def force_metrics(res, f_des=20.0, z_surface=Z_SURFACE_DEFAULT, tail_frac=0.3):
    fz = res.f_ext[:, 2]
    n = len(fz)
    i0 = int((1.0 - tail_frac) * n)
    tail = fz[i0:] if n > 10 else fz
    pen = np.maximum(z_surface - res.x[:, 2], 0.0)
    steady = float(np.mean(tail))
    return dict(
        peak_N=float(np.max(np.abs(fz))),
        steady_N=steady,
        error_N=float(steady - f_des),
        ripple_N=float(np.std(tail)),
        penetration_mm=float(np.mean(pen[i0:]) * 1000.0),
        max_penetration_mm=float(np.max(pen) * 1000.0),
        tau_peak_Nm=float(np.max(np.abs(res.tau))),
        contact_fraction=float(np.mean(np.abs(fz) > 0.5)),
    )
