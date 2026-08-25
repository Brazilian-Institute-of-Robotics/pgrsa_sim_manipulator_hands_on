import numpy as np

from kuka_forward_kinematics.kinematics import forward_kinematics

# Posturas de referência do workspace.
POSTURES = {
    "home": np.array([0.0, -1.57, 1.57, 0.0, 0.30, 0.0]),
    "pick": np.array([0.0, -1.00, 1.20, 0.0, 0.80, 0.0]),
    "place": np.array([1.20, -1.00, 1.00, 0.0, 0.50, 0.0]),
    "stretch": np.array([0.0, -1.40, 0.60, 0.0, 0.90, 0.0]),
    "left": np.array([1.57, -1.20, 1.30, 0.0, 0.70, 0.0]),
}


def get_posture(name, custom=None):
    if name == "custom" and custom is not None and len(custom) == 6:
        return np.asarray(custom, dtype=float)
    return POSTURES.get(name, POSTURES["pick"]).copy()


class JointTrajectory:
    """Referência no espaço das juntas: q_des(t), qd_des(t), qdd_des(t)."""

    def __init__(
        self, q_start, q_goal, duration=3.0, kind="quintic", amplitude=0.35, freq=0.4
    ):
        self.q0 = np.asarray(q_start, float)
        self.qf = np.asarray(q_goal, float)
        self.T = float(duration)
        self.kind = kind
        self.amp = float(amplitude)
        self.freq = float(freq)

    def __call__(self, t):
        if self.kind == "step":
            return self.qf.copy(), np.zeros(6), np.zeros(6)

        if self.kind == "sine":
            w = 2 * np.pi * self.freq
            q = self.q0 + self.amp * np.sin(w * t)
            qd = self.amp * w * np.cos(w * t)
            qdd = -self.amp * w * w * np.sin(w * t)
            return q, qd, qdd

        # quintic
        s = min(max(t / self.T, 0.0), 1.0)
        p = 10 * s**3 - 15 * s**4 + 6 * s**5
        dp = (30 * s**2 - 60 * s**3 + 30 * s**4) / self.T
        ddp = (60 * s - 180 * s**2 + 120 * s**3) / (self.T**2)
        if t >= self.T:
            dp = 0.0
            ddp = 0.0
        d = self.qf - self.q0
        return self.q0 + p * d, dp * d, ddp * d


class CartesianTrajectory:

    def __init__(self, x_start, x_goal, duration=3.0, kind="quintic"):
        self.x0 = np.asarray(x_start, float)
        self.xf = np.asarray(x_goal, float)
        self.T = float(duration)
        self.kind = kind

    def __call__(self, t):
        if self.kind == "step":
            return self.xf.copy(), np.zeros(3), np.zeros(3)
        s = min(max(t / self.T, 0.0), 1.0)
        p = 10 * s**3 - 15 * s**4 + 6 * s**5
        dp = (30 * s**2 - 60 * s**3 + 30 * s**4) / self.T
        ddp = (60 * s - 180 * s**2 + 120 * s**3) / (self.T**2)
        if t >= self.T:
            dp = 0.0
            ddp = 0.0
        d = self.xf - self.x0
        return self.x0 + p * d, dp * d, ddp * d


def cartesian_from_joint(traj_joint, t):
    """Converte a referência de junta em referência cartesiana (só posição)."""
    q, _, _ = traj_joint(t)
    T, _ = forward_kinematics(q)
    return T[:3, 3]


def make_step_around(posture="pick", delta=0.05, custom=None):
    q0 = get_posture(posture, custom)
    qf = q0 + float(delta)
    tj = JointTrajectory(q0, qf, kind="step")
    x0 = forward_kinematics(q0)[0][:3, 3]
    xf = forward_kinematics(qf)[0][:3, 3]
    tc = CartesianTrajectory(x0, xf, kind="step")
    return tj, tc, q0, qf


def make_pair(
    start="home",
    goal="pick",
    duration=3.0,
    kind="quintic",
    custom_start=None,
    custom_goal=None,
):

    q0 = get_posture(start, custom_start)
    qf = get_posture(goal, custom_goal)
    tj = JointTrajectory(q0, qf, duration=duration, kind=kind)
    x0 = forward_kinematics(q0)[0][:3, 3]
    xf = forward_kinematics(qf)[0][:3, 3]
    tc = CartesianTrajectory(x0, xf, duration=duration, kind=kind)
    return tj, tc, q0, qf
