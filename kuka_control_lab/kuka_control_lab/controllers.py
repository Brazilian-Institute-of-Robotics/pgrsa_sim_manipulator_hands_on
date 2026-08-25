import inspect

import numpy as np

from . import dynamics as dyn
from kuka_forward_kinematics.kinematics import forward_kinematics, geometric_jacobian

# Postura de referência para a inércia nominal do PD local.
Q_NOMINAL = np.array([0.0, -1.0, 1.2, 0.0, 0.8, 0.0])
NOMINAL_INERTIA = np.diag(dyn.mass_matrix(Q_NOMINAL)).copy()


class BaseController:
    name = "base"
    space = "joint"  # espaço em que a lei mede o erro

    def __init__(
        self,
        wn=6.0,
        zeta=1.0,
        kp_scale=1.0,
        kd_scale=1.0,
        gravity_comp=True,
        tau_max=None,
    ):
        self.wn = float(wn)
        self.zeta = float(zeta)
        self.kp_scale = float(kp_scale)
        self.kd_scale = float(kd_scale)
        self.gravity_comp = bool(gravity_comp)
        self.tau_max = dyn.TAU_MAX if tau_max is None else np.asarray(tau_max, float)
        self.last = {}

    def saturate(self, tau):
        return np.clip(tau, -self.tau_max, self.tau_max)

    def compute(self, t, q, qd, ref_joint, ref_cart):
        raise NotImplementedError

    def gains_description(self):
        return f"wn={self.wn:.2f} rad/s, zeta={self.zeta:.2f}"


class LocalPDController(BaseController):

    name = "local_pd"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.kp = self.kp_scale * NOMINAL_INERTIA * self.wn**2
        self.kd = self.kd_scale * 2.0 * self.zeta * self.wn * NOMINAL_INERTIA

    def compute(self, t, q, qd, ref_joint, ref_cart):
        q_des, qd_des, _ = ref_joint(t)
        e = q_des - q
        de = qd_des - qd
        tau = self.kp * e + self.kd * de
        if self.gravity_comp:
            tau = tau + dyn.gravity_torque(q)
        self.last = dict(e=e, de=de, tau=tau)
        return self.saturate(tau)

    def gains_description(self):
        return (
            f"Kp={np.round(self.kp, 1).tolist()} N·m/rad, "
            f"Kd={np.round(self.kd, 1).tolist()} N·m·s/rad"
        )


class ComputedTorqueController(BaseController):

    name = "computed_torque"

    def __init__(self, *args, model_error=1.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.kp = self.kp_scale * self.wn**2
        self.kd = self.kd_scale * 2.0 * self.zeta * self.wn
        self.model_error = float(model_error)

    def compute(self, t, q, qd, ref_joint, ref_cart):
        q_des, qd_des, qdd_des = ref_joint(t)
        e = q_des - q
        de = qd_des - qd
        a_cmd = qdd_des + self.kd * de + self.kp * e
        M = dyn.mass_matrix(q) * self.model_error
        h = dyn.rnea(q, qd, np.zeros(6), gravity=True) * self.model_error
        tau = M @ a_cmd + h
        self.last = dict(e=e, de=de, tau=tau)
        return self.saturate(tau)

    def gains_description(self):
        return (
            f"Kp={self.kp:.1f} 1/s², Kd={self.kd:.1f} 1/s "
            f"(aceleração resolvida) — erro de modelo ×{self.model_error:.2f}"
        )


class OperationalSpaceController(BaseController):

    name = "operational_space"
    space = "cartesian"

    def __init__(self, *args, null_damping=8.0, null_kp=4.0, lam=0.05, **kwargs):
        super().__init__(*args, **kwargs)
        self.kp = self.kp_scale * self.wn**2
        self.kd = self.kd_scale * 2.0 * self.zeta * self.wn
        self.null_damping = float(null_damping)
        self.null_kp = float(null_kp)
        self.lam = float(lam)

    def compute(self, t, q, qd, ref_joint, ref_cart):
        x_des, xd_des, xdd_des = ref_cart(t)

        T, _ = forward_kinematics(q)
        x = T[:3, 3]
        J6 = geometric_jacobian(q)
        J = J6[:3, :]
        xd = J @ qd

        e = x_des - x
        de = xd_des - xd
        a_cmd = xdd_des + self.kd * de + self.kp * e

        M = dyn.mass_matrix(q)
        Minv = np.linalg.inv(M)
        A = J @ Minv @ J.T
        Lam = np.linalg.inv(A + self.lam**2 * np.eye(3))
        F = Lam @ a_cmd

        h = dyn.rnea(q, qd, np.zeros(6), gravity=True)
        tau_task = J.T @ F

        q_post, qd_post, _ = ref_joint(t)
        Jbar_T = Lam @ J @ Minv
        Nproj = np.eye(6) - J.T @ Jbar_T
        a_null = self.null_kp * (q_post - q) - self.null_damping * (qd - qd_post)
        tau_null = Nproj @ (M @ a_null)

        tau = tau_task + tau_null + h
        self.last = dict(
            e_cart=e,
            de_cart=de,
            tau=tau,
            F=F,
            manipulability=float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0))),
        )
        return self.saturate(tau)

    def gains_description(self):
        return (
            f"Kp={self.kp:.1f} 1/s², Kd={self.kd:.1f} 1/s (cartesiano, "
            f"aceleração resolvida) — amortecimento de espaço nulo "
            f"{self.null_damping:.1f}"
        )


STRATEGIES = {
    "local_pd": LocalPDController,
    "computed_torque": ComputedTorqueController,
    "operational_space": OperationalSpaceController,
}

STRATEGY_LABELS = {
    "local_pd": "Local (PD por junta)",
    "computed_torque": "Centralizado (torque computado)",
    "operational_space": "Espaço operacional (task-space)",
}


def accepted_kwargs(cls):
    """
    Nomes de parâmetro que `cls` aceita de fato, somando os próprios com os
    herdados de BaseController.

    Os nós deste pacote declaram um conjunto ÚNICO de parâmetros ROS e o
    repassam para qualquer uma das três estratégias — mas cada estratégia
    conhece só um subconjunto (`model_error` só existe no torque computado;
    `null_kp`, `null_damping` e `lam` só no espaço operacional). Alguém precisa
    descartar o resto.
    """
    names = set()
    for c in (cls, BaseController):
        for p in inspect.signature(c.__init__).parameters.values():
            if p.kind in (p.POSITIONAL_OR_KEYWORD, p.KEYWORD_ONLY):
                names.add(p.name)
    names.discard("self")
    return names


def build(strategy, **kwargs):
    """
    Instancia a estratégia pedida, descartando os parâmetros que ela não
    conhece.

    A filtragem é feita por INTROSPECÇÃO da assinatura, e não por uma lista
    escrita à mão. A diferença importa: uma lista manual precisa ser atualizada
    toda vez que um parâmetro novo é acrescentado a algum controlador, e quando
    isso é esquecido o erro só aparece em tempo de execução, na forma de um
    `TypeError: got an unexpected keyword argument`. Com introspecção, um
    parâmetro novo passa a ser aceito automaticamente pela classe que o declara
    e a ser descartado pelas demais, sem que nada aqui precise mudar.
    """
    if strategy not in STRATEGIES:
        raise ValueError(
            f"estratégia desconhecida: {strategy} " f"(use uma de {list(STRATEGIES)})"
        )
    cls = STRATEGIES[strategy]
    ok = accepted_kwargs(cls)
    return cls(**{k: v for k, v in kwargs.items() if k in ok})
