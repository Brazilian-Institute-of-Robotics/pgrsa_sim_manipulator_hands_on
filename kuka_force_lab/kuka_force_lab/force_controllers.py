import numpy as np

from kuka_control_lab import dynamics as dyn
from kuka_control_lab.controllers import BaseController, NOMINAL_INERTIA
from kuka_forward_kinematics.kinematics import forward_kinematics, geometric_jacobian


class ImpedanceController(BaseController):

    name = "impedance"
    space = "cartesian"

    def __init__(
        self,
        K_d=(2000.0, 2000.0, 1000.0),
        B_d=(400.0, 400.0, 250.0),
        M_d=(20.0, 20.0, 20.0),
        inertia_shaping=False,
        null_damping=8.0,
        null_kp=4.0,
        tau_max=None,
    ):
        super().__init__(wn=1.0, zeta=1.0, tau_max=tau_max)
        self.K_d = np.asarray(K_d, float)
        self.B_d = np.asarray(B_d, float)
        self.M_d = np.asarray(M_d, float)
        self.inertia_shaping = bool(inertia_shaping)
        self.null_damping = float(null_damping)
        self.null_kp = float(null_kp)
        self.f_ext = np.zeros(6)

    def set_external_wrench(self, w):
        self.f_ext = np.asarray(w, float)

    def compute(self, t, q, qd, ref_joint, ref_cart):
        x_des, xd_des, xdd_des = ref_cart(t)
        T, _ = forward_kinematics(q)
        x = T[:3, 3]
        J6 = geometric_jacobian(q)
        J = J6[:3, :]
        xd = J @ qd

        e = x_des - x
        de = xd_des - xd
        F = self.K_d * e + self.B_d * de

        if self.inertia_shaping:
            Lam = dyn.task_space_inertia(q, J6)[:3, :3]
            a_ref = xdd_des + (self.B_d * de + self.K_d * e) / self.M_d
            F = Lam @ a_ref + (np.eye(3) - Lam / self.M_d) @ self.f_ext[:3]

        h = dyn.rnea(q, qd, np.zeros(6), gravity=True)
        M = dyn.mass_matrix(q)
        Minv = np.linalg.inv(M)
        A = J @ Minv @ J.T
        Lam3 = np.linalg.inv(A + 0.05**2 * np.eye(3))
        Jbar_T = Lam3 @ J @ Minv
        Nproj = np.eye(6) - J.T @ Jbar_T

        q_post, qd_post, _ = ref_joint(t)
        a_null = self.null_kp * (q_post - q) - self.null_damping * qd
        tau = J.T @ F + Nproj @ (M @ a_null) + h

        self.last = dict(e_cart=e, F_cmd=F, x=x, xd=xd)
        return self.saturate(tau)

    def gains_description(self):
        return f"K_d={self.K_d.tolist()} N/m, B_d={self.B_d.tolist()} N·s/m" + (
            f", M_d={self.M_d.tolist()} kg"
            if self.inertia_shaping
            else " (sem moldagem de inércia)"
        )


class AdmittanceController(BaseController):
    name = "admittance"
    space = "cartesian"

    def __init__(
        self,
        M_d=(50.0, 50.0, 50.0),
        B_d=(1000.0, 1000.0, 1000.0),
        K_d=(0.0, 0.0, 0.0),
        F_des=(0.0, 0.0, 0.0),
        selection=(0.0, 0.0, 1.0),
        dt=0.005,
        max_deviation=0.15,
        deadband=0.5,
        position_servo_wn=30.0,
        position_servo_zeta=1.0,
        lam=0.05,
        tau_max=None,
    ):
        super().__init__(
            wn=position_servo_wn, zeta=position_servo_zeta, tau_max=tau_max
        )
        self.M_d = np.asarray(M_d, float)
        self.B_d = np.asarray(B_d, float)
        self.K_d = np.asarray(K_d, float)
        self.F_des = np.asarray(F_des, float)
        self.S = np.asarray(selection, float)  # 1 = direção regulada em força
        self.dt = float(dt)
        self.max_dev = float(max_deviation)
        self.deadband = float(deadband)
        self.lam = float(lam)

        self.x_a = np.zeros(3)
        self.xd_a = np.zeros(3)
        self.f_ext = np.zeros(6)

        self.kp_srv = NOMINAL_INERTIA * self.wn**2
        self.kd_srv = 2.0 * self.zeta * self.wn * NOMINAL_INERTIA

    def set_external_wrench(self, w):
        self.f_ext = np.asarray(w, float)

    def reset(self):
        self.x_a[:] = 0.0
        self.xd_a[:] = 0.0

    def compute(self, t, q, qd, ref_joint, ref_cart):
        F = self.f_ext[:3].copy()

        # Zona morta: filtra ruído da estimação de força e evita deriva por
        # viés. Só se aplica às direções NÃO reguladas em força — numa direção
        # em que se pede 20 N, a zona morta atrapalharia a regulação.
        for i in range(3):
            if self.S[i] < 0.5 and abs(F[i]) < self.deadband:
                F[i] = 0.0

        F_eff = F - self.F_des * self.S

        # Integração do modelo virtual (Euler semi-implícito: mais estável que
        # o explícito para sistemas massa-amortecedor).
        xdd = (F_eff - self.B_d * self.xd_a - self.K_d * self.x_a) / self.M_d
        self.xd_a = self.xd_a + xdd * self.dt
        self.x_a = self.x_a + self.xd_a * self.dt

        n = np.linalg.norm(self.x_a)
        if n > self.max_dev:
            self.x_a *= self.max_dev / n
            self.xd_a *= 0.5  # amortece ao bater na saturação

        # Comando cartesiano = nominal + desvio de admitância.
        x_nom, xd_nom, _ = ref_cart(t)
        x_cmd = x_nom + self.x_a
        xd_cmd = xd_nom + self.xd_a

        # Conversão para juntas pela pseudo-inversa amortecida do Jacobiano
        # AVALIADO NA CONFIGURAÇÃO MEDIDA.
        T, _ = forward_kinematics(q)
        x = T[:3, 3]
        J = geometric_jacobian(q)[:3, :]
        Jp = dyn.damped_pinv(J, self.lam)

        q_post, _, _ = ref_joint(t)
        dq = Jp @ (x_cmd - x)
        q_cmd = q + dq
        qd_cmd = Jp @ xd_cmd

        # Servo de posição interno (o papel do JTC no robô real).
        tau = (
            self.kp_srv * (q_cmd - q)
            + self.kd_srv * (qd_cmd - qd)
            + dyn.gravity_torque(q)
        )

        self.last = dict(x=x, x_cmd=x_cmd, deviation=self.x_a.copy(), F=F, F_eff=F_eff)
        del q_post
        return self.saturate(tau)

    def gains_description(self):
        return (
            f"M_d={self.M_d.tolist()} kg, B_d={self.B_d.tolist()} N·s/m, "
            f"K_d={self.K_d.tolist()} N/m, F_des={self.F_des.tolist()} N, "
            f"servo ωn={self.wn:.0f} rad/s"
        )


class HybridForcePositionController(BaseController):
    """
    CONTROLE HÍBRIDO FORÇA/POSIÇÃO (força direta, Raibert & Craig).

    Particiona o espaço da tarefa por uma matriz de seleção diagonal S_f
    (1 nas direções restringidas pelo ambiente) e S_p = I − S_f:

        direções de FORÇA    : PI sobre o erro de força
            u_f = Kf·(F_des − F) + Kfi·∫(F_des − F)dt
        direções de POSIÇÃO  : PD cartesiano com inércia de tarefa
            u_p = Λ·[ẍ_des + Kd·(ẋ_des − ẋ) + Kp·(x_des − x)]

        τ = Jᵀ·(S_f·u_f + S_p·u_p) + C(q,q̇)q̇ + g(q)

    Modos prontos:
        surface_polish — força em Z, posição em X e Y (polir/lixar um plano)
        wall_contact   — força em X, posição em Y e Z (encostar numa parede)
        free           — só posição

    O termo integral tem anti-windup por saturação: sem ele, o integrador
    acumularia durante todo o tempo em que o robô ainda estivesse no ar (erro
    de força = F_des, constante) e produziria um solavanco no instante do
    toque.
    """

    name = "hybrid"
    space = "cartesian"

    MODES = {
        "surface_polish": np.array([0.0, 0.0, 1.0]),
        "wall_contact": np.array([1.0, 0.0, 0.0]),
        "free": np.array([0.0, 0.0, 0.0]),
    }

    def __init__(
        self,
        mode="surface_polish",
        F_des=20.0,
        Kf=0.8,
        Kfi=6.0,
        Kv=400.0,
        int_limit=200.0,
        wn=8.0,
        zeta=1.0,
        dt=0.005,
        null_damping=8.0,
        null_kp=4.0,
        tau_max=None,
    ):
        super().__init__(wn=wn, zeta=zeta, tau_max=tau_max)
        self.mode = mode
        self.S_f = self.MODES.get(mode, self.MODES["surface_polish"]).copy()
        self.S_p = 1.0 - self.S_f
        self.F_des_scalar = float(F_des)
        self.F_des = self.F_des_scalar * self.S_f
        self.Kf = float(Kf)
        self.Kfi = float(Kfi)
        self.Kv = float(Kv)
        self.int_limit = float(int_limit)
        self.dt = float(dt)
        self.kp = wn**2
        self.kd = 2.0 * zeta * wn
        self.null_damping = float(null_damping)
        self.null_kp = float(null_kp)
        self.f_int = np.zeros(3)
        self.f_ext = np.zeros(6)

    def set_external_wrench(self, w):
        self.f_ext = np.asarray(w, float)

    def compute(self, t, q, qd, ref_joint, ref_cart):
        x_des, xd_des, xdd_des = ref_cart(t)
        T, _ = forward_kinematics(q)
        x = T[:3, 3]
        J6 = geometric_jacobian(q)
        J = J6[:3, :]
        xd = J @ qd

        # ── Malha de FORÇA (PI) nas direções restringidas ────────────────────
        # CONVENÇÃO DE SINAL (a fonte de erro mais comum em controle de força):
        # `f_ext` é a força que o AMBIENTE faz SOBRE O ROBÔ. Ao pressionar uma
        # mesa horizontal, a reação medida é para CIMA, +Z. Logo `F_des` é a
        # leitura desejada no sensor: pedir 20 N contra a mesa é pedir F_des =
        # +20 em Z. Para AUMENTAR essa leitura o robô tem de empurrar para
        # BAIXO — por isso o comando de força sai com sinal TROCADO em relação
        # ao erro. Inverter isto faz o robô fugir da superfície, que é
        # exatamente o sintoma de quem errou o sinal.
        F_meas = self.f_ext[:3]
        e_f = (self.F_des - F_meas) * self.S_f
        self.f_int = np.clip(
            self.f_int + e_f * self.dt, -self.int_limit, self.int_limit
        )
        # Realimentação direta de F_des (feedforward) + PI sobre o erro, tudo
        # com sinal invertido, e um termo de amortecimento sobre a velocidade
        # na direção normal. Sem esse amortecimento a direção de força fica sem
        # nenhuma oposição ao movimento enquanto o robô ainda está no ar, e a
        # aproximação vira queda livre até bater na superfície.
        u_f = -(self.F_des + self.Kf * e_f + self.Kfi * self.f_int) - self.Kv * (
            xd * self.S_f
        )

        # ── Malha de POSIÇÃO (PD com inércia de tarefa) ──────────────────────
        M = dyn.mass_matrix(q)
        Minv = np.linalg.inv(M)
        A = J @ Minv @ J.T
        Lam = np.linalg.inv(A + 0.05**2 * np.eye(3))
        a_cmd = xdd_des + self.kd * (xd_des - xd) + self.kp * (x_des - x)
        u_p = Lam @ a_cmd

        F_cmd = self.S_f * u_f + self.S_p * u_p

        h = dyn.rnea(q, qd, np.zeros(6), gravity=True)
        Jbar_T = Lam @ J @ Minv
        Nproj = np.eye(6) - J.T @ Jbar_T
        q_post, _, _ = ref_joint(t)
        a_null = self.null_kp * (q_post - q) - self.null_damping * qd

        tau = J.T @ F_cmd + Nproj @ (M @ a_null) + h

        self.last = dict(
            F_meas=F_meas.copy(),
            e_force=e_f.copy(),
            F_cmd=F_cmd.copy(),
            x=x,
            e_pos=(x_des - x) * self.S_p,
        )
        return self.saturate(tau)

    def gains_description(self):
        return (
            f"modo={self.mode}, F_des={self.F_des_scalar:.1f} N, "
            f"Kf={self.Kf}, Kfi={self.Kfi}, Kv={self.Kv}, "
            f"posição ωn={self.wn:.1f} ζ={self.zeta:.1f}"
        )


class PurePositionController(BaseController):
    """
    Controle de POSIÇÃO puro no espaço operacional — o grupo de controle do
    experimento. Não sabe que o ambiente existe: manda o TCP para a posição
    comandada e insiste. É o que produz forças de contato absurdas quando essa
    posição está do lado de dentro de uma superfície rígida, e é justamente
    esse número que justifica todo o resto do pacote.
    """

    name = "position_only"
    space = "cartesian"

    def __init__(self, wn=30.0, zeta=1.0, null_damping=8.0, null_kp=4.0, tau_max=None):
        super().__init__(wn=wn, zeta=zeta, tau_max=tau_max)
        self.kp = wn**2
        self.kd = 2.0 * zeta * wn
        self.null_damping = float(null_damping)
        self.null_kp = float(null_kp)
        self.f_ext = np.zeros(6)

    def set_external_wrench(self, w):
        self.f_ext = np.asarray(w, float)

    def compute(self, t, q, qd, ref_joint, ref_cart):
        x_des, xd_des, xdd_des = ref_cart(t)
        T, _ = forward_kinematics(q)
        x = T[:3, 3]
        J6 = geometric_jacobian(q)
        J = J6[:3, :]
        xd = J @ qd

        M = dyn.mass_matrix(q)
        Minv = np.linalg.inv(M)
        A = J @ Minv @ J.T
        Lam = np.linalg.inv(A + 0.05**2 * np.eye(3))
        a_cmd = xdd_des + self.kd * (xd_des - xd) + self.kp * (x_des - x)
        F = Lam @ a_cmd

        h = dyn.rnea(q, qd, np.zeros(6), gravity=True)
        Jbar_T = Lam @ J @ Minv
        Nproj = np.eye(6) - J.T @ Jbar_T
        q_post, _, _ = ref_joint(t)
        a_null = self.null_kp * (q_post - q) - self.null_damping * qd

        tau = J.T @ F + Nproj @ (M @ a_null) + h
        self.last = dict(x=x, e_cart=x_des - x)
        return self.saturate(tau)

    def gains_description(self):
        return f"posição pura, ωn={self.wn:.1f} rad/s, ζ={self.zeta:.1f}"


CONTROLLERS = {
    "position_only": PurePositionController,
    "impedance": ImpedanceController,
    "admittance": AdmittanceController,
    "hybrid": HybridForcePositionController,
}

LABELS = {
    "position_only": "Posição pura (sem controle de força)",
    "impedance": "Impedância (força indireta, torque)",
    "admittance": "Admitância (força indireta, posição)",
    "hybrid": "Híbrido força/posição (força direta)",
}
