import numpy as np

from . import dynamics as dyn
from kuka_forward_kinematics.kinematics import forward_kinematics, geometric_jacobian


class SimResult:
    """Histórico completo de uma simulação, pronto para análise e gráfico."""

    def __init__(self):
        self.t = []
        self.q = []
        self.qd = []
        self.qdd = []
        self.tau = []
        self.q_des = []
        self.x = []
        self.x_des = []
        self.f_ext = []
        self.diverged = False
        self.diverge_time = None
        self.meta = {}

    def finish(self):
        for k in ("t", "q", "qd", "qdd", "tau", "q_des", "x", "x_des", "f_ext"):
            setattr(self, k, np.array(getattr(self, k)))
        return self

    # ── Métricas ─────────────────────────────────────────────────────────────
    def joint_error(self):
        return self.q_des - self.q

    def cart_error(self):
        return self.x_des - self.x

    def metrics(
        self,
        settle_tol_rad=0.01,
        settle_tol_m=0.002,
        ripple_tol_rad=0.005,
        ripple_tol_m=0.002,
    ):
        """
        Métricas de desempenho e estabilidade da corrida.

        Nota sobre "instabilidade": com saturação de torque (que todo
        acionamento real tem) um sistema mal sintonizado quase nunca diverge
        para infinito — ele entra em CICLO LIMITE, oscilando em torno do alvo
        com amplitude constante. Por isso a detecção não se baseia só em
        divergência: mede-se a ONDULAÇÃO RESIDUAL (desvio padrão do erro no
        trecho final, quando a referência já parou) e o conteúdo de alta
        frequência desse mesmo trecho. Erro que não para de balançar depois
        que a referência parou é, para todo efeito prático, instabilidade.
        """
        e_j = np.linalg.norm(self.joint_error(), axis=1)
        e_c = np.linalg.norm(self.cart_error(), axis=1)
        n = len(self.t)
        tail = max(int(0.1 * n), 1)
        i_tail = int(0.6 * n)

        e_tail = e_j[i_tail:] if n > 10 else e_j
        ec_tail = e_c[i_tail:] if n > 10 else e_c
        t_tail = self.t[i_tail:] if n > 10 else self.t
        ripple = float(np.std(e_tail)) if len(e_tail) else 0.0
        ripple_c = float(np.std(ec_tail)) if len(ec_tail) else 0.0

        # A ondulação tem de ser medida NO ESPAÇO EM QUE O CONTROLADOR TRABALHA.
        # O controlador de espaço operacional não regula os ângulos de junta —
        # ele deixa a configuração interna do braço acomodar-se livremente no
        # espaço nulo. Medir a ondulação em radianos o classificaria como
        # instável mesmo com o TCP perfeitamente parado, que é o oposto da
        # verdade. Para ele a régua é o erro cartesiano; para os de junta, o
        # erro em radianos.
        space = self.meta.get("space", "joint")
        if space == "cartesian":
            osc_src, ripple_metric, ripple_lim = ec_tail, ripple_c, ripple_tol_m
        else:
            osc_src, ripple_metric, ripple_lim = e_tail, ripple, ripple_tol_rad
        osc = _oscillation_ratio(t_tail, osc_src) if len(osc_src) > 32 else 0.0
        decay = _decay_ratio(osc_src)

        out = {
            "diverged": bool(self.diverged),
            "diverge_time": self.diverge_time,
            "rms_joint_rad": float(np.sqrt(np.mean(e_j**2))),
            "max_joint_rad": float(np.max(e_j)),
            "final_joint_rad": float(np.mean(e_j[-tail:])),
            "rms_cart_mm": float(np.sqrt(np.mean(e_c**2)) * 1000.0),
            "max_cart_mm": float(np.max(e_c) * 1000.0),
            "final_cart_mm": float(np.mean(e_c[-tail:]) * 1000.0),
            "tau_rms_Nm": float(np.sqrt(np.mean(np.sum(self.tau**2, axis=1)))),
            "tau_peak_Nm": float(np.max(np.abs(self.tau))),
            "settling_time_s": _settling_time(self.t, e_j, settle_tol_rad),
            "settling_time_cart_s": _settling_time(self.t, e_c, settle_tol_m),
            "overshoot_pct": self.overshoot(),
            "ripple_rad": ripple,
            "ripple_mm": ripple_c * 1000.0,
            "osc_ratio": osc,
            "osc_freq_hz": (
                dominant_frequency(t_tail, osc_src) if len(osc_src) > 32 else 0.0
            ),
        }
        out["decay_ratio"] = decay
        # Instável = a oscilação NÃO está morrendo. Amplitude residual grande
        # sozinha não basta: uma resposta lenta ainda em convergência também
        # tem desvio alto no fim da janela, e chamá-la de instável seria
        # confundir lentidão com perda de estabilidade. Por isso exige-se, além
        # da amplitude, que a segunda metade da janela final não seja
        # sensivelmente menor que a primeira — assinatura de ciclo limite.
        out["unstable"] = bool(
            out["diverged"] or (ripple_metric > ripple_lim and decay > 0.5)
        )
        return out

    def overshoot(self):
        """
        Sobressinal (%) na junta de maior deslocamento, definição de livro:
        quanto a resposta passa do alvo, em porcentagem do próprio degrau.
        """
        if len(self.t) < 5:
            return 0.0
        q_target = self.q_des[-1]
        q_start = self.q[0]
        d = q_target - q_start
        j = int(np.argmax(np.abs(d)))
        if abs(d[j]) < 1e-6:
            return 0.0
        traj = (self.q[:, j] - q_target[j]) * np.sign(d[j])
        return float(max(0.0, np.max(traj) / abs(d[j]) * 100.0))


def _settling_time(t, e, tol):
    """Primeiro instante a partir do qual o erro NUNCA mais sai da faixa."""
    inside = e < tol
    if not np.any(inside):
        return -1.0
    idx = len(e) - 1
    while idx > 0 and inside[idx - 1]:
        idx -= 1
    if not inside[-1]:
        return -1.0
    return float(t[idx])


def _decay_ratio(e):
    """
    Razão entre a ondulação da segunda e da primeira metade da janela final.
    ≈0 → a oscilação está morrendo (estável, só lento);
    ≈1 → amplitude constante (ciclo limite);
    >1 → crescendo (divergindo).
    """
    n = len(e)
    if n < 16:
        return 0.0
    a = np.std(e[: n // 2])
    b = np.std(e[n // 2 :])
    if a < 1e-12:
        return 0.0 if b < 1e-12 else 10.0
    return float(b / a)


def _overshoot_unused(e):
    """Sobressinal normalizado (%) em relação ao erro inicial."""
    if len(e) < 5 or e[0] <= 1e-9:
        return 0.0
    n = len(e)
    tail = max(int(0.1 * n), 1)
    final = np.mean(e[-tail:])
    peak = np.max(e[int(0.05 * n) :])
    return float(max(0.0, (peak - final) / e[0] * 100.0))


def _oscillation_ratio(t, e, f_cut=2.0):
    """
    Fração da energia espectral do erro acima de `f_cut` Hz, após remover a
    média. Alto = o erro está oscilando em vez de convergir suavemente —
    assinatura de Kp alto demais para o Kd disponível.
    """
    if len(e) < 32:
        return 0.0
    dt = float(np.mean(np.diff(t)))
    x = e - np.mean(e)
    spec = np.abs(np.fft.rfft(x))
    freqs = np.fft.rfftfreq(len(x), d=dt)
    total = np.sum(spec)
    if total <= 1e-12:
        return 0.0
    return float(np.sum(spec[freqs > f_cut]) / total)


def dominant_frequency(t, e, f_min=0.2):
    """Frequência dominante do erro (Hz) — usada para medir o período crítico
    Tu na estimativa de Ziegler-Nichols."""
    if len(e) < 32:
        return 0.0
    dt = float(np.mean(np.diff(t)))
    x = e - np.mean(e)
    spec = np.abs(np.fft.rfft(x))
    freqs = np.fft.rfftfreq(len(x), d=dt)
    mask = freqs > f_min
    if not np.any(mask):
        return 0.0
    return float(freqs[mask][int(np.argmax(spec[mask]))])


def simulate(
    controller,
    ref_joint,
    ref_cart,
    q0=None,
    qd0=None,
    duration=6.0,
    control_rate=200.0,
    physics_rate=2000.0,
    viscous=None,
    coulomb=None,
    encoder_noise=0.0,
    force_noise=0.0,
    feedback_delay=0.0,
    environment=None,
    payload=0.0,
    diverge_limit=50.0,
    seed=0,
):
    """
    Roda a malha fechada e devolve um SimResult.

    controller   : objeto com .compute(t, q, qd, ref_joint, ref_cart) -> tau
    environment  : objeto com .wrench(x, xd) -> wrench 6D no frame da base,
                   ou None para movimento livre (ver kuka_force_lab).
    payload      : massa (kg) acoplada ao flange — a PLANTA sempre a "sente";
                   se o controlador não a modelar, isso vira erro de modelagem.
    """
    rng = np.random.default_rng(seed)
    dyn.set_payload(payload)

    q = np.array(q0, float) if q0 is not None else ref_joint(0.0)[0].copy()
    qd = np.array(qd0, float) if qd0 is not None else np.zeros(6)

    dt_c = 1.0 / control_rate
    dt_p = 1.0 / physics_rate
    substeps = max(int(round(dt_c / dt_p)), 1)
    dt_p = dt_c / substeps

    visc = dyn.VISCOUS_DEFAULT if viscous is None else np.asarray(viscous, float)
    coul = dyn.COULOMB_DEFAULT if coulomb is None else np.asarray(coulomb, float)

    delay_steps = int(round(feedback_delay / dt_c))
    buf = [(q.copy(), qd.copy()) for _ in range(delay_steps + 1)]

    res = SimResult()
    res.meta = dict(
        controller=controller.name,
        space=getattr(controller, "space", "joint"),
        control_rate=control_rate,
        physics_rate=physics_rate,
        duration=duration,
        payload=payload,
        encoder_noise=encoder_noise,
        force_noise=force_noise,
        feedback_delay=feedback_delay,
        environment=(environment.name if environment is not None else "free"),
        gains=controller.gains_description(),
    )

    n_steps = int(round(duration / dt_c))
    t = 0.0

    for k in range(n_steps):
        # ── Ambiente: estado de contato ANTES de calcular o controle ─────────
        # A ordem importa. O controlador de força precisa da leitura do sensor
        # correspondente à configuração ATUAL — medir depois de já ter agido
        # introduziria um passo inteiro de atraso artificial na malha de força,
        # que é justamente onde essa malha é mais sensível.
        if environment is not None:
            if hasattr(environment, "step"):
                environment.step(t)
            T_now, _ = forward_kinematics(q)
            J_now = geometric_jacobian(q)
            x_now = T_now[:3, 3]
            xd_now = (J_now @ qd)[:3]
            w_ext = environment.wrench(x_now, xd_now)
        else:
            w_ext = None

        # ── Realimentação: atrasada e ruidosa, como na vida real ─────────────
        q_fb, qd_fb = buf[0]
        if encoder_noise > 0.0:
            q_fb = q_fb + rng.normal(0.0, encoder_noise, 6)
            qd_fb = qd_fb + rng.normal(0.0, encoder_noise * 10.0, 6)

        if hasattr(controller, "set_external_wrench"):
            w_meas = np.zeros(6) if w_ext is None else np.asarray(w_ext, float)
            if force_noise > 0.0:
                w_meas = w_meas + rng.normal(0.0, force_noise, 6)
            controller.set_external_wrench(w_meas)

        tau = controller.compute(t, q_fb, qd_fb, ref_joint, ref_cart)
        tau = np.asarray(tau, float)

        # ── Integração da planta (RK4) ───────────────────────────────────────
        # M(q) é avaliada UMA vez por sub-passo e reaproveitada nos 4 estágios
        # do RK4; os termos de Coriolis/gravidade (que é onde está quase toda a
        # variação rápida) continuam sendo reavaliados em cada estágio. Como as
        # quatro configurações de um sub-passo diferem entre si por O(dt), o
        # erro introduzido é desprezível a 1 kHz e o ganho de tempo é de ~4×,
        # que é o que torna a varredura de ganhos viável em minutos.
        for _ in range(substeps):
            M = dyn.mass_matrix(q)
            Minv = np.linalg.inv(M)

            def acc(qq, qv):
                h = dyn.rnea(qq, qv, np.zeros(6), gravity=True, f_ext_tcp=w_ext)
                return Minv @ (tau - h - visc * qv - coul * np.tanh(50.0 * qv))

            k1q, k1v = qd, acc(q, qd)
            k2q, k2v = (
                qd + 0.5 * dt_p * k1v,
                acc(q + 0.5 * dt_p * k1q, qd + 0.5 * dt_p * k1v),
            )
            k3q, k3v = (
                qd + 0.5 * dt_p * k2v,
                acc(q + 0.5 * dt_p * k2q, qd + 0.5 * dt_p * k2v),
            )
            k4q, k4v = (qd + dt_p * k3v, acc(q + dt_p * k3q, qd + dt_p * k3v))
            q = q + dt_p / 6.0 * (k1q + 2 * k2q + 2 * k3q + k4q)
            qd = qd + dt_p / 6.0 * (k1v + 2 * k2v + 2 * k3v + k4v)

            if not np.all(np.isfinite(q)) or not np.all(np.isfinite(qd)):
                break

        buf.append((q.copy(), qd.copy()))
        if len(buf) > delay_steps + 1:
            buf.pop(0)

        if np.all(np.isfinite(q)) and np.all(np.isfinite(qd)):
            qdd = dyn.forward_dynamics(
                q, qd, tau, f_ext_tcp=w_ext, viscous=visc, coulomb=coul
            )
        else:
            qdd = np.full(6, np.nan)

        q_des, _, _ = ref_joint(t)
        x_des, _, _ = ref_cart(t)
        T, _ = forward_kinematics(q)

        res.t.append(t)
        res.q.append(q.copy())
        res.qd.append(qd.copy())
        res.qdd.append(qdd.copy())
        res.tau.append(tau.copy())
        res.q_des.append(q_des.copy())
        res.x.append(T[:3, 3].copy())
        res.x_des.append(np.asarray(x_des, float).copy())
        res.f_ext.append(np.zeros(6) if w_ext is None else np.asarray(w_ext, float))

        # ── Guarda de divergência ────────────────────────────────────────────
        if (
            not np.all(np.isfinite(q))
            or not np.all(np.isfinite(qd))
            or np.max(np.abs(qd)) > diverge_limit
            or np.max(np.abs(q)) > 4 * np.pi
        ):
            res.diverged = True
            res.diverge_time = t
            break

        t += dt_c

    dyn.set_payload(0.0)
    return res.finish()
