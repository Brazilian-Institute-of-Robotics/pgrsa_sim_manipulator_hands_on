"""
dynamics.py — Modelo dinâmico rígido completo do KUKA KR70 R2100.

Implementa M(q), C(q,qd)*qd e g(q) por Newton-Euler recursivo (RNEA), a partir
dos parâmetros inerciais REAIS do URDF (massa, centro de massa e tensor de
inércia de cada elo), e a dinâmica direta

    qdd = M(q)^-1 * ( tau - C(q,qd)*qd - g(q) - tau_atrito )

que é o que permite FECHAR A MALHA de verdade: dado um torque, o modelo diz
como o robô acelera. Sem isso não há como comparar leis de controle — só
observá-las.

CONSISTÊNCIA DE MODELO
----------------------
A cinemática (transformações estáticas elo-a-elo) é IMPORTADA de
`kuka_forward_kinematics.kinematics`, que é a fonte única de verdade de
cinemática do workspace (usada também por kuka_inverse_kinematics,
kuka_force_control e kuka_state_estimation). Este módulo NÃO redefine
nenhuma cadeia cinemática própria — apenas acrescenta a camada inercial.

Convenções do RNEA implementado (Craig, cap. 6):
  - frame i = frame do elo i (filho da junta i), i = 1..6;
  - A_i(q_i) = T_STATIC[i-1] @ rot_z(q_i) leva do frame i-1 ao frame i;
  - todas as juntas giram em torno do eixo z LOCAL (verdade no URDF do KR70);
  - gravidade entrada como aceleração da base a_0 = [0, 0, +9.81].
"""

import numpy as np

# Fonte única de verdade da cinemática do workspace.
from kuka_forward_kinematics.kinematics import (  # noqa: F401
    forward_kinematics,
    geometric_jacobian,
    tcp_position,
    JOINT_NAMES,
    JOINT_LIMITS,
    _T_STATIC,
    _rot_z,
)

G = 9.81
N_JOINTS = 6


# ── Parâmetros inerciais extraídos do URDF (kr70_r2100_macro.xacro) ───────────
# Para cada elo: massa (kg), posição do CoM no frame do elo (m), rpy do frame
# principal de inércia, e as inércias principais (kg·m²) nesse frame.
_LINK_INERTIAL = [
    dict(m=175.205,
         com=[0.0386086184755001, -0.0346095020119289, -0.237989526554607],
         rpy=[-0.7923258603339018, -0.8943966305036485, 2.6984590381230245],
         I=[5.40709339054184, 7.71982652844139, 9.08867781161129]),
    dict(m=108.86,
         com=[0.322390685283851, 0.0011177475656806899, -0.0921965092779717],
         rpy=[2.0043980356557825, -1.4175712597169723, -0.4411071139781909],
         I=[15.3115976467419, 1.25648697469284, 15.5308363638855]),
    dict(m=76.987,
         com=[0.0313301089794381, -0.020586332757478502, 0.14882084299946702],
         rpy=[-1.4493563584878768, -1.5363130245492544, 2.877566531592554],
         I=[1.92337293034476, 1.27849052849714, 2.39171458731211]),
    dict(m=36.937064569,
         com=[1.89432198144002e-4, 0.024470785786346698, -0.41246255028967],
         rpy=[0.828589798090399, -1.4728909399076815, -1.576276607483943],
         I=[0.155169399999395, 2.40535850557874, 2.40600523859326]),
    dict(m=16.209,
         com=[0.0338747121969276, 4.7815719661916203e-4, -0.0563914652353631],
         rpy=[-1.5546090234283156, -1.0268303292864105, -0.0223629762980488],
         I=[0.104253153076611, 0.080454547765652, 0.129775394933814]),
    dict(m=3.62,
         com=[-0.00392541436464088, 5.41436464088398e-6, -0.00152140883977901],
         rpy=[-0.2657207469665607, 5.14585534055515e-4, 1.571146297496868],
         I=[0.00543884122921238, 0.00497050004638771, 0.00667989273554355]),
]


def _rpy_to_R(r, p, y):
    """rpy (fixed-axis XYZ, convenção URDF) -> matriz de rotação 3x3."""
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# Pré-computa massa, CoM e tensor de inércia no frame do elo.
MASSES = np.array([lk['m'] for lk in _LINK_INERTIAL])
COM = np.array([lk['com'] for lk in _LINK_INERTIAL])
INERTIA = []
for _lk in _LINK_INERTIAL:
    _R = _rpy_to_R(*_lk['rpy'])
    INERTIA.append(_R @ np.diag(_lk['I']) @ _R.T)
INERTIA = np.array(INERTIA)

# Inércia do rotor de cada acionamento, refletida pelo redutor (kg·m²).
# O URDF descreve apenas a geometria dos ELOS; num robô industrial real, a
# inércia do motor multiplicada pelo quadrado da razão de redução é da mesma
# ordem (ou maior) que a do elo, sobretudo no punho. Sem esse termo, M(q)
# ficaria com número de condição da ordem de 10⁴ e as juntas 5 e 6 apareceriam
# com inércia quase nula — o que produziria ganhos de PD local absurdamente
# pequenos e uma comparação distorcida.
ROTOR_INERTIA = np.array([12.0, 12.0, 6.0, 2.0, 1.5, 0.8])

# Carga acoplada ao flange (gripper + peça). Alterável em tempo de execução
# via set_payload() — usado para estudar robustez a erro de modelagem.
PAYLOAD_MASS_DEFAULT = 0.0
_payload = dict(m=PAYLOAD_MASS_DEFAULT, com=np.zeros(3))


def set_payload(mass: float, com=(0.0, 0.0, 0.10)):
    """Acopla uma carga pontual ao elo 6 (frame do flange)."""
    _payload['m'] = float(mass)
    _payload['com'] = np.asarray(com, dtype=float)


def get_payload():
    return _payload['m'], _payload['com'].copy()


def _link_chain(q):
    """Rotações e translações elo-a-elo: R[i], p[i] levam do frame i ao i+1."""
    R = np.zeros((N_JOINTS, 3, 3))
    p = np.zeros((N_JOINTS, 3))
    for i in range(N_JOINTS):
        A = _T_STATIC[i] @ _rot_z(q[i])
        R[i] = A[:3, :3]
        p[i] = A[:3, 3]
    return R, p


def _effective_link(i):
    """Massa/CoM/inércia do elo i, já somando o payload se for o elo 6."""
    m = MASSES[i]
    c = COM[i]
    I = INERTIA[i]
    if i == N_JOINTS - 1 and _payload['m'] > 0.0:
        mp, cp = _payload['m'], _payload['com']
        m_tot = m + mp
        c_tot = (m * c + mp * cp) / m_tot
        # Steiner para transferir as duas inércias ao novo CoM comum.
        def _steiner(I0, mass, d):
            return I0 + mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        I_tot = (_steiner(I, m, c - c_tot)
                 + _steiner(np.zeros((3, 3)), mp, cp - c_tot))
        return m_tot, c_tot, I_tot
    return m, c, I


def _cross(a, b):
    """Produto vetorial 3D explícito. Para vetores de tamanho 3, isto é cerca
    de 5× mais rápido que np.cross — e o RNEA faz dezenas deles por chamada,
    então a diferença aparece na varredura de ganhos (centenas de simulações)."""
    return np.array((a[1] * b[2] - a[2] * b[1],
                     a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]))


def _skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def _links_effective():
    """Lista [(m, c, I)] dos 6 elos, já com o payload embutido no elo 6."""
    return [_effective_link(i) for i in range(N_JOINTS)]


def rnea(q, qd, qdd, gravity=True, f_ext_tcp=None):
    """
    Newton-Euler recursivo. Retorna o vetor de torques (6,) necessário para
    produzir (q, qd, qdd).

    f_ext_tcp: wrench externo [fx,fy,fz,tx,ty,tz] aplicado NO TCP e expresso
               no frame da BASE (reação do ambiente sobre o robô).
    """
    q = np.asarray(q, float)
    qd = np.asarray(qd, float)
    qdd = np.asarray(qdd, float)

    R, p = _link_chain(q)
    links = _links_effective()
    z = np.array([0.0, 0.0, 1.0])

    # ── Propagação para fora (base → efetuador) ──────────────────────────────
    w_prev = np.zeros(3)
    wd_prev = np.zeros(3)
    a_prev = np.array([0.0, 0.0, G]) if gravity else np.zeros(3)

    F = [None] * N_JOINTS
    Nm = [None] * N_JOINTS

    for i in range(N_JOINTS):
        Rt = R[i].T
        w_par = Rt @ w_prev
        zqd = z * qd[i]
        w_i = w_par + zqd
        wd_i = Rt @ wd_prev + _cross(w_par, zqd) + z * qdd[i]
        a_i = Rt @ (a_prev + _cross(wd_prev, p[i])
                    + _cross(w_prev, _cross(w_prev, p[i])))

        m_i, c_i, I_i = links[i]
        a_c = a_i + _cross(wd_i, c_i) + _cross(w_i, _cross(w_i, c_i))
        F[i] = m_i * a_c
        Iw = I_i @ w_i
        Nm[i] = I_i @ wd_i + _cross(w_i, Iw)

        w_prev, wd_prev, a_prev = w_i, wd_i, a_i

    # ── Propagação para dentro (efetuador → base) ────────────────────────────
    f_next = np.zeros(3)
    n_next = np.zeros(3)

    if f_ext_tcp is not None:
        # Reação do ambiente sobre o robô, convertida para o frame do elo 6.
        T_tcp, frames = forward_kinematics(q)
        R_06 = frames[6][:3, :3]
        f_env = -np.asarray(f_ext_tcp[:3], float)
        n_env = -np.asarray(f_ext_tcp[3:], float)
        d_tcp = T_tcp[:3, 3] - frames[6][:3, 3]
        n_env = n_env + _cross(d_tcp, f_env)
        f_next = R_06.T @ f_env
        n_next = R_06.T @ n_env

    tau = np.zeros(N_JOINTS)
    for i in range(N_JOINTS - 1, -1, -1):
        m_i, c_i, _ = links[i]
        if i + 1 < N_JOINTS:
            Rf = R[i + 1] @ f_next
            Rn = R[i + 1] @ n_next
            p_next = p[i + 1]
        else:
            Rf = f_next
            Rn = n_next
            p_next = np.zeros(3)
        f_i = Rf + F[i]
        n_i = Nm[i] + Rn + _cross(c_i, F[i]) + _cross(p_next, Rf)
        tau[i] = n_i[2]
        f_next, n_next = f_i, n_i

    return tau


def gravity_torque(q):
    """g(q): torque necessário só para segurar o braço parado."""
    return rnea(q, np.zeros(N_JOINTS), np.zeros(N_JOINTS), gravity=True)


def coriolis_torque(q, qd):
    """C(q,qd)*qd: termos de Coriolis e centrífugos."""
    return rnea(q, qd, np.zeros(N_JOINTS), gravity=False)


def _spatial_inertia(m, c, I_com_frame):
    """
    Inércia espacial 6×6 de um corpo, referida à ORIGEM do frame do elo,
    na convenção de Featherstone v = (ω ; v_linear).

        I_sp = [[ Ī + m·c×·c×ᵀ ,  m·c× ],
                [ m·c×ᵀ        ,  m·1  ]]

    `I_com_frame` já é o tensor em torno do CoM, expresso no frame do elo.
    """
    S = _skew(c)
    Isp = np.zeros((6, 6))
    Isp[:3, :3] = I_com_frame + m * (S @ S.T)
    Isp[:3, 3:] = m * S
    Isp[3:, :3] = m * S.T
    Isp[3:, 3:] = m * np.eye(3)
    return Isp


def _spatial_transforms(R, p):
    """
    Para cada junta i devolve o par (X, Xs) com:
      X  = ^{i}X_{i-1}  (transforma MOVIMENTO do frame i-1 para o frame i)
      Xs = ^{i-1}X_i^*  (transforma FORÇA do frame i para o frame i-1)
    dados R[i] (rotação frame i → frame i-1) e p[i] (origem de i em i-1).
    """
    Xs = []
    for i in range(N_JOINTS):
        Ri = R[i]
        Pi = _skew(p[i])
        Xf = np.zeros((6, 6))          # ^{i-1}X_i^*  (força)
        Xf[:3, :3] = Ri
        Xf[:3, 3:] = Pi @ Ri
        Xf[3:, 3:] = Ri
        Xs.append(Xf)
    return Xs


def mass_matrix(q):
    """
    M(q) pelo algoritmo do corpo rígido composto (CRBA).

    Percorre a cadeia do efetuador para a base acumulando a inércia espacial
    composta de cada subcadeia; cada elemento de M sai então de uma projeção
    dessa inércia no eixo das juntas. É O(n²) e substitui as 6 chamadas ao
    RNEA da formulação ingênua — o ganho de tempo é o que torna praticável
    rodar centenas de simulações na varredura de ganhos.
    """
    R, p = _link_chain(q)
    links = _links_effective()
    Xf = _spatial_transforms(R, p)

    Ic = [_spatial_inertia(m, c, I) for (m, c, I) in links]
    S = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])   # junta rotacional em z

    M = np.zeros((N_JOINTS, N_JOINTS))
    for i in range(N_JOINTS - 1, -1, -1):
        if i > 0:
            # Ic[i-1] += ^{i-1}X_i^* · Ic[i] · ^{i}X_{i-1}
            Ic[i - 1] = Ic[i - 1] + Xf[i] @ Ic[i] @ Xf[i].T
        Fv = Ic[i] @ S
        M[i, i] = float(S @ Fv)
        j = i
        while j > 0:
            Fv = Xf[j] @ Fv
            j -= 1
            M[i, j] = float(S @ Fv)
            M[j, i] = M[i, j]
    return M + np.diag(ROTOR_INERTIA)


def inverse_dynamics(q, qd, qdd, f_ext_tcp=None):
    """
    Dinâmica inversa COMPLETA: tau = M(q)qdd + C(q,qd)qd + g(q) + Jᵀ·F_ext.

    Difere de `rnea(q,qd,qdd)` por incluir também a inércia dos rotores, que é
    diagonal e não aparece na recursão de Newton-Euler dos elos. É esta a
    função a usar para estimar torque (sensor F/T virtual, por exemplo).
    """
    return (rnea(q, qd, np.zeros(N_JOINTS), gravity=True, f_ext_tcp=f_ext_tcp)
            + ROTOR_INERTIA * np.asarray(qdd, float))


def forward_dynamics(q, qd, tau, f_ext_tcp=None, viscous=None, coulomb=None):
    """
    Dinâmica direta: dado o torque aplicado, devolve a aceleração das juntas.
    É esta função que fecha a malha nas simulações deste pacote.
    """
    M = mass_matrix(q)
    h = rnea(q, qd, np.zeros(N_JOINTS), gravity=True, f_ext_tcp=f_ext_tcp)
    tau_eff = np.asarray(tau, float) - h
    if viscous is not None:
        tau_eff = tau_eff - np.asarray(viscous, float) * qd
    if coulomb is not None:
        tau_eff = tau_eff - np.asarray(coulomb, float) * np.tanh(50.0 * qd)
    return np.linalg.solve(M, tau_eff)


# ── Utilidades de espaço operacional ──────────────────────────────────────────
def task_space_inertia(q, J=None):
    """
    Matriz de inércia no espaço operacional (Khatib):
        Lambda(q) = ( J M^-1 J^T )^-1
    Calculada com amortecimento para não explodir perto de singularidade.
    """
    if J is None:
        J = geometric_jacobian(q)
    M = mass_matrix(q)
    Minv = np.linalg.inv(M)
    A = J @ Minv @ J.T
    return np.linalg.inv(A + 1e-6 * np.eye(A.shape[0]))


def damped_pinv(A, lam=0.05):
    """Pseudo-inversa amortecida (damped least squares / Tikhonov)."""
    A = np.asarray(A, float)
    m, n = A.shape
    if m >= n:
        return np.linalg.solve(A.T @ A + lam ** 2 * np.eye(n), A.T)
    return A.T @ np.linalg.inv(A @ A.T + lam ** 2 * np.eye(m))


def manipulability(q, J=None):
    """Medida de Yoshikawa: sqrt(det(J J^T)). Zero em singularidade."""
    if J is None:
        J = geometric_jacobian(q)
    return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


# Torques máximos de referência por junta (N·m). Usados só para saturar o
# comando na simulação, do jeito que um acionamento real satura.
TAU_MAX = np.array([3000.0, 3000.0, 1500.0, 400.0, 400.0, 200.0])

# Atrito viscoso nominal (N·m·s/rad). Só atua com o braço em movimento, então
# não cria erro de regime — apenas amortece o transitório, como um redutor real.
VISCOUS_DEFAULT = np.array([50.0, 50.0, 25.0, 8.0, 5.0, 2.0])

# Atrito seco (Coulomb, N·m). O PADRÃO É ZERO de propósito: o atrito seco é uma
# perturbação NÃO MODELADA por nenhuma das três leis de controle e produz erro
# de regime permanente mesmo no torque computado (que, sendo PD, não tem ação
# integral). Deixá-lo desligado por padrão permite comparar as estratégias sem
# esse fator confundindo o resultado; o experimento de robustez do roteiro o
# liga explicitamente usando COULOMB_NOMINAL.
COULOMB_DEFAULT = np.zeros(6)
COULOMB_NOMINAL = np.array([10.0, 10.0, 5.0, 1.5, 1.0, 0.5])
