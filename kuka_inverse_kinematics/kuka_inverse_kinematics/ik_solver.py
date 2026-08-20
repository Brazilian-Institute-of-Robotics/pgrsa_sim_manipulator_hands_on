"""
ik_solver.py — Cinemática Inversa do KUKA KR70 R2100

FK baseada diretamente nas transformações do URDF (sem DH intermediário).
Jacobiano numérico para robustez.

Métodos:
  - Levenberg-Marquardt (recomendado)
  - Jacobiano pseudo-inverso
  - Jacobiano transposto
"""

import numpy as np
import sys, os

try:
    from kuka_forward_kinematics.kinematics import (
        forward_kinematics, geometric_jacobian,
        rotation_to_euler_zyx, JOINT_LIMITS, JOINT_NAMES
    )
except ImportError:
    # Fallback inline — replica a FK URDF direta
    def _rot_x(a):
        c,s=np.cos(a),np.sin(a)
        return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]],dtype=float)
    def _rot_y(a):
        c,s=np.cos(a),np.sin(a)
        return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]],dtype=float)
    def _rot_z(a):
        c,s=np.cos(a),np.sin(a)
        return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]],dtype=float)
    def _trans(x,y,z):
        T=np.eye(4); T[:3,3]=[x,y,z]; return T
    def _rpy(r,p,y): return _rot_z(y)@_rot_y(p)@_rot_x(r)

    _JOINTS=[
        {'xyz':[0.,0.,0.236],   'rpy':[np.pi,0.,0.]},
        {'xyz':[0.175,0.054,-0.339],'rpy':[np.pi/2,0.,0.]},
        {'xyz':[0.89,0.,-0.0975],'rpy':[0.,0.,0.]},
        {'xyz':[0.30805,-0.05,0.1515],'rpy':[np.pi/2,0.,-np.pi/2]},
        {'xyz':[0.,0.04795,-0.72695],'rpy':[0.,np.pi/2,np.pi/2]},
        {'xyz':[0.149,0.,-0.04795],'rpy':[np.pi/2,0.,-np.pi/2]},
    ]
    _T_FLANGE = _trans(0,0,-0.036)@_rpy(0,np.pi,0)
    _T_STATIC = [_trans(*j['xyz'])@_rpy(*j['rpy']) for j in _JOINTS]

    def forward_kinematics(q):
        T=np.eye(4); frames=[np.eye(4)]
        for i in range(6):
            T=T@_T_STATIC[i]@_rot_z(q[i]); frames.append(T.copy())
        T_tcp=T@_T_FLANGE; frames.append(T_tcp.copy())
        return T_tcp, frames

    def geometric_jacobian(q, frames=None, eps=1e-6):
        T0,_=forward_kinematics(q); p0=T0[:3,3]; R0=T0[:3,:3]
        J=np.zeros((6,6))
        for i in range(6):
            qp=q.copy(); qp[i]+=eps; Tp,_=forward_kinematics(qp)
            J[:3,i]=(Tp[:3,3]-p0)/eps
            dR=(Tp[:3,:3]-R0)/eps; S=dR@R0.T
            J[3,i]=S[2,1]; J[4,i]=S[0,2]; J[5,i]=S[1,0]
        return J

    JOINT_LIMITS=np.array([[-3.2289,3.2289],[-3.0543,1.0472],[-2.0944,2.8798],
                            [-3.1416,3.1416],[-2.1817,2.1817],[-6.1087,6.1087]])
    JOINT_NAMES=['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6']

    def rotation_to_euler_zyx(R):
        sy=np.sqrt(R[0,0]**2+R[1,0]**2)
        if sy>1e-6:
            roll=np.arctan2(R[2,1],R[2,2]); pitch=np.arctan2(-R[2,0],sy); yaw=np.arctan2(R[1,0],R[0,0])
        else:
            roll=np.arctan2(-R[1,2],R[1,1]); pitch=np.arctan2(-R[2,0],sy); yaw=0.
        return np.degrees(np.array([yaw,pitch,roll]))


def _rot_err(R_des, R_cur):
    R_e = R_des @ R_cur.T
    angle = np.arccos(np.clip((np.trace(R_e)-1)/2, -1, 1))
    if abs(angle) < 1e-8: return np.zeros(3)
    ax = np.array([R_e[2,1]-R_e[1,2], R_e[0,2]-R_e[2,0], R_e[1,0]-R_e[0,1]]) / (2*np.sin(angle))
    return ax * angle


def _clamp(q):
    return np.clip(q, JOINT_LIMITS[:,0], JOINT_LIMITS[:,1])


# ── Levenberg-Marquardt (método principal) ─────────────────────────────────────
def ik_levenberg_marquardt(p_des, R_des, q_init=None, max_iter=300,
                            tol_pos=1e-4, tol_ori=1e-3, lam=0.01):
    q = (q_init.copy() if q_init is not None else np.zeros(6))
    hist = {'q': [q.copy()], 'error_pos': [], 'error_ori': [], 'method': 'Levenberg-Marquardt'}

    for it in range(max_iter):
        T, _ = forward_kinematics(q)
        ep = p_des - T[:3,3]
        eo = _rot_err(R_des, T[:3,:3])
        e  = np.concatenate([ep, eo])
        ep_n, eo_n = np.linalg.norm(ep), np.linalg.norm(eo)
        hist['error_pos'].append(ep_n)
        hist['error_ori'].append(eo_n)
        if ep_n < tol_pos and eo_n < tol_ori:
            hist['converged'] = True; hist['iterations'] = it+1; break
        J = geometric_jacobian(q)
        dq = np.linalg.solve(J.T@J + lam*np.eye(6), J.T@e)
        dq = np.clip(dq, -0.15, 0.15)
        q = _clamp(q + dq)
        hist['q'].append(q.copy())
    else:
        hist['converged'] = False; hist['iterations'] = max_iter

    hist['q_result'] = q
    hist['final_error_pos'] = hist['error_pos'][-1] if hist['error_pos'] else -1.
    hist['final_error_ori'] = hist['error_ori'][-1] if hist['error_ori'] else -1.
    return hist


# ── Pseudo-Inverso ─────────────────────────────────────────────────────────────
def ik_jacobian_pinv(p_des, R_des, q_init=None, max_iter=400,
                     tol_pos=1e-4, tol_ori=1e-3, alpha=0.5):
    q = (q_init.copy() if q_init is not None else np.zeros(6))
    hist = {'q': [q.copy()], 'error_pos': [], 'error_ori': [], 'method': 'Pseudo-Inverso'}

    for it in range(max_iter):
        T, _ = forward_kinematics(q)
        ep = p_des - T[:3,3]
        eo = _rot_err(R_des, T[:3,:3])
        e  = np.concatenate([ep, eo])
        ep_n, eo_n = np.linalg.norm(ep), np.linalg.norm(eo)
        hist['error_pos'].append(ep_n)
        hist['error_ori'].append(eo_n)
        if ep_n < tol_pos and eo_n < tol_ori:
            hist['converged'] = True; hist['iterations'] = it+1; break
        J = geometric_jacobian(q)
        dq = alpha * np.linalg.pinv(J) @ e
        dq = np.clip(dq, -0.15, 0.15)
        q = _clamp(q + dq)
        hist['q'].append(q.copy())
    else:
        hist['converged'] = False; hist['iterations'] = max_iter

    hist['q_result'] = q
    hist['final_error_pos'] = hist['error_pos'][-1] if hist['error_pos'] else -1.
    hist['final_error_ori'] = hist['error_ori'][-1] if hist['error_ori'] else -1.
    return hist


# ── Transposto ─────────────────────────────────────────────────────────────────
def ik_jacobian_transpose(p_des, R_des, q_init=None, max_iter=600,
                           tol_pos=1e-4, tol_ori=1e-3, alpha=0.02):
    q = (q_init.copy() if q_init is not None else np.zeros(6))
    hist = {'q': [q.copy()], 'error_pos': [], 'error_ori': [], 'method': 'Transposto'}

    for it in range(max_iter):
        T, _ = forward_kinematics(q)
        ep = p_des - T[:3,3]
        eo = _rot_err(R_des, T[:3,:3])
        e  = np.concatenate([ep, eo])
        ep_n, eo_n = np.linalg.norm(ep), np.linalg.norm(eo)
        hist['error_pos'].append(ep_n)
        hist['error_ori'].append(eo_n)
        if ep_n < tol_pos and eo_n < tol_ori:
            hist['converged'] = True; hist['iterations'] = it+1; break
        J = geometric_jacobian(q)
        dq = alpha * J.T @ e
        dq = np.clip(dq, -0.15, 0.15)
        q = _clamp(q + dq)
        hist['q'].append(q.copy())
    else:
        hist['converged'] = False; hist['iterations'] = max_iter

    hist['q_result'] = q
    hist['final_error_pos'] = hist['error_pos'][-1] if hist['error_pos'] else -1.
    hist['final_error_ori'] = hist['error_ori'][-1] if hist['error_ori'] else -1.
    return hist


# ── Interface unificada com multi-restart ──────────────────────────────────────
def solve_ik(p_des, R_des=None, q_init=None, method='lm',
             n_restarts=5, **kwargs):
    """
    Interface unificada para IK com múltiplos restarts aleatórios.

    Args:
        p_des      : posição desejada do TCP (3,)
        R_des      : orientação desejada 3×3. None = identidade
        q_init     : config inicial (6,). None = zeros
        method     : 'lm' | 'pinv' | 'transpose'
        n_restarts : número de tentativas com configs aleatórias

    Returns:
        dict com q_result, converged, iterations, error histories
    """
    if R_des is None:  R_des = np.eye(3)
    if q_init is None: q_init = np.zeros(6)

    solvers = {'lm': ik_levenberg_marquardt,
               'pinv': ik_jacobian_pinv,
               'transpose': ik_jacobian_transpose}
    solver = solvers.get(method, ik_levenberg_marquardt)

    rng = np.random.default_rng(0)
    best = None

    for trial in range(n_restarts):
        q0 = q_init.copy() if trial == 0 else rng.uniform(JOINT_LIMITS[:,0], JOINT_LIMITS[:,1])
        result = solver(p_des, R_des, q0, **kwargs)
        result['p_des'] = p_des
        result['R_des'] = R_des
        if best is None or result['final_error_pos'] < best['final_error_pos']:
            best = result
        if best['converged']:
            break

    return best
