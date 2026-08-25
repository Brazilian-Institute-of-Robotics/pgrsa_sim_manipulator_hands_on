import numpy as np


class FreeSpace:

    name = "free"

    def wrench(self, x, xd):
        return np.zeros(6)

    def contact_info(self, x, xd):
        return dict(in_contact=False, penetration=0.0, f_normal=0.0)


class PlanarSurface:

    name = "plane"

    def __init__(
        self,
        origin=(1.30, 0.0, 0.60),
        normal=(0.0, 0.0, 1.0),
        stiffness=5.0e4,
        damping=3.0e2,
        friction=0.2,
        max_force=1.0e4,
    ):
        self.origin = np.asarray(origin, float)
        n = np.asarray(normal, float)
        self.normal = n / np.linalg.norm(n)
        self.K = float(stiffness)
        self.B = float(damping)
        self.mu = float(friction)
        self.max_force = float(max_force)

    def _penetration(self, x, xd):
        d = float(np.dot(np.asarray(x, float) - self.origin, self.normal))
        dd = float(np.dot(np.asarray(xd, float), self.normal))
        return -d, -dd  # penetração positiva = dentro do material

    def wrench(self, x, xd):
        pen, pen_rate = self._penetration(x, xd)
        if pen <= 0.0:
            return np.zeros(6)

        f_n = self.K * pen + self.B * pen_rate
        f_n = min(max(f_n, 0.0), self.max_force)  # a superfície só empurra

        F = f_n * self.normal

        # Atrito direção tangencial ao movimento.
        xd = np.asarray(xd, float)
        v_t = xd - np.dot(xd, self.normal) * self.normal
        speed = np.linalg.norm(v_t)
        if speed > 1e-6 and self.mu > 0.0:
            F = F - self.mu * f_n * (v_t / speed)

        return np.array([F[0], F[1], F[2], 0.0, 0.0, 0.0])

    def contact_info(self, x, xd):
        pen, _ = self._penetration(x, xd)
        w = self.wrench(x, xd)
        return dict(
            in_contact=pen > 0.0,
            penetration=max(pen, 0.0),
            f_normal=float(np.dot(w[:3], self.normal)),
        )


class CompliantWall(PlanarSurface):
    """Parede vertical (normal em −X): o robô a encontra avançando em +X."""

    name = "wall"

    def __init__(self, x_wall=1.45, **kw):
        kw.setdefault("normal", (-1.0, 0.0, 0.0))
        super().__init__(origin=(x_wall, 0.0, 0.0), **kw)


class MovingSurface(PlanarSurface):
    """
    Superfície que se desloca sozinha ao longo da própria normal, com um perfil
    senoidal. Serve para testar a REJEIÇÃO DE PERTURBAÇÃO da malha de força:
    o controlador precisa manter a força constante enquanto o chão sobe e
    desce sob a ferramenta — a situação de uma peça empenada numa esteira.
    """

    name = "moving"

    def __init__(self, amplitude=0.010, freq=0.25, **kw):
        super().__init__(**kw)
        self.amplitude = float(amplitude)
        self.freq = float(freq)
        self.t = 0.0
        self._origin0 = self.origin.copy()

    def step(self, t):
        self.t = float(t)
        off = self.amplitude * np.sin(2 * np.pi * self.freq * self.t)
        self.origin = self._origin0 + off * self.normal


PRESETS = {
    "espuma": dict(stiffness=1.0e3, damping=40.0, friction=0.30),
    "borracha": dict(stiffness=1.0e4, damping=120.0, friction=0.40),
    "madeira": dict(stiffness=5.0e4, damping=300.0, friction=0.25),
    "aluminio": dict(stiffness=1.0e6, damping=2.0e3, friction=0.15),
    "aco": dict(stiffness=5.0e6, damping=5.0e3, friction=0.12),
}


def build(kind="plane", material="madeira", **kw):
    """Fábrica: `build('wall', 'aco', x_wall=1.4)`."""
    params = dict(PRESETS.get(material, PRESETS["madeira"]))
    params.update(kw)
    if kind == "free":
        return FreeSpace()
    if kind == "wall":
        return CompliantWall(**params)
    if kind == "moving":
        return MovingSurface(**params)
    return PlanarSurface(**params)
