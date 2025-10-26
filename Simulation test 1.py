import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

# ---------------------------------------------------------------
# Hjelpefunksjoner
# ---------------------------------------------------------------
def wrap_angle(a):
    """Pakk vinkel til [-pi, pi]."""
    return (a + np.pi) % (2*np.pi) - np.pi

def cumulative_arclength(x, y):
    ds = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate(([0.0], np.cumsum(ds)))
    return s

# ---------------------------------------------------------------
# Bicycle-modell (kinematisk) + RK4
# ---------------------------------------------------------------
class BicycleModel:
    def __init__(self, L=2.7, dt=0.02, x0=0.0, y0=0.0, theta0=0.0):
        self.L = L
        self.dt = dt
        self.state = np.array([x0, y0, theta0], dtype=float)  # [x, y, theta]

    def f(self, state, delta, v):
        x, y, th = state
        dx = v * np.cos(th)
        dy = v * np.sin(th)
        dth = v / self.L * np.tan(delta)
        return np.array([dx, dy, dth], dtype=float)

    def step_rk4(self, delta, v):
        s = self.state
        dt = self.dt
        k1 = self.f(s, delta, v)
        k2 = self.f(s + 0.5*dt*k1, delta, v)
        k3 = self.f(s + 0.5*dt*k2, delta, v)
        k4 = self.f(s + dt*k3,      delta, v)
        self.state = s + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        self.state[2] = wrap_angle(self.state[2])

# ---------------------------------------------------------------
# Stanley-kontroller
# ---------------------------------------------------------------
class StanleyController:
    def __init__(self, k=1.0, max_steer=np.deg2rad(35)):
        self.k = k
        self.max_steer = max_steer

    def control(self, x, y, th, path_xy, path_yaw, v):
        """
        x, y, th: kjøretøyets posisjon og heading
        path_xy: (N,2) punkter tett langs banen
        path_yaw: (N,) kurs (tangent) til banen i hvert punkt
        v: hastighet
        """
        # Finn nærmeste punkt på banen
        d2 = (path_xy[:,0] - x)**2 + (path_xy[:,1] - y)**2
        i = int(np.argmin(d2))

        # Kursfeil
        psi_ref = path_yaw[i]
        psi_err = wrap_angle(psi_ref - th)

        # Tverrfeil (signert) via kryssprodukt mellom banetangent og p->kjt vektor
        vx = np.cos(psi_ref)
        vy = np.sin(psi_ref)
        dx = x - path_xy[i,0]
        dy = y - path_xy[i,1]
        e_ct = vx * dy - vy * dx  # z-komp av kryssprodukt i 2D

        # Stanley-lov
        delta = psi_err + np.arctan2(self.k * e_ct, v + 1e-6)
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        return float(delta), i, e_ct, psi_err

# ---------------------------------------------------------------
# Lag en jevn bane fra veipunkter med kubisk spline
# ---------------------------------------------------------------
def make_spline_path(waypoints, ds=0.1):
    """
    waypoints: (M,2) grove veipunkter
    ds: steg langs arklengde for utprøver-punkter
    return: path_xy (N,2), path_yaw (N,)
    """
    xw = waypoints[:,0]
    yw = waypoints[:,1]

    # Fjern eventuelle duplikater etter hverandre (for spline-stabilitet)
    mask = np.r_[True, np.hypot(np.diff(xw), np.diff(yw)) > 1e-9]
    xw = xw[mask]; yw = yw[mask]

    s = cumulative_arclength(xw, yw)
    cx = CubicSpline(s, xw, bc_type='natural')
    cy = CubicSpline(s, yw, bc_type='natural')

    s_dense = np.arange(0.0, s[-1], ds)
    xd = cx(s_dense)
    yd = cy(s_dense)

    # Deriver for yaw (tangent-retning)
    dx = cx(s_dense, 1)
    dy = cy(s_dense, 1)
    yaw = np.arctan2(dy, dx)

    path_xy = np.column_stack([xd, yd])
    return path_xy, yaw

# ---------------------------------------------------------------
# Eksempel: rektangel 20 x 10 m
# ---------------------------------------------------------------
if __name__ == "__main__":
    # Definer grov rektangelbane (start nederst til venstre, mot høyre, CCW)
    W, H = 20.0, 10.0
    wp = np.array([
        [0, 0], [W, 0], [W, H], [0, H], [0, 0]
    ], dtype=float)

    # Spline-bane
    path_xy, path_yaw = make_spline_path(wp, ds=0.1)

    # Init modell og kontroller
    model = BicycleModel(L=2.7, dt=0.02, x0=-2.0, y0=-1.0, theta0=np.deg2rad(15))
    ctrl  = StanleyController(k=1.2, max_steer=np.deg2rad(35))

    # Konstanthastighet (du kan legge inn profil hvis ønskelig)
    v = 4.0  # m/s

    # Simulering
    T = 60.0
    steps = int(T / model.dt)

    xs, ys, ths = [], [], []
    deltas, e_ct_hist, psi_err_hist = [], [], []
    closest_idx_hist = []

    for _ in range(steps):
        x, y, th = model.state
        delta, i, e_ct, psi_err = ctrl.control(x, y, th, path_xy, path_yaw, v)
        model.step_rk4(delta, v)

        xs.append(model.state[0]); ys.append(model.state[1]); ths.append(model.state[2])
        deltas.append(delta); e_ct_hist.append(e_ct); psi_err_hist.append(psi_err); closest_idx_hist.append(i)

    xs = np.array(xs); ys = np.array(ys)

    # Plot bane og kjørt spor
    plt.figure(figsize=(8,6))
    plt.plot(path_xy[:,0], path_xy[:,1], linewidth=2, label="Referansebane (spline)")
    plt.plot(xs, ys, linestyle='--', label="Kjørt spor")
    plt.axis('equal'); plt.grid(True); plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.title("Bicycle + Stanley: Banefølging")
    plt.legend()
    plt.show()

    # Plot feil og styrevinkel
    t = np.arange(len(e_ct_hist)) * model.dt

    plt.figure(figsize=(8,4))
    plt.plot(t, e_ct_hist)
    plt.grid(True); plt.xlabel("tid [s]"); plt.ylabel("tverrfeil e_ct [m]")
    plt.title("Tverrfeil")
    plt.show()

    plt.figure(figsize=(8,4))
    plt.plot(t, np.rad2deg(psi_err_hist))
    plt.grid(True); plt.xlabel("tid [s]"); plt.ylabel("kursfeil ψ_err [deg]")
    plt.title("Kursfeil")
    plt.show()

    plt.figure(figsize=(8,4))
    plt.plot(t, np.rad2deg(deltas))
    plt.grid(True); plt.xlabel("tid [s]"); plt.ylabel("styrevinkel δ [deg]")
    plt.title("Styresignal (Stanley)")
    plt.show()
