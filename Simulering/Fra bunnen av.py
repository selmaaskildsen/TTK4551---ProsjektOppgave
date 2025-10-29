import numpy as np
import matplotlib.pyplot as plt

# --- Bicycle Model ---
def derivatives(state, delta, v, L):
    x, y, th = state
    dx = v * np.cos(th)
    dy = v * np.sin(th)
    dth = v / L * np.tan(delta)
    return np.array([dx, dy, dth])

# -- RK4 ---
def rk4_step(state, delta, v, L, dt):
    f = derivatives
    k1 = f(state, delta, v, L)
    k2 = f(state + 0.5 * dt * k1, delta, v, L)
    k3 = f(state + 0.5 * dt * k2, delta, v, L)
    k4 = f(state + dt * k3, delta, v, L)

    new_state = state + dt * (k1 + 2*k2 + 2*k3 + k4) / 6
    new_state[2] = (new_state[2] + np.pi) % (2*np.pi) - np.pi
    return new_state


# --- Stanley Controller ---
def stanley_control(state, path, v, k):
    x, y, th = state

    # Finn nærmeste punkt på banen
    dists = np.sum((path - np.array([x, y]))**2, axis=1)
    i = np.argmin(dists)
    target = path[min(i + 1, len(path) - 1)]
    path_dir = np.arctan2(target[1] - path[i, 1], target[0] - path[i, 0])

    # Cross-track error (sign fra kryssprodukt)
    dx, dy = x - path[i, 0], y - path[i, 1]
    v_path = np.array([np.cos(path_dir), np.sin(path_dir)])
    e = -np.cross(np.r_[v_path, 0], np.r_[dx, dy, 0])[2]

    psi_err = (path_dir - th + np.pi) % (2*np.pi) - np.pi
    delta = psi_err + np.arctan2(k * e, v)
    return delta, e, psi_err


# --- Hovedprogram ---

if __name__ == "__main__":
    # Definer veipunkter (rektangel)
    waypoints = np.array([
        [0, 0], [20, 0], [20, 10], [0, 10], [0, 0]
    ])

    # Parametre
    L = 2.7
    dt = 0.007
    v = 1.0
    k = 1.2
    state = np.array([0, 0, 0])  # [x, y, theta]

    log = {"x": [], "y": [], "cte": [], "psi": [], "delta": []}

    # --- Bestem målpunkt ---
    goal_idx = len(waypoints) - 1
    if np.allclose(waypoints[0], waypoints[-1]):  # Hvis start og slutt er like
        goal_idx = len(waypoints) - 2
    goal = waypoints[goal_idx]
    goal_tol = 0.3
    min_steps_before_stop = 50  # vent litt før du får stoppe

    # --- Simulasjonsløkke ---
    for step in range(9000):
        delta, e, psi = stanley_control(state, waypoints, v, k)
        state = rk4_step(state, delta, v, L, dt)
        x, y, th = state

        log["x"].append(x)
        log["y"].append(y)
        log["cte"].append(e)
        log["psi"].append(psi)
        log["delta"].append(delta)

        # Sjekk om vi har nådd målet
        if step > min_steps_before_stop and np.linalg.norm(state[:2] - goal) < goal_tol:
            print("Mål nådd!")
            break

    print(f"Antall steg kjørt: {len(log['x'])}")

    # --- Plotting ---
    plt.figure()
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro--', label="Path")
    if len(log["x"]) >= 2:
        plt.plot(log["x"], log["y"], 'b', label="Vehicle")
    else:
        plt.plot(log["x"], log["y"], 'bo', label="Vehicle")

    # Marker start og stopp
    """
    plt.plot(log["x"][0], log["y"][0], 'go', label="Start")
    plt.plot(log["x"][-1], log["y"][-1], 'kx', label="Stop")
    """

    plt.legend()
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Bicycle Model med Stanley Controller (funksjonsbasert)")
    plt.grid(True)
    plt.show()
