"""
Created on Mon Oct  6 13:49:04 2025

@author: selma
"""

import numpy as np
import matplotlib.pyplot as plt


# --- Vehicle Model ---

class BicycleModel:

    def __init__(self, L=2.7, dt=0.02, x0=0, y0=0, theta0=0):
        self.L = L
        self.dt = dt
        self.state = np.array([x0, y0, theta0]) # [x, y, theta]

    def derivatives(self, state, delta, v): 
        x, y, th = state
        dx = v * np.cos(th)
        dy = v * np.sin(th)
        dth = v / self.L * np.tan(delta)
        return np.array([dx, dy, dth])

    def rk4_step(self, delta, v):
        s = self.state
        f = self.derivatives
        dt = self.dt
        
        k1 = f(s, delta, v)
        k2 = f(s + 0.5*dt*k1, delta, v)
        k3 = f(s + 0.5*dt*k2, delta, v)
        k4 = f(s + dt*k3, delta, v)
        
        self.state = s + dt*(k1+2*k2+2*k3+k4)/6
        self.state[2] = (self.state[2] + np.pi) % (2*np.pi) - np.pi
        return self.state


# --- Stanley Controller ---

class StanleyController:
    def __init__(self, k=0.8):
        self.k = k

    def control(self, state, path, v):
        x, y, th = state
        
        # Finn nærmeste punkt
        dists = np.sum((path - np.array([x,y]))**2, axis=1)
        i = np.argmin(dists)
        target = path[min(i+1, len(path)-1)]
        path_dir = np.arctan2(target[1]-path[i,1], target[0]-path[i,0])
        
        # Cross-track error (sign via kryssprodukt)
        dx, dy = x-path[i,0], y-path[i,1]
        v_path = np.array([np.cos(path_dir), np.sin(path_dir)])
        e = np.cross(np.r_[v_path,0], np.r_[dx,dy,0])[2]
        psi_err = (path_dir - th + np.pi)%(2*np.pi)-np.pi
        delta = psi_err + np.arctan2(self.k*e, v)
        return delta, e, psi_err

# --- Demo ---

if __name__ == "__main__":

    # Lag veipunkter (rektangel)
    waypoints = np.array([[0,0],[5,0],[10,0],[15,0],[20,0],[20,2.5],[20,5],[20,7.5],[20,10],
                          [15,10],[10,10],[5,10],[0,10],[0,7.5],[0,5],[0,2.5],[0,0]])
    model = BicycleModel()
    ctrl = StanleyController(k=0.8) 

    v = 1.0
    log = {"x":[],"y":[],"cte":[],"psi":[],"delta":[]}
    
    for _ in range(2000):
        delta, e, psi = ctrl.control(model.state, waypoints, v)
        x,y,th = model.rk4_step(delta, v)

        log["x"].append(x); log["y"].append(y)
        log["cte"].append(e); log["psi"].append(psi); log["delta"].append(delta)


# Plott

plt.figure(); plt.plot(waypoints[:,0], waypoints[:,1],'ro--',label="Path")
plt.plot(log["x"], log["y"],'b',label="Vehicle"); plt.legend(); plt.axis("equal")

plt.figure(); plt.plot(log["cte"]); plt.title("Cross-Track Error")
plt.figure(); plt.plot(log["psi"]); plt.title("Heading Error")
plt.figure(); plt.plot(np.rad2deg(log["delta"])); plt.title("Steering [deg]")
plt.show()