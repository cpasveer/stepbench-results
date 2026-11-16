"""Rough beam-frame model of the connector in A-local coordinates (x=along face A, y=world Z, z=toward B)."""
import numpy as np, math

cB = np.array([19.4932, -23.5, 68.0])          # B face centre in A-local (computed from task.json)
tB = np.array([-0.94551, 0.0, 0.32557]); nB = np.array([-0.32557, 0.0, -0.94551])
S3 = math.sqrt(3) / 2

def beam_k(p1, p2, A, Iy, Iz, J, E, nu=0.35):
    G = E / (2 * (1 + nu))
    L = np.linalg.norm(p2 - p1); ex = (p2 - p1) / L
    ref = np.array([0.0, 0.0, 1.0]) if abs(ex[2]) < 0.9 else np.array([1.0, 0, 0])
    ey = np.cross(ref, ex); ey /= np.linalg.norm(ey); ez = np.cross(ex, ey)
    k = np.zeros((12, 12))
    a = E * A / L; t = G * J / L
    b1 = 12 * E * Iz / L**3; b2 = 6 * E * Iz / L**2; b3 = 4 * E * Iz / L; b4 = 2 * E * Iz / L
    c1 = 12 * E * Iy / L**3; c2 = 6 * E * Iy / L**2; c3 = 4 * E * Iy / L; c4 = 2 * E * Iy / L
    kl = np.zeros((12, 12))
    kl[0, 0] = kl[6, 6] = a; kl[0, 6] = kl[6, 0] = -a
    kl[3, 3] = kl[9, 9] = t; kl[3, 9] = kl[9, 3] = -t
    kl[1, 1] = kl[7, 7] = b1; kl[1, 7] = kl[7, 1] = -b1
    kl[1, 5] = kl[5, 1] = b2; kl[1, 11] = kl[11, 1] = b2
    kl[5, 7] = kl[7, 5] = -b2; kl[7, 11] = kl[11, 7] = -b2
    kl[5, 5] = kl[11, 11] = b3; kl[5, 11] = kl[11, 5] = b4
    kl[2, 2] = kl[8, 8] = c1; kl[2, 8] = kl[8, 2] = -c1
    kl[2, 4] = kl[4, 2] = -c2; kl[2, 10] = kl[10, 2] = -c2
    kl[4, 8] = kl[8, 4] = c2; kl[8, 10] = kl[10, 8] = c2
    kl[4, 4] = kl[10, 10] = c3; kl[4, 10] = kl[10, 4] = c4
    T = np.zeros((12, 12))
    R = np.vstack([ex, ey, ez])
    for i in range(4):
        T[3*i:3*i+3, 3*i:3*i+3] = R
    return T.T @ kl @ T

class Model:
    def __init__(self):
        self.nodes = []; self.elems = []
    def node(self, p):
        self.nodes.append(np.array(p, float)); return len(self.nodes) - 1
    def beam(self, i, j, A, Iy, Iz, J, E):
        self.elems.append((i, j, A, Iy, Iz, J, E))
    def solve(self, fixed_nodes, load_node, F6):
        n = len(self.nodes); K = np.zeros((6*n, 6*n))
        for i, j, A, Iy, Iz, J, E in self.elems:
            ke = beam_k(self.nodes[i], self.nodes[j], A, Iy, Iz, J, E)
            dof = list(range(6*i, 6*i+6)) + list(range(6*j, 6*j+6))
            K[np.ix_(dof, dof)] += ke
        fixed = []
        for f in fixed_nodes: fixed += list(range(6*f, 6*f+6))
        free = [d for d in range(6*n) if d not in fixed]
        Fv = np.zeros(6*n); Fv[6*load_node:6*load_node+6] = F6
        u = np.zeros(6*n)
        u[free] = np.linalg.solve(K[np.ix_(free, free)] + 1e-12*np.eye(len(free)), Fv[free])
        return u

def rigid_link(m, i, j):
    # very stiff beam between two nodes (penalty rigid link)
    m.beam(i, j, 1e6, 1e6, 1e6, 1e6, 1e3)
