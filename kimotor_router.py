# Copyright 2022-2024 Stefano Cottafavi <stefano.cottafavi@gmail.com>.
# SPDX-License-Identifier: GPL-2.0-only

""" Router for the coil-to-coil connections, placed inside the coils inner diameter

Each connection joins two or more coil terminals (all at the coils inner radius).
A route is made of radial stubs (one per terminal) and one arc concentric to the
motor axis. The router tries arc radius (level), layers and via placements until
it finds a route that keeps the clearance against everything already placed.
"""

import math
import time
import numpy as np

# arcs are checked as polylines: a 0.25mm chord deviates from the arc by < 2um
# for any radius above 4mm
ARC_STEP = 250000
CELL = 1000000

class Router:

    def __init__(self, clearance, track_w, via_d, layers, tol=2000):
        """
        Args:
            clearance (int): min copper clearance [nm]
            track_w (int): track width [nm]
            via_d (int): via diameter [nm]
            layers (list): copper layers that can be used for the arcs
            tol (int): tolerance on the clearance check [nm]
        """
        self.clr = clearance
        self.w = track_w
        self.via_d = via_d
        self.layers = layers
        self.tol = tol
        self.grid = {}

    # geometry primitives: dict(layer (None: all layers), pts (Nx2 array), w, ends, bb)

    def _prims(self, layer, pts, w, chunk=6):
        """ Split a polyline in short chunks (better bounding boxes) """
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        ends = (pts[0], pts[-1])
        out = []
        i = 0
        while True:
            c = pts[i:i+chunk+1]
            h = w/2 + self.clr
            out.append(dict(layer=layer, pts=c, w=w, ends=ends,
                bb=(c[:,0].min()-h, c[:,1].min()-h, c[:,0].max()+h, c[:,1].max()+h)))
            i += chunk
            if i >= len(pts)-1:
                break
        return out

    def _cells(self, bb):
        for i in range(int(bb[0]//CELL), int(bb[2]//CELL)+1):
            for j in range(int(bb[1]//CELL), int(bb[3]//CELL)+1):
                yield (i, j)

    def _add(self, prims):
        for p in prims:
            for c in self._cells(p['bb']):
                self.grid.setdefault(c, []).append(p)

    def _remove(self, prims):
        ids = set(id(p) for p in prims)
        for p in prims:
            for c in self._cells(p['bb']):
                self.grid[c] = [q for q in self.grid[c] if id(q) not in ids]

    def obstacle_track(self, layer, pts, w):
        self._add(self._prims(layer, pts, w))

    def obstacle_via(self, p, d):
        self._add(self._prims(None, [p], d))

    @staticmethod
    def arc_points(r, a0, a1, step=ARC_STEP):
        n = max(2, int(abs(a1-a0)*r/step) + 2)
        a = np.linspace(a0, a1, n)
        return np.column_stack([r*np.cos(a), r*np.sin(a)])

    @staticmethod
    def arc_points_3(s, m, e, step=ARC_STEP):
        """ Sample the arc through start, mid and end points """
        (x1, y1), (x2, y2), (x3, y3) = s, m, e
        d = 2*(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))
        if abs(d) < 1e-9:
            return np.array([s, e], dtype=float)
        ux = ((x1*x1+y1*y1)*(y2-y3) + (x2*x2+y2*y2)*(y3-y1) + (x3*x3+y3*y3)*(y1-y2)) / d
        uy = ((x1*x1+y1*y1)*(x3-x2) + (x2*x2+y2*y2)*(x1-x3) + (x3*x3+y3*y3)*(x2-x1)) / d
        r = math.hypot(x1-ux, y1-uy)
        a0 = math.atan2(y1-uy, x1-ux)
        am = (math.atan2(y2-uy, x2-ux) - a0) % (2*math.pi)
        a1 = (math.atan2(y3-uy, x3-ux) - a0) % (2*math.pi)
        sweep = a1 if am < a1 else a1 - 2*math.pi
        pts = Router.arc_points(r, a0, a0+sweep, step)
        return pts + np.array([ux, uy])

    # clearance check

    @staticmethod
    def _pt_seg(X, U, V):
        D = V - U
        L = (D*D).sum(1)
        L[L == 0] = 1
        W = X[:,None,:] - U[None,:,:]
        t = np.clip((W*D[None]).sum(2)/L[None], 0, 1)
        C = U[None] + t[...,None]*D[None]
        return np.sqrt(((X[:,None,:]-C)**2).sum(2)).min()

    def _dist(self, A, B):
        if len(A) == 1: A = np.vstack([A, A])
        if len(B) == 1: B = np.vstack([B, B])
        a0, a1, b0, b1 = A[:-1], A[1:], B[:-1], B[1:]
        d = min(self._pt_seg(a0, b0, b1), self._pt_seg(a1, b0, b1),
                self._pt_seg(b0, a0, a1), self._pt_seg(b1, a0, a1))
        def cr(o, u, v):
            return (u[...,0]-o[...,0])*(v[...,1]-o[...,1]) - (u[...,1]-o[...,1])*(v[...,0]-o[...,0])
        A0, A1, B0, B1 = a0[:,None], a1[:,None], b0[None], b1[None]
        if ((cr(B0,B1,A0)*cr(B0,B1,A1) < 0) & (cr(A0,A1,B0)*cr(A0,A1,B1) < 0)).any():
            return 0
        return d

    def fits(self, cands):
        for c in cands:
            seen = set()
            for cell in self._cells(c['bb']):
                for p in self.grid.get(cell, ()):
                    if id(p) in seen:
                        continue
                    seen.add(id(p))
                    if c['layer'] is not None and p['layer'] is not None and c['layer'] != p['layer']:
                        continue
                    a, b = c['bb'], p['bb']
                    if a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1]:
                        continue
                    # intended contact (track ending on a coil terminal, or on a via)
                    if any(abs(e[0]-f[0]) < 1000 and abs(e[1]-f[1]) < 1000 for e in c['ends'] for f in p['ends']):
                        continue
                    gap = self._dist(c['pts'], p['pts']) - c['w']/2 - p['w']/2
                    if gap < self.clr - self.tol:
                        return False
        return True

    # routing

    def _track(self, prims, draw, layer, a, b):
        prims += self._prims(layer, [a, b], self.w)
        draw.append(('track', layer, a, b))

    def _arc(self, prims, draw, layer, r, a0, a1):
        prims += self._prims(layer, self.arc_points(r, a0, a1), self.w)
        am = (a0 + a1)/2
        draw.append(('arc', layer,
            (r*math.cos(a0), r*math.sin(a0)),
            (r*math.cos(am), r*math.sin(am)),
            (r*math.cos(a1), r*math.sin(a1))))

    def _via(self, prims, draw, p):
        prims += self._prims(None, [p], self.via_d)
        draw.append(('via', p))

    def _candidate(self, terms, rho, layer, modes, r_jog, jog, span):
        """ Build route geometry for one combination of parameters

        Each terminal goes radially down to the arc radius rho. Terminals on a
        layer other than the arc's need a via: they first go down to r_jog and
        move sideways by the jog angle, away from the adjacent terminal of the
        next coil, so that the via does not crowd it. The via is placed at the
        jog radius ('top') or at the arc ('bottom').

        Returns:
            (list, list): geometry primitives (for checking), drawing primitives
        """
        P = lambda r, a: (r*math.cos(a), r*math.sin(a))
        prims, draw = [], []
        for (p, t_layer, jog_dir), mode in zip(terms, modes):
            th = math.atan2(p[1], p[0])
            if t_layer == layer:
                self._track(prims, draw, layer, p, P(rho, th))
                continue
            th_j = th + jog_dir*jog
            j0, j1, q = P(r_jog, th), P(r_jog, th_j), P(rho, th_j)
            self._track(prims, draw, t_layer, p, j0)
            if jog:
                self._arc(prims, draw, t_layer, r_jog, min(th, th_j), max(th, th_j))
            if mode == 'bottom':
                self._track(prims, draw, t_layer, j1, q)
                self._via(prims, draw, q)
            else:
                self._via(prims, draw, j1)
                self._track(prims, draw, layer, j1, q)

        a0, a1 = span
        self._arc(prims, draw, layer, rho, a0, a1)
        return prims, draw

    @staticmethod
    def spans(ths):
        """ Candidate angular spans [a0, a1] (a0 < a1) covering all the given angles, shortest first """
        ths = sorted(ths)
        n = len(ths)
        # skip one of the gaps between consecutive angles: the arc covers the rest
        gaps = []
        for i in range(n):
            b = ths[(i+1) % n] + (2*math.pi if i == n-1 else 0)
            gaps.append((b - ths[i], i))
        out = []
        for g, i in sorted(gaps, reverse=True):
            a0 = ths[(i+1) % n]
            a1 = ths[i] + (2*math.pi if i < n-1 else 0)
            out.append((a0, a1))
        return out[:2]

    def options(self, terms, levels, r_jog, jog):
        """ Yield all candidate routes, best first: shortest arc, inner layers (the
        terminal tracks are on the outer layers, so they never cross arcs on the
        inner ones), fewest vias, outermost level """
        outer = (self.layers[0], self.layers[-1]) if len(self.layers) > 2 else ()
        cands = []
        r_via_band = self.via_d/2 + self.clr + self.w/2
        for layer in self.layers:
            n_other = sum(1 for _, tl, _ in terms if tl != layer)
            ths = [math.atan2(p[1], p[0]) + (d*jog if tl != layer else 0) for p, tl, d in terms]
            for i_span, span in enumerate(self.spans(ths)):
                for i_rho, rho in enumerate(levels):
                    # keep the band of the jog vias free
                    if abs(rho - r_jog) < r_via_band or (n_other and rho > r_jog):
                        continue
                    for m in range(2**n_other):
                        cands.append(((i_span, layer in outer, n_other, i_rho, m), layer, span, rho, m))
        cands.sort(key=lambda c: c[0])

        for _, layer, span, rho, m in cands:
            modes, bit = [], 0
            for _, tl, _ in terms:
                if tl == layer:
                    modes.append(None)
                else:
                    modes.append('top' if (m >> bit) & 1 else 'bottom')
                    bit += 1
            yield self._candidate(terms, rho, layer, modes, r_jog, jog, span)

    def route_all(self, conns, levels, r_jog, jog, ref=0, time_limit=30):
        """ Route all the connections, with backtracking

        Args:
            conns (list): each connection is a list of (point, layer, jog direction) terminals
            levels (list): candidate arc radii, from the outermost
            r_jog (float): radius where the terminal tracks can jog sideways and change layer
            jog (float): jog angle
            ref (float): reference angle, where the phase chains start
            time_limit (float): give up after this time [s]

        Returns:
            list: drawing primitives of each connection (None if routing failed)
        """
        def span(c):
            a0, a1 = self.spans([math.atan2(p[1], p[0]) for p, _, _ in c])[0]
            return (a0 - ref) % (2*math.pi), a1 - a0

        # which connection gets the outer levels matters: try a few orderings
        idx = range(len(conns))
        orders = [
            sorted(idx, key=lambda i: span(conns[i])[1]),
            sorted(idx, key=lambda i: span(conns[i])[0], reverse=True),
            sorted(idx, key=lambda i: span(conns[i])[0]),
            sorted(idx, key=lambda i: span(conns[i])[1], reverse=True),
        ]

        self.steps = 0
        for n, order in enumerate(orders):
            t_end = time.time() + time_limit/len(orders)
            result = [None]*len(conns)
            placed = []

            def dfs(k):
                if k == len(order):
                    return True
                i = order[k]
                for prims, draw in self.options(conns[i], levels, r_jog, jog):
                    self.steps += 1
                    if time.time() > t_end:
                        return False
                    if self.fits(prims):
                        self._add(prims)
                        placed.append(prims)
                        result[i] = draw
                        if dfs(k+1):
                            return True
                        self._remove(placed.pop())
                        result[i] = None
                return False

            if dfs(0):
                return result
            for prims in placed:
                self._remove(prims)

        return None
