# erp42_racing_planning/utils/quintic_polynomial.py
from __future__ import annotations
import numpy as np


class QuinticPolynomial:
    """
    x(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5

    boundary:
      x(0)   = x0
      x'(0)  = x_dot0
      x''(0) = x_ddot0

      x(T)   = xT
      x'(T)  = x_dotT
      x''(T) = x_ddotT
    """

    def __init__(
        self,
        x0: float,
        x_dot0: float,
        x_ddot0: float,
        xT: float,
        x_dotT: float,
        x_ddotT: float,
        T: float,
    ):
        if T <= 1e-6:
            raise ValueError("T must be positive")

        self.T = float(T)

        self.a0 = float(x0)
        self.a1 = float(x_dot0)
        self.a2 = float(x_ddot0) / 2.0

        t1 = self.T
        t2 = t1 * t1
        t3 = t2 * t1
        t4 = t3 * t1
        t5 = t4 * t1

        c0 = self.a0 + self.a1 * t1 + self.a2 * t2
        c1 = self.a1 + 2.0 * self.a2 * t1
        c2 = 2.0 * self.a2

        A = np.array([
            [t3,    t4,    t5],
            [3*t2,  4*t3,  5*t4],
            [6*t1, 12*t2, 20*t3],
        ], dtype=float)

        b = np.array([
            float(xT) - c0,
            float(x_dotT) - c1,
            float(x_ddotT) - c2,
        ], dtype=float)

        x = np.linalg.solve(A, b)

        self.a3 = float(x[0])
        self.a4 = float(x[1])
        self.a5 = float(x[2])

    def calc_point(self, t: float) -> float:
        t = float(t)
        return (
            self.a0 +
            self.a1*t +
            self.a2*t**2 +
            self.a3*t**3 +
            self.a4*t**4 +
            self.a5*t**5
        )

    def calc_first_derivative(self, t: float) -> float:
        t = float(t)
        return (
            self.a1 +
            2*self.a2*t +
            3*self.a3*t**2 +
            4*self.a4*t**3 +
            5*self.a5*t**4
        )

    def calc_second_derivative(self, t: float) -> float:
        t = float(t)
        return (
            2*self.a2 +
            6*self.a3*t +
            12*self.a4*t**2 +
            20*self.a5*t**3
        )

    def calc_third_derivative(self, t: float) -> float:
        t = float(t)
        return (
            6*self.a3 +
            24*self.a4*t +
            60*self.a5*t**2
        )