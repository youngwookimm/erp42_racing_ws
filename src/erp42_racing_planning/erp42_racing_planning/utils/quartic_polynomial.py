# erp42_racing_planning/utils/quartic_polynomial.py
from __future__ import annotations


class QuarticPolynomial:
    """
    x(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4

    boundary:
      x(0)   = x0
      x'(0)  = x_dot0
      x''(0) = x_ddot0

      x'(T)  = x_dotT
      x''(T) = x_ddotT
    """

    def __init__(
        self,
        x0: float,
        x_dot0: float,
        x_ddot0: float,
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

        c1 = self.a1 + 2.0 * self.a2 * t1
        c2 = 2.0 * self.a2

        rhs1 = float(x_dotT) - c1
        rhs2 = float(x_ddotT) - c2

        det = (3.0 * t2) * (12.0 * t2) - (4.0 * t3) * (6.0 * t1)

        if abs(det) <= 1e-12:
            raise ValueError("Singular system in QuarticPolynomial")

        self.a3 = (rhs1 * (12.0 * t2) - (4.0 * t3) * rhs2) / det
        self.a4 = ((3.0 * t2) * rhs2 - rhs1 * (6.0 * t1)) / det

    def calc_point(self, t: float) -> float:
        t = float(t)
        return (
            self.a0 +
            self.a1 * t +
            self.a2 * t**2 +
            self.a3 * t**3 +
            self.a4 * t**4
        )

    def calc_first_derivative(self, t: float) -> float:
        t = float(t)
        return (
            self.a1 +
            2*self.a2*t +
            3*self.a3*t**2 +
            4*self.a4*t**3
        )

    def calc_second_derivative(self, t: float) -> float:
        t = float(t)
        return (
            2*self.a2 +
            6*self.a3*t +
            12*self.a4*t**2
        )

    def calc_third_derivative(self, t: float) -> float:
        t = float(t)
        return 6*self.a3 + 24*self.a4*t