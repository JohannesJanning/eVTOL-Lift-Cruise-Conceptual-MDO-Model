def drag_calculation(rho, V, c, b, C_D):
    """Calculate aerodynamic drag force.

    Matches `src/models/aerodynamics/drag.py` signature and formula.
    """
    S = c * b
    return 0.5 * rho * V ** 2 * S * C_D
