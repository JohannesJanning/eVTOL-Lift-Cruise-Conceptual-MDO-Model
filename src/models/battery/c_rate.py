def c_rate(P_req: float, E_battery: float) -> float:
    """Compute instantaneous discharge C-rate."""
    return P_req / E_battery
