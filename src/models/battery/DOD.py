def depth_of_discharge(energy_trip: float, E_battery: float) -> float:
    """Compute depth of discharge."""
    return energy_trip / E_battery
