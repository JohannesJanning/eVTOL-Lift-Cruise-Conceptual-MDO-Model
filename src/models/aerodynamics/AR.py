def AR_calculation(b, c):
    """
    Calculate the aspect ratio (AR) of a wing.

    Parameters:
        b (float): Wing span [m]
        c (float): Wing chord length [m]

    Returns:
        float: Wing aspect ratio (AR), defined as span divided by chord [-]
    """
    return b / c


# Backwards-compatible alias
def aspect_ratio(b, c):
    return AR_calculation(b, c)
