import jax.numpy as jnp
from src.models.smooth.smoothing import soft_floor

def power_required_hover(sigma_hover, T_req_total_hover, rho, eta_h):
    """Calculate total power required during hover."""
    # ensure disk loading is non-negative to avoid sqrt of negative

    sigma_safe = soft_floor(sigma_hover, 1e-8, k=80.0)
    # induced velocity from momentum theory (safe)
    # compute directly from physics; avoid artificial hard caps
    v_i = jnp.sqrt(sigma_safe / (2.0 * jnp.maximum(rho, 1e-8)))
    eta_safe = jnp.maximum(eta_h, 1e-6)
    return T_req_total_hover * v_i / eta_safe
