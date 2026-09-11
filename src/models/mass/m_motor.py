from src.models.smooth.smoothing import soft_floor


def motor_mass(P_hover, P_climb, n_vert, n_hor):
    hp_hover = soft_floor(P_hover / (n_vert * 745.7) * 1.5, 1e-6)
    hp_climb = soft_floor(P_climb / (n_hor * 745.7) * 1.5, 1e-6)

    m_motor_vert = n_vert * 0.6756 * hp_hover ** 0.783
    m_motor_hor = n_hor * 0.6756 * hp_climb ** 0.783

    return m_motor_vert + m_motor_hor
