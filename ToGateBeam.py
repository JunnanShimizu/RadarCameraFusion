import numpy as np

class ToGateBeam:

    # Used to convert from (x, y) to (r, theta) to (gate, beam)
    def to_gate_beam(x, y, range_resolution, theta_resolution):
        # range_resolution = 120
        # theta_resolution = .05

        r = (x**2) + (y**2)
        theta = np.arctan2(y, x)

        gate = r / range_resolution
        beam = theta / theta_resolution

        return gate, beam