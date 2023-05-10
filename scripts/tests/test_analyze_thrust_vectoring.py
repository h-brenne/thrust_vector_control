import numpy as np
from scripts.analyze_thrust_vectoring import calc_elevation_azimuth_angles


def test_calc_elevation_azimuth_angles():
    # Check that the angles are calculated correctly for a few force vectors
    # NED coordinate system is used, such that [0,0,-1] corresponds to a nominal rotor thrust vector
    force_vectors = np.array(
        [[1, 0, 0], [0, 1, 0], [0, 0, -1], [-1, 0, 0], [0, -1, 0], [1, 1, -1]]
    )

    elevation, azimuth = calc_elevation_azimuth_angles(force_vectors)

    assert np.allclose(elevation, [90, 90, 0, 90, 90, 54.7356], rtol=1e-3)
    assert np.allclose(azimuth, [0, 90, 0, 180, -90, 45], rtol=1e-3)


if __name__ == "__main__":
    test_calc_elevation_azimuth_angles()
