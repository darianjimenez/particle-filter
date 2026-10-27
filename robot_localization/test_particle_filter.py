import math
import numpy as np
import pytest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent)) # needed to fix pf path

from pf import Particle, ParticleFilter
from unittest.mock import Mock

# Set up particle filter without running ROS
@pytest.fixture
def particle_filter():
    pf = ParticleFilter.__new__(ParticleFilter)

    pf.n_particles = 400
    pf.particle_cloud = []
    
    pf.odom_pose = (0, 0, 0)
    pf.current_odom_xy_theta = (0, 0, 0)

    pf.xy_noise = 0.30
    pf.theta_noise = 0.05
    pf.motion_xy_noise = 0
    pf.motion_theta_noise = 0
    pf.laser_sigma = 0.5

    # Fake transform helper
    pf.transform_helper = Mock(
        convert_pose_to_xy_and_theta=lambda pose: pose,
        angle_normalize=lambda a: math.atan2(math.sin(a), math.cos(a))
    )

    return pf

# Tests ---------------------------------------------------

# Test particles initialize around given position
def test_initialization_of_particles(particle_filter):
    np.random.seed(0)
    particle_filter.initialize_particle_cloud(None, (1, 2, 0))
    particles = particle_filter.particle_cloud
    assert len(particles) == 400 # check num of particles

    # Average position should be close to (1, 2)
    assert abs(np.mean([p.x for p in particles]) - 1) < 0.1
    assert abs(np.mean([p.y for p in particles]) - 2) < 0.1


# Test particle weights are normalized correctly
def test_normalization_of_particle_weights(particle_filter):
    particle_filter.particle_cloud = [Particle(w=2), Particle(w=3), Particle(w=5)]
    particle_filter.normalize_particles()
    weights = [p.w for p in particle_filter.particle_cloud]

    assert weights == pytest.approx([0.2, 0.3, 0.5])
    assert sum(weights) == pytest.approx(1)  # Weights add to 1

# Test odometry moves particles in correct direction
def test_odometry_moves_particles_correctly(particle_filter):
    # Particle facing 90 degrees
    particle_filter.particle_cloud = [Particle(0, 0, math.pi / 2)]

    # Move 1m forward
    particle_filter.odom_pose = (1, 0, 0)
    particle_filter.update_particles_with_odom()

    # Check new position
    particle = particle_filter.particle_cloud[0]
    assert (particle.x, particle.y, particle.theta) == pytest.approx((0, 1, math.pi / 2))


# Test laser measurements give higher weights to better positions
def test_laser_measurements_give_proper_weights(particle_filter):
    # Wall at x = 5
    class FakeMap:
        def get_closest_obstacle_distance(self, x, y):
            return abs(x - 5)

    particle_filter.occupancy_field = FakeMap()
    particle_filter.particle_cloud = [Particle(3, 0), Particle(2, 0)]

    # Laser reading
    particle_filter.update_particles_with_laser([2.0], [0.0])
    weights = [p.w for p in particle_filter.particle_cloud]

    # Check weights
    assert weights[0] > weights[1]
    assert sum(weights) == pytest.approx(1)

# Test zero weights
def test_zero_weights(particle_filter):
    particle_filter.particle_cloud = [Particle(w=0), Particle(w=0)]
    particle_filter.normalize_particles()

    # assert both particles have equal weight
    assert particle_filter.particle_cloud[0].w == pytest.approx(0.5)
    assert particle_filter.particle_cloud[1].w == pytest.approx(0.5)

# Test rotation
def test_rotation(particle_filter):
    particle_filter.particle_cloud = [Particle(0, 0, 0)]

    # Rotate 90 degrees
    particle_filter.odom_pose = (0, 0, math.pi / 2)
    particle_filter.update_particles_with_odom()
    particle = particle_filter.particle_cloud[0]

    # Position stays the same but angle changes
    assert particle.x == pytest.approx(0)
    assert particle.y == pytest.approx(0)
    assert particle.theta == pytest.approx(math.pi / 2)

# Test weighted resampling
def test_resampling(particle_filter):
    particle_filter.particle_cloud = [
        Particle(1, 1, w=1),
        Particle(5, 5, w=0)
    ]
    particle_filter.resample_particles_v2()

    assert len(particle_filter.particle_cloud) == 400 # Check particle count
    # Only select first particle
    assert all(p.x == 1 for p in particle_filter.particle_cloud)
    # Check normalized weights
    assert sum(p.w for p in particle_filter.particle_cloud) == pytest.approx(1) 
