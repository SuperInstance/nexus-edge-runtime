"""Tests for src/navigation/path.py"""
import math
import random
import pytest
from navigation.path import Pose, Waypoint, Obstacle, DeadReckoner, WaypointFollower, ObstacleAvoidance


class TestDeadReckoner:
    def test_update_straight_line(self, monkeypatch):
        monkeypatch.setattr(random, "gauss", lambda _mu, _sigma: 0.0)
        dr = DeadReckoner(Pose(0, 0, 0, 0))
        pose = dr.update(vx=1.0, vy=0.0, vz=0.0, yaw_rate=0.0, dt=2.0)
        assert pose.x == pytest.approx(2.0)
        assert pose.y == pytest.approx(0.0)

    def test_update_with_rotation(self, monkeypatch):
        monkeypatch.setattr(random, "gauss", lambda _mu, _sigma: 0.0)
        dr = DeadReckoner(Pose(0, 0, 0, 90))
        # facing +y, move forward in body x
        pose = dr.update(vx=1.0, vy=0.0, vz=0.0, yaw_rate=0.0, dt=1.0)
        assert pose.x == pytest.approx(0.0, abs=1e-6)
        assert pose.y == pytest.approx(1.0)

    def test_yaw_rate_update(self, monkeypatch):
        monkeypatch.setattr(random, "gauss", lambda _mu, _sigma: 0.0)
        dr = DeadReckoner(Pose(0, 0, 0, 0))
        pose = dr.update(vx=0.0, vy=0.0, vz=0.0, yaw_rate=math.radians(90), dt=1.0)
        assert pose.yaw == pytest.approx(90.0)

    def test_yaw_normalization_negative(self, monkeypatch):
        monkeypatch.setattr(random, "gauss", lambda _mu, _sigma: 0.0)
        dr = DeadReckoner(Pose(0, 0, 0, -10))
        pose = dr.update(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0, dt=1.0)
        assert pose.yaw == pytest.approx(-10.0)

    def test_gps_correct(self):
        dr = DeadReckoner(Pose(1, 1, 0, 0))
        error = dr.correct(4, 5)
        assert error == pytest.approx(5.0)
        assert dr.pose.x == 4
        assert dr.pose.y == 5

    def test_distance_to(self):
        dr = DeadReckoner(Pose(0, 0, 0, 0))
        target = Pose(3, 4, 0, 0)
        assert dr.distance_to(target) == pytest.approx(5.0)


class TestWaypointFollower:
    def test_set_waypoints_total_distance(self):
        wf = WaypointFollower()
        wf.set_waypoints([
            Waypoint(0, 0),
            Waypoint(10, 0),
            Waypoint(10, 10),
        ])
        assert wf.total_distance == pytest.approx(20.0)
        assert wf.get_target().name == ""

    def test_get_desired_heading(self):
        wf = WaypointFollower()
        wf.set_waypoints([Waypoint(10, 10)])
        heading = wf.get_desired_heading(Pose(0, 0, 0, 0))
        assert heading == pytest.approx(45.0)

    def test_check_arrival_true(self):
        wf = WaypointFollower()
        wf.set_waypoints([Waypoint(10, 0, tolerance=2.0)])
        assert wf.check_arrival(Pose(10, 0, 0, 0)) is True
        assert wf.check_arrival(Pose(11.5, 0, 0, 0)) is True

    def test_check_arrival_false(self):
        wf = WaypointFollower()
        wf.set_waypoints([Waypoint(10, 0, tolerance=1.0)])
        assert wf.check_arrival(Pose(0, 0, 0, 0)) is False

    def test_get_desired_speed_far(self):
        wf = WaypointFollower()
        wf.set_waypoints([Waypoint(100, 0, speed_limit=2.0)])
        speed = wf.get_desired_speed(Pose(0, 0, 0, 0))
        assert speed == pytest.approx(2.0)

    def test_get_desired_speed_near(self):
        wf = WaypointFollower()
        wf.set_waypoints([Waypoint(0, 0, tolerance=2.0, speed_limit=2.0)])
        # distance = 3, tolerance*3 = 6 -> scale = 3/6 = 0.5
        speed = wf.get_desired_speed(Pose(3, 0, 0, 0))
        assert speed == pytest.approx(1.0)

    def test_update_progress(self):
        wf = WaypointFollower()
        wf.set_waypoints([Waypoint(0, 0), Waypoint(10, 0)])
        wf.update(Pose(0, 0, 0, 0))
        result = wf.update(Pose(5, 0, 0, 0))
        assert result["progress"] == 50.0
        assert result["arrived"] is False

    def test_advance(self):
        wf = WaypointFollower()
        wf.set_waypoints([Waypoint(0, 0), Waypoint(10, 0)])
        wf.advance()
        assert wf.current_idx == 1
        assert wf.completed == [0]
        assert wf.get_target().x == 10

    def test_arrival_and_advance(self):
        wf = WaypointFollower()
        wf.set_waypoints([Waypoint(0, 0, tolerance=1.0), Waypoint(10, 0)])
        result = wf.update(Pose(0, 0, 0, 0))
        assert result["arrived"] is True
        wf.advance()
        assert wf.current_idx == 1


class TestObstacleAvoidance:
    def test_attraction_only(self):
        oa = ObstacleAvoidance(safety_radius=5.0, repulsion_gain=10.0)
        pose = Pose(0, 0, 0, 0)
        target = Waypoint(10, 0)
        fx, fy = oa.compute_avoidance(pose, target)
        assert fx == pytest.approx(1.0)
        assert fy == pytest.approx(0.0)

    def test_repulsion_changes_direction(self):
        oa = ObstacleAvoidance(safety_radius=5.0, repulsion_gain=10.0)
        # obstacle below the path should push upward
        oa.add_obstacle(Obstacle(5, -2, 0.5, 1.0))
        pose = Pose(0, 0, 0, 0)
        target = Waypoint(10, 0)
        fx, fy = oa.compute_avoidance(pose, target)
        # net vector should still point forward but deflect away from obstacle
        assert fx > 0
        assert fy > 0

    def test_at_target_zero(self):
        oa = ObstacleAvoidance()
        pose = Pose(5, 5, 0, 0)
        target = Waypoint(5, 5)
        fx, fy = oa.compute_avoidance(pose, target)
        assert fx == pytest.approx(0.0)
        assert fy == pytest.approx(0.0)
