import argparse
import json
from simulator import Map, Environment, UAV
from simulator.maps import maps
from simulator.utils.shared_imports import Pos,State, TMState, np, PrettyTable, pytest
from simulator.path_planner.path_planner import AStarPlanner,remove_same_timestep_oscillations, ObliviousPlanner
from simulator.scenario.scenario import Scenario
from simulator.tester.tester import run_tests
import simulator.utils.config as cfg
#--------------------------------Fixtures--------------------------------------------------
@pytest.fixture
def empty_env():
    # An empty 10×10×10 world for testing
    world = np.zeros((10, 10, 10))
    return Environment(world_data=world, output_mode=0)

@pytest.fixture
def planner():
    return AStarPlanner()

#------------------------------------Tests--------------------------------------------------------------

#--------------------------Environment--------------------------------------------------
def test_environment_init(empty_env):
    """
    Test the initialization of the Environment class.
    """
    env = empty_env
    assert env.world_data.shape == (10, 10, 10)
    assert env.output_mode == 0
    assert env.uav_list == []
    assert env.reservations == []
    assert env.candidate_paths == {}
    assert env.timestep == 0
    assert env.uav_list == []
    assert env.reservations == []


def test_set_reservations(empty_env):
    """
    Test the assign_reservations method.
    """
    env = empty_env
    env.set_reservations([State(1, 2, 3,0), State(4, 5, 6,1)])
    assert len(env.reservations) == 2
    assert State(1, 2, 3,0) in env.reservations
    assert not State(4, 5, 6,0) in env.reservations
    assert not State(1, 2, 3,1) in env.reservations
    assert State(4, 5, 6,1)in env.reservations
    env.set_reservations([])
    assert len(env.reservations) == 0

def test_environment_completion(empty_env):
    env = empty_env
    env.next_timestep()
    #as no uavs are registered, the timestep should not change
    assert env.timestep == 0
    uav1 = UAV(0, destinations=[Pos(1, 2, 3), Pos(2, 2, 3)])
    env.register_uav(uav1)
    uav1.assign_path([State(1, 2, 3, 0), State(2, 2, 3, 1)])
    env.next_timestep()
    #timestep should now be 1
    assert env.timestep == 1
    assert uav1.is_finished() == True
    env.reset_environment()
    assert uav1.is_finished() == False
    env.uav_list = []
    assert env.timestep == 0

def test_environment_reset(empty_env):
    """
    Test the reset_environment method.
    """
    env = empty_env
    env.set_reservations([State(1, 2, 3,0), State(4, 5, 6,1)])
    env.reset_environment()
    assert len(env.reservations) == 2
    assert env.timestep == 0
    assert len(env.uav_list) == 0

def test_environment_collision(empty_env):
    """
    Test the detect_collisions method.
    """
    env = empty_env
    env.set_reservations([])
    uav1 = UAV(0, destinations=[Pos(1, 2, 3), Pos(2, 2, 3)])
    uav2 = UAV(1, destinations=[Pos(1, 2, 3), Pos(2, 2, 3)])
    env.register_uav(uav1)
    env.register_uav(uav2)
    uav1.assign_path([State(1, 2, 3, 0), State(1, 1, 3, 1)])
    uav2.assign_path([State(1, 2, 3, 0), State(2, 2, 3, 1)])
    env.reset_environment()
    collisions = env.detect_collisions()
    env.map_uavs()
    env.detect_collisions()
    assert len(collisions) == 1
    env.next_timestep()
    collisions = env.detect_collisions()
    #as uavs occupied the final space of the prev timestep still 1 collision 
    assert len(collisions) == 1
    env.next_timestep()
    assert env.timestep == 2
    
def test_variable_speed_planning(empty_env):
    """
    A UAV with max_speed=2 should traverse up to 2 cells in one timestep.
    We place it at (0,0,0) with goal at (2,0,0).  Distance=2.
    With speed=2, it should plan to reach in a single timestep.
    """
    u = UAV(
        uav_type=0,
        destinations=[Pos(0,0,0), Pos(2,0,0)],
        max_speed=2,
        inaccuracy=[0,0]
    )
    empty_env.register_uav(u)
    planner = AStarPlanner()
    candidate_paths, _, _ = planner.plan_path(empty_env)
    path = candidate_paths[u.id]
    # we expect two states: start at t=0, goal at t=1
    assert len(path) >= 2
    times = [s.time for s in path]
    # must finish at time 1 since can move two cells in one step
    assert any(s.x == 2 and s.time == 1 for s in path)

def test_inflated_footprint_persistence(empty_env):
    """
    After a UAV finishes its final move at time t, its footprint
    should still appear in the environment at time t+1 exactly once.
    """
    # define a 3×3×1 world
    env = empty_env
    u = UAV(0, destinations=[Pos(0,0,0), Pos(1,0,0)], max_speed=1, inaccuracy=[1,0])
    env.register_uav(u)
    # assign a trivial path: move at t=1
    u.assign_path([State(0,0,0,0), State(1,0,0,1)])
    # run two timesteps
    env.map_uavs(); env.detect_collisions()
    env.next_timestep()
    env.next_timestep()
    # at t=2 (one after finish), footprint of (1,0,0) inflated by radius=1
    env.map_uavs()
    voxels = set(env.uav_map.keys())
    # inflated footprint around (1,0,0) includes (0,0,0),(1,0,0),(2,0,0),(1,1,0),(1,-1,0) clipped
    assert (1,0,0) in voxels
    assert (0,0,0) in voxels
    assert (2,0,0) in voxels

def test_multi_goal_planning(empty_env):
    """
    A UAV with three sequential waypoints should have a path that visits them in order.
    """
    u = UAV(0,
            destinations=[Pos(0,0,0), Pos(1,1,0), Pos(2,2,0)],
            max_speed=1,
            inaccuracy=[0,0])
    empty_env.register_uav(u)
    planner = AStarPlanner()
    candidate_paths, _, _ = planner.plan_path(empty_env)
    path = candidate_paths[u.id]
    # extract the sequence of visited unique positions (ignoring repeated waits)
    visited = [ (s.x, s.y) for s in path if (s.x, s.y) not in [(p.x,p.y) for p in path[:path.index(s)]] ]
    # should include all three in order
    assert (0,0) == visited[0]
    assert (1,1) in visited
    assert (2,2) in visited
    assert visited.index((1,1)) < visited.index((2,2))

def test_dynamic_spawn_times(empty_env):
    """
    Two UAVs, one starting at t=0 and one at t=3, should have their
    first move appear only once their start_time is reached.
    """
    u1 = UAV(0, destinations=[Pos(0,0,0), Pos(1,0,0)], start_time=0, max_speed=1)
    u2 = UAV(1, destinations=[Pos(2,0,0), Pos(3,0,0)], start_time=3, max_speed=1)
    empty_env.register_uav(u1)
    empty_env.register_uav(u2)
    planner = AStarPlanner()
    candidate_paths, _, _ = planner.plan_path(empty_env)
    empty_env.candidate_paths = {u1.id: candidate_paths[u1.id],
                                 u2.id: candidate_paths[u2.id]}
    empty_env.set_active_candidate_path(u1.id)
    # steps 0–2: u2 should not appear in uav_map
    for t in range(1, 3):
        empty_env.next_timestep()
        empty_env.map_uavs()
        assert all(uav_id != u2.id for _, uav_ids in empty_env.uav_map.items() for uav_id in uav_ids)
    # at t=3, u2 starts
    empty_env.next_timestep()
    empty_env.map_uavs()
    assert any(uav_id == u2.id for _, uav_ids in empty_env.uav_map.items() for uav_id in uav_ids)


def test_environment_run(empty_env):
    env = empty_env
    uav1 = UAV(0, destinations=[Pos(1, 2, 3), Pos(2, 2, 3)])
    uav2 = UAV(1, destinations=[Pos(3, 2, 3), Pos(2, 2, 3)])
    uav3 = UAV(2, destinations=[Pos(0, 0, 0), Pos(0, 0, 4)],max_speed=4)
    uav4 = UAV(3, destinations=[Pos(1, 0, 0), Pos(1, 0, 1)],start_time=1)

    env.register_uav(uav1)
    env.register_uav(uav2)
    env.register_uav(uav3)
    env.register_uav(uav4)

    uav1.assign_path([State(1, 2, 3, 0), State(2, 2, 3, 1)])
    uav2.assign_path([State(3, 2, 3, 0), State(2, 2, 3, 1)])
    uav3.assign_path([State(0, 0, 0, 0),State(0,0,1,1),State(0,0,2,1),State(0,0,3,1), State(0, 0, 4, 1)])
    uav4.assign_path([State(1, 0, 0, 2), State(1,0,1,3)])
    env.set_reservations([State(0, 0, 0,0),State(0, 0, 0,1)])
    
    results = env.run()
    #Completes in 3 timesteps as uav4 starts at t=1,
    #  waits a turn (t=2) then completes at t=3
    assert results["timesteps"] == 3
    #Uavs move a total of 4 times
    assert results["movements"] == 5
    #predicted 3 collisions, 1 at time 1 between uav3 and reservation
    #and 2 at time 1 between uav1 and uav2 / uav3 and reservation again
    assert results["collisions"] == (0,3)
    #all uavs complete their paths
    assert results["success"] == True

    
#------------------------------------UAV--------------------------------------------------------------
def test_default_destinations():
    u = UAV(0)
    assert isinstance(u.destinations, list)
    assert u.destinations[0] == Pos(0,0,0)
    assert u.destinations[1] == Pos(0,0,0)

def test_reset_uav():
    u = UAV(0, destinations=[Pos(1,1,1), Pos(2,2,2)], start_time=3)
    u.units_moved = 5
    u.finished = True
    u.time_finished = 10
    u.current_position = Pos(9,9,9)
    u.times_waited = 2
    u.previous_positions = [Pos(0,0,0)]
    u.planned_route = [State(2,2,2,3)]
    u.traversed_positions = [Pos(1,1,1)]
    u.reset_uav()
    assert u.units_moved == 0
    assert not u.finished
    assert u.time_finished == -1
    assert u.current_position == u.destinations[0]
    assert u.times_waited == 0
    assert u.previous_positions == []

def test_move_and_is_finished():
    # ensure uav only stays at spawn in first timestep and following that moves to the next destination
    u = UAV(0, destinations=[Pos(0,0,0), Pos(1,0,0), Pos(2,0,0)], max_speed=1)
    u.planned_route = [State(0,0,0,0),State(0,0,0,1), State(1,0,0,2), State(2,0,0,3)]
    # first timestep
    assert u.current_position == Pos(0,0,0)
    assert not u.finished
    u.move(0)
    assert u.current_position == Pos(0,0,0)
    assert not u.finished
    u.move(1)
    assert u.current_position == Pos(0,0,0)
    assert not u.finished
    # second timestep
    u.move(2)
    assert u.current_position == Pos(1,0,0)
    assert not u.finished
    # second timestep
    u.move(3)
    assert u.current_position == Pos(2,0,0)
    assert u.finished
    assert u.time_finished == 3
    assert u.times_waited == 1
    assert u.units_moved == 3
    assert u.previous_positions == [Pos(0,0,0),Pos(0,0,0),Pos(1,0,0)]
    # is_finished now True
    assert u.is_finished()
    u2 = UAV(0, destinations=[Pos(0,0,0), Pos(1,0,0), Pos(6,0,0)], max_speed=6)
    u2.planned_route = [
        State(0,0,0,0), State(1,0,0,1),State(2,0,0,1),
        State(3,0,0,1),State(4,0,0,1),State(5,0,0,1),State(6,0,0,1)
    ]
    # first timestep
    u2.move(0)
    assert u2.current_position == Pos(0,0,0)
    assert not u2.finished
    # second timestep
    u2.move(1)
    assert u2.current_position == Pos(6,0,0)
    assert u2.finished
    assert u2.time_finished == 1


def test_assign_path_sets_route_and_position():
    u = UAV(0)
    new_route = [State(5,5,5,0), State(6,6,6,1)]
    u.assign_path(new_route)
    assert u.planned_route == new_route
    assert u.current_position == Pos(5,5,5)
    assert u.units_moved == 0

def test_uav_class():
    """
    Test the UAV class.
    """
    uav = UAV(
        uav_type=0,
        destinations=[Pos(1, 2, 3), Pos(4, 5, 6)],
        inaccuracy=[0, 0],
        start_time=0,
        max_speed=1,
        name="Test UAV"
    )
    assert uav.uav_type == 0
    assert uav.destinations == [Pos(1, 2, 3), Pos(4, 5, 6)]
    assert uav.inaccuracy == [0, 0]
    assert uav.start_time == 0
    assert uav.max_speed == 1
    assert uav.name == "Test UAV"
    test_default_destinations()
    test_reset_uav()
    test_move_and_is_finished()
    test_assign_path_sets_route_and_position()

#------------------------------------PLANNER--------------------------------------------------------------
def test_remove_same_timestep_oscillations():
    # A → B → A all at time 0 should collapse B
    path = [
        State(0, 0, 0, 0),
        State(1, 0, 0, 0),
        State(0, 0, 0, 0),
        State(0, 1, 0, 1)
    ]
    cleaned = remove_same_timestep_oscillations(path)
    assert cleaned == [
        State(0, 0, 0, 0),
        State(0, 1, 0, 1)
    ]
    
def test_remove_same_timestep_oscillations_no_change():
    # A → B all at time 0 should not change
    path = [
        State(0, 0, 0, 0),
        State(1, 0, 0, 0),
        State(0, 1, 0, 1)
    ]
    cleaned = remove_same_timestep_oscillations(path)
    assert cleaned == path
    
def test_compute_circular_footprint(planner):
    # Inaccuracy radius=1, circular shape=0
    u = UAV(0, inaccuracy=[1, 0])  # radius 1, circular
    pts = planner.compute_footprint(u, Pos(2, 2, 1))
    pts_set = set((p.x, p.y, p.z) for p in pts)
    # Should include center and its 6 axial neighbors and 12 diagonal within sphere
    assert (2, 2, 1) in pts_set
    # Check a diagonal interior point at distance sqrt(2)
    assert (3, 2, 1) in pts_set
    # Check a point just outside radius
    assert (4, 2, 1) not in pts_set
    
    assert (3,3,1) not in pts_set


def test_compute_square_footprint(planner):
    # Inaccuracy radius=1, square shape=1
    u = UAV(0, inaccuracy=[1, 1])  # radius 1, square
    pts = planner.compute_footprint(u, Pos(0, 0, 0))
    pts_set = set((p.x, p.y, p.z) for p in pts)
    # All 3×3×3 voxels around origin
    expected = {
        (x, y, z)
        for x in [-1, 0, 1]
        for y in [-1, 0, 1]
        for z in [-1, 0, 1]
        if 0 <= x and 0 <= y and 0 <= z  # clipped by grid bounds
    }
    assert expected.issubset(pts_set)


def test_add_reservation_and_vertex_conflict(planner):
    # Build a simple path: two states, reservation footprints should cover both timesteps
    u = UAV(0, inaccuracy=[0, 0])
    path = [
        State(1, 1, 1, 0),
        State(2, 1, 1, 1)
    ]
    res = {}
    res = planner.add_reservation(res, u, path)
    # Should have both (1,1,1,0) and (2,1,1,1) keys
    assert (1, 1, 1, 0) in res
    assert (2, 1, 1, 1) in res
    #1 from t = 0, 2 from t = 1, 1 from t = 2
    assert len(res) == 4
    # Each reserved by UAV -1 (as UAV not assigned to environment)
    assert res[(1,1,1,0)] == [-1]
    assert res[(2,1,1,1)] == [-1]
    assert res[(2,1,1,2)] == [-1]


def test_order_uavs_by_start_time_and_id(planner):
    # Create three UAVs with same start time but different ids
    u1 = UAV(0, start_time=5)
    u2 = UAV(1, start_time=5)
    u3 = UAV(2, start_time=3)
    u1.id, u2.id, u3.id = 0, 1, 2
    ordered = planner.order_uavs([u1, u2, u3])
    # u3 has earliest start_time, then u1 (id 0), then u2 (id 1)
    assert [u.id for u in ordered] == [2, 0, 1]


def test_sequential_planning_avoids_vertex_conflict(empty_env, planner):
    # Two UAVs on the same 1D corridor from (0,0,0) to (2,0,0)
    u1 = UAV(0, destinations=[Pos(0,0,0), Pos(2,0,0)], max_speed=1)
    u2 = UAV(1, destinations=[Pos(0,0,0), Pos(2,0,0)], max_speed=1)
    empty_env.register_uav(u1)
    empty_env.register_uav(u2)

    # Plan both paths
    paths, delay_counts, _ = planner.plan_path(empty_env)

    path1 = {(s.x, s.y, s.z, s.time) for s in paths[u1.id]}
    path2 = {(s.x, s.y, s.z, s.time) for s in paths[u2.id]}
    for s in paths[u1.id]:
        print(s)
    for s in paths[u2.id]:
        print(s)

    # No overlapping space–time cells
    assert path1.isdisjoint(path2)

    # At least one UAV must be delayed to avoid conflict
    assert delay_counts[u1.id] + delay_counts[u2.id] == 2
    
def test_delay_spawn_if_no_neighbours(empty_env, planner):
    # Two UAVs on the same 1D corridor from (0,0,0) to (2,0,0)
    u1 = UAV(0, destinations=[Pos(0,0,0), Pos(2,0,0)], max_speed=2)
    u2 = UAV(1, destinations=[Pos(0,0,0), Pos(2,0,0)], max_speed=1)
    empty_env.register_uav(u1)
    empty_env.register_uav(u2)

    # Plan both paths
    paths, delay_counts, _ = planner.plan_path(empty_env)

    path1 = {(s.x, s.y, s.z, s.time) for s in paths[u1.id]}
    path2 = {(s.x, s.y, s.z, s.time) for s in paths[u2.id]}
    for s in paths[u1.id]:
        print(s)
    for s in paths[u2.id]:
        print(s)

    # No overlapping space–time cells
    assert path1.isdisjoint(path2)

    # At least one UAV must be delayed to avoid conflict
    assert delay_counts[u1.id] + delay_counts[u2.id] == 2
    
def test_immediately_following_only(empty_env, planner):
    # Two UAVs on the same 1D corridor from (0,0,0) to (2,0,0)
    #Default ordering is by id, so u1 should be delayed
    u1 = UAV(0, destinations=[Pos(0,0,0), Pos(3,0,0)], max_speed=3)
    u2 = UAV(1, destinations=[Pos(0,0,0), Pos(3,0,0)], max_speed=1)
    empty_env.register_uav(u1)
    empty_env.register_uav(u2)

    # Plan both paths
    paths, delay_counts, _ = planner.plan_path(empty_env)

    path1 = {(s.x, s.y, s.z, s.time) for s in paths[u1.id]}
    path2 = {(s.x, s.y, s.z, s.time) for s in paths[u2.id]}
    for s in paths[u1.id]:
        print(s)
    for s in paths[u2.id]:
        print(s)

    # No overlapping space–time cells
    assert path1.isdisjoint(path2)

    # At least one UAV must be delayed to avoid conflict
    assert delay_counts[u1.id] + delay_counts[u2.id] == 2
