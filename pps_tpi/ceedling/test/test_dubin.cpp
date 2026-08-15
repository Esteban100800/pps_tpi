#include "unity.h"
#include "dubin.h"

static DubinsPlanner *planner;

void setUp(void)
{
    planner = new DubinsPlanner(1.0f); // radio de giro = 1m
}

void tearDown(void)
{
    delete planner;
}

void test_compute_rmin_matches_formula(void)
{
    // r_min = (L / tan(delta_rad)) + W/2, con L=0.175 y W=0.18 fijos en la implementacion
    float r_min = planner->compute_rmin(30.0f); // 30 grados

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.393109f, r_min);
}

void test_compute_finds_path_for_aligned_straight_goal(void)
{
    Pose start{0.0, 0.0, 0.0};
    Pose goal{10.0, 0.0, 0.0};
    DubinsPath path;

    bool found = planner->compute(start, goal, path);

    TEST_ASSERT_TRUE(found);
    // Con inicio y fin alineados y misma orientacion, el camino optimo
    // es (casi) una linea recta: longitud total ~= distancia euclidea.
    TEST_ASSERT_FLOAT_WITHIN(0.01, 10.0, path.totalLength);
}

void test_compute_finds_path_for_perpendicular_goal(void)
{
    Pose start{0.0, 0.0, 0.0};
    Pose goal{5.0, 5.0, M_PI_2};
    DubinsPath path;

    bool found = planner->compute(start, goal, path);

    TEST_ASSERT_TRUE(found);
    TEST_ASSERT_TRUE(path.totalLength > 0.0);
    TEST_ASSERT_TRUE(path.totalLength < 1e9);
}

void test_start_and_goal_pose_round_trip(void)
{
    Pose start{1.0, 2.0, 0.5};
    Pose goal{3.0, 4.0, 1.0};

    planner->setStartPose(start);
    planner->setGoalPose(goal);

    Pose gotStart = planner->getStartPose();
    Pose gotGoal = planner->getGoalPose();

    TEST_ASSERT_EQUAL_DOUBLE(1.0, gotStart.x);
    TEST_ASSERT_EQUAL_DOUBLE(2.0, gotStart.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.5, gotStart.theta);
    TEST_ASSERT_EQUAL_DOUBLE(3.0, gotGoal.x);
    TEST_ASSERT_EQUAL_DOUBLE(4.0, gotGoal.y);
    TEST_ASSERT_EQUAL_DOUBLE(1.0, gotGoal.theta);
}

void test_segment_and_angle_state_round_trip(void)
{
    planner->setSegmentIndex(2);
    planner->setAngleStraight(1.23f);
    planner->setNewSegment(false);

    TEST_ASSERT_EQUAL_INT(2, planner->getSegmentIndex());
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.23f, planner->getAngleStraight());
    TEST_ASSERT_FALSE(planner->getNewSegment());
}

void test_new_segment_defaults_true(void)
{
    TEST_ASSERT_TRUE(planner->getNewSegment());
}
