#include "unity.h"
#include "reeds_shepp.h"

static ReedsShepp *rs;

void setUp(void)
{
    rs = new ReedsShepp(2.0f); // radio de giro = 2m
}

void tearDown(void)
{
    delete rs;
}

void test_normalize_pose_divides_xy_by_radius(void)
{
    Pose p{4.0, 6.0, 1.0};
    Pose normalized = rs->normalizePose(p);

    TEST_ASSERT_EQUAL_DOUBLE(2.0, normalized.x);
    TEST_ASSERT_EQUAL_DOUBLE(3.0, normalized.y);
    TEST_ASSERT_EQUAL_DOUBLE(1.0, normalized.theta); // theta no se toca
}

void test_denormalize_path_multiplies_lengths_by_radius(void)
{
    std::vector<RSPathSegment> path = {
        {1.5f, STRAIGHT, FORWARD},
        {0.5f, LEFT, BACKWARD},
    };

    rs->denormalizePath(path);

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.0f, path[0].length);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, path[1].length);
}

void test_get_all_paths_finds_candidates_for_forward_goal(void)
{
    // getAllPaths espera poses ya normalizadas (ver src/main.cpp), asi que
    // trabajamos directamente en unidades de radio.
    Pose start{0.0, 0.0, 0.0};
    Pose goal{4.0, 0.0, 0.0};

    auto all_paths = rs->getAllPaths(start, goal);

    TEST_ASSERT_TRUE(all_paths.size() > 0);

    int best = rs->getOptimalPathIndex(all_paths);
    TEST_ASSERT_TRUE(best >= 0);
    TEST_ASSERT_TRUE((size_t)best < all_paths.size());
    TEST_ASSERT_TRUE(all_paths[best].size() > 0);
}

void test_get_optimal_path_index_picks_shortest(void)
{
    std::vector<std::vector<RSPathSegment>> paths = {
        {{5.0f, STRAIGHT, FORWARD}},
        {{1.0f, STRAIGHT, FORWARD}},
        {{3.0f, STRAIGHT, FORWARD}},
    };

    int best = rs->getOptimalPathIndex(paths);

    TEST_ASSERT_EQUAL_INT(1, best);
}

void test_get_optimal_path_index_empty_returns_negative(void)
{
    std::vector<std::vector<RSPathSegment>> paths;

    int best = rs->getOptimalPathIndex(paths);

    TEST_ASSERT_EQUAL_INT(-1, best);
}
