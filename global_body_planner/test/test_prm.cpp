#include <gtest/gtest.h>
#include "global_body_planner/prm.h"

TEST(PRMTest, SamplePointTest) {
    PRM prm;
    PRM_PlannerClass G(planner_config, FORWARD);
    State sampled_state = prm.samplePoint(G, planner_config);
    ASSERT_TRUE(isValidState(sampled_state, planner_config, LEAP_STANCE));
}

TEST(PRMTest, AStarPathTest) {
    PRM prm;
    PRM_PlannerClass G(planner_config, FORWARD);
    prm.buildRoadmap(G, planner_config, 100, 5.0);
    auto path = prm.findPathUsingAStar(G, 0, 10);
    ASSERT_GT(path.size(), 0);  // Ensure a path was found
}
