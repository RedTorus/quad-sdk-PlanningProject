#include "global_body_planner/global_body_planner.h"
#include <ros/package.h>
#include <quad_utils/collision_checker.h>
#include <quad_utils/bounding_boxes.h>
#include "global_body_planner/prm_planner_class.h"
#include "global_body_planner/prm.h"

using namespace planning_utils;

GlobalBodyPlanner::GlobalBodyPlanner(ros::NodeHandle nh): PRM_Graph(planner_config_) {
  nh_ = nh;
  

  // Load rosparams from parameter server
  std::string body_plan_topic, discrete_body_plan_topic, body_plan_tree_topic,
      goal_state_topic;
  std::vector<double> goal_state_vec(2);

  quad_utils::loadROSParam(nh_, "topics/start_state", robot_state_topic_);
  quad_utils::loadROSParam(nh_, "topics/goal_state", goal_state_topic);
  quad_utils::loadROSParam(nh_, "/topics/terrain_map", terrain_map_topic_);
  quad_utils::loadROSParam(nh_, "topics/global_plan", body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_discrete",
                           discrete_body_plan_topic);
  quad_utils::loadROSParam(nh_, "topics/global_plan_tree",
                           body_plan_tree_topic);
  quad_utils::loadROSParam(nh_, "/map_frame", map_frame_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/update_rate",
                           update_rate_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/num_calls", num_calls_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/max_planning_time",
                           max_planning_time_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/pos_error_threshold",
                           pos_error_threshold_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/startup_delay",
                           reset_publish_delay_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/replanning",
                           replanning_allowed_);
  quad_utils::loadROSParam(nh_, "/local_planner/timestep", dt_);
  quad_utils::loadROSParam(nh_, "/global_body_planner/goal_state",
                           goal_state_vec);

  // Setup pubs and subs
  terrain_map_sub_ = nh_.subscribe(
      terrain_map_topic_, 1, &GlobalBodyPlanner::terrainMapCallback, this);
  robot_state_sub_ = nh_.subscribe(
      robot_state_topic_, 1, &GlobalBodyPlanner::robotStateCallback, this);
  goal_state_sub_ = nh_.subscribe(goal_state_topic, 1,
                                  &GlobalBodyPlanner::goalStateCallback, this);
  body_plan_pub_ = nh_.advertise<quad_msgs::RobotPlan>(body_plan_topic, 1);
  discrete_body_plan_pub_ =
      nh_.advertise<quad_msgs::RobotPlan>(discrete_body_plan_topic, 1);
  tree_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(body_plan_tree_topic, 1);

  // Load planner config
  bool enable_leaping;
  planner_config_.loadParamsFromServer(nh);
  planner_config_.bounding_boxes = BoundingBoxes(nh_, planner_config_.yaml);
  //ROS_INFO("GBP: yaml: %s", planner_config_.yaml.c_str());
  planner_config_.BB = planner_config_.bounding_boxes.getBoundingBoxes();
  //ROS_INFO("GBP: BB size: %d", planner_config_.BB.size());
  planner_config_.collision_checker = std::make_shared<CollisionChecker>(
      planner_config_.bounding_boxes);
  //std::make_shared<BoundingBoxes>(nh_,planner_config_.yaml);
  /* planner_config_.BB = planner_config_.bounding_boxes->getBoundingBoxes();
  planner_config_.collision_checker = std::make_shared<CollisionChecker>(
      planner_config_.bounding_boxes); */
  //////////////////////////////////////////////
  nh_.param<bool>("global_body_planner/enable_leaping", enable_leaping, true);
  if (!enable_leaping) {
    planner_config_.enable_leaping = false;
    planner_config_.num_leap_samples = 0;
    planner_config_.h_min = 0;
    planner_config_.h_max = 0.5;
  }

 // Initialize BoundingBoxes with YAML file path



  // Fill in the goal state information
  goal_state_vec.resize(12, 0);
  vectorToFullState(goal_state_vec, goal_state_);

  // Zero planning data
  start_index_ = 0;
  triggerReset();

  //initializePlannerConfig(planner_config_);
}

void GlobalBodyPlanner::initializePlannerConfig() {
    // Initialize PRM_Graph now that planner_config_ is available
    PRM_Graph = PRM_PlannerClass(planner_config_);
}

void GlobalBodyPlanner::terrainMapCallback(
    const grid_map_msgs::GridMap::ConstPtr& msg) {
  // Get the map in its native form
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);

  // Convert to FastTerrainMap structure for faster querying
  planner_config_.terrain.loadDataFromGridMap(map);  // Takes ~10ms
  planner_config_.terrain_grid_map = map;            // Takes ~0.1ms

  // Uodate the goal state of the planner
  goal_state_.pos[2] =
      planner_config_.h_nom + planner_config_.terrain.getGroundHeight(
                                  goal_state_.pos[0], goal_state_.pos[1]);
}

void GlobalBodyPlanner::robotStateCallback(
    const quad_msgs::RobotState::ConstPtr& msg) {
  eigenToFullState(quad_utils::bodyStateMsgToEigen(msg->body), robot_state_);
}

void GlobalBodyPlanner::triggerReset() {
  planner_status_ = RESET;
  current_plan_.clear();
  reset_time_ = ros::Time::now();
}

void GlobalBodyPlanner::goalStateCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg) {
  // If same as previous goal state, ignore
  if (goal_state_msg_ != NULL) {
    if (goal_state_msg_->header.stamp == msg->header.stamp) {
      return;
    }
  }

  // Load the message
  goal_state_msg_ = msg;

  // Store the x and y locations along with the terrain height (this will be
  // overriden)
  goal_state_.pos[0] = goal_state_msg_->point.x;
  goal_state_.pos[1] = goal_state_msg_->point.y;
  goal_state_.pos[2] = planner_config_.h_nom +
                       planner_config_.terrain.getGroundHeight(
                           goal_state_msg_->point.x, goal_state_msg_->point.y);

  // Invalidate the current plan to force a new one
  current_plan_.invalidate();

  // If the old plan has been executed, allow full replanning, otherwise
  // immediately update plan
  if (current_plan_.getDuration() <=
      (ros::Time::now() - current_plan_.getPublishedTimestamp()).toSec()) {
    triggerReset();
  }
}

void GlobalBodyPlanner::setStartState() {
  // Reset if too far from plan
  if (!current_plan_.isEmpty() && !publish_after_reset_delay_) {
    int current_index;
    double first_element_duration;
    quad_utils::getPlanIndex(current_plan_.getPublishedTimestamp(), dt_,
                             current_index, first_element_duration);
    current_index = std::min(current_index, current_plan_.getSize() - 1);
    FullState current_state_in_plan_ =
        current_plan_.getStateFromIndex(current_index);
    if (poseDistance(robot_state_, current_state_in_plan_) >
        pos_error_threshold_) {
      ROS_WARN_THROTTLE(0.5, "Too far from nominal plan, resetting");
      triggerReset();
    }
  }

  if (planner_status_ == RESET) {
    ROS_INFO_THROTTLE(2, "In reset mode");
    start_state_ = robot_state_;
    replan_start_time_ = 0;
    start_index_ = 0;
    publish_after_reset_delay_ = true;

  } else if (planner_status_ == REFINE) {
    ROS_INFO_THROTTLE(2, "GBP in refine mode");

    start_index_ =
        std::floor((ros::Time::now() + ros::Duration(max_planning_time_) -
                    current_plan_.getPublishedTimestamp())
                       .toSec() /
                   dt_);

    // Ensure start index is not too close to goal
    start_index_ = (start_index_ + 25 >= current_plan_.getSize() - 1)
                       ? current_plan_.getSize() - 1
                       : start_index_;

    // Iterate until start_index is in a connect phase
    while (current_plan_.getPrimitiveFromIndex(start_index_) != CONNECT &&
           start_index_ < current_plan_.getSize() - 1) {
      start_index_++;
    }

    start_state_ = current_plan_.getStateFromIndex(start_index_);
    replan_start_time_ = current_plan_.getTime(start_index_);

  } else {
    ROS_ERROR("Invalid planning status");
  }
}

void GlobalBodyPlanner::setGoalState() {}

bool GlobalBodyPlanner::callPlanner() {
  if (!replanning_allowed_ && !publish_after_reset_delay_) {
    newest_plan_.setComputedTimestamp(ros::Time::now());
    return false;
  }

  newest_plan_ = current_plan_;

  // Clear out old statistics
  solve_time_info_.clear();
  vertices_generated_info_.clear();
  cost_vector_.clear();
  cost_vector_times_.clear();

  // Copy start and goal states and adjust for ground height
  State start_state = fullStateToState(start_state_);
  State goal_state = fullStateToState(goal_state_);

  // Initialize statistics variables
  double plan_time, path_length, path_duration, total_solve_time,
      total_vertices_generated, total_path_length, total_path_duration,
      dist_to_goal;
  int vertices_generated;

  // Construct RRT object
  GBPL gbpl;

  // Loop through num_calls_ planner calls
  for (int i = 0; i < num_calls_; ++i) {
    // Exit if ros is down
    if (!ros::ok()) {
      return false;
    }

    // Clear out previous solutions and initialize new statistics variables
    std::vector<State> state_sequence;
    std::vector<Action> action_sequence;

    // Call the planner method
    int plan_status = gbpl.findPlan2(planner_config_, start_state, goal_state,
                                    state_sequence, action_sequence, tree_pub_);
    newest_plan_.setComputedTimestamp(ros::Time::now());

    if (plan_status != VALID && plan_status != VALID_PARTIAL) {
      if (plan_status == INVALID_START_STATE) {
        ROS_WARN_THROTTLE(1, "Invalid start state, exiting");
      } else if (plan_status == INVALID_GOAL_STATE) {
        ROS_WARN_THROTTLE(1, "Invalid goal state, exiting");
      } else if (plan_status == INVALID_START_GOAL_EQUAL) {
        ROS_WARN_THROTTLE(1, "Start is sufficiently close to goal, exiting");
      } else if (plan_status == UNSOLVED) {
        ROS_WARN_THROTTLE(1,
                          "Planner was unable to make any progress, start "
                          "state likely trapped");
      }
      return false;
    }
    gbpl.getStatistics(plan_time, vertices_generated, path_length,
                       path_duration, dist_to_goal);

    // Add the existing path length to the new
    path_length += current_plan_.getLengthAtIndex(start_index_);

    // Handle the statistical data
    cost_vector_.push_back(path_length);
    cost_vector_times_.push_back(plan_time);

    total_solve_time += plan_time;
    total_vertices_generated += vertices_generated;
    total_path_length += path_length;
    total_path_duration += path_duration;

    solve_time_info_.push_back(plan_time);
    vertices_generated_info_.push_back(vertices_generated);

    newest_plan_.eraseAfterIndex(start_index_);
    newest_plan_.loadPlanData(plan_status, start_state_, dist_to_goal,
                              state_sequence, action_sequence, dt_,
                              replan_start_time_, planner_config_);

    // Check if this plan is better:
    // 1) If valid and shorter or previous plan not valid OR
    // 2) If partially valid and closer to the goal OR
    // 3) If goal has moved
    double eps = 0.99;  // Require significant improvement
    bool is_updated = false;
    if ((plan_status == VALID) &&
        ((newest_plan_.getLength() / eps) < current_plan_.getLength() ||
         current_plan_.getStatus() != VALID)) {
      ROS_INFO("valid and shorter or previous plan not valid");
      is_updated = true;

    } else if ((plan_status == VALID_PARTIAL) &&
               (current_plan_.getStatus() == UNSOLVED ||
                (poseDistance(state_sequence.back(), goal_state) <
                 current_plan_.getGoalDistance()))) {
      ROS_INFO("partially valid and closer to the goal");
      is_updated = true;
    }

    if (is_updated) {
      state_sequence_ = state_sequence;
      action_sequence_ = action_sequence;

      std::cout << "Solve time: " << plan_time << " s" << std::endl;
      std::cout << "Vertices generated: " << vertices_generated << std::endl;
      std::cout << "Path length: " << path_length << " m" << std::endl;
      std::cout << "Path duration: " << path_duration << " s" << std::endl;
      std::cout << std::endl;

      current_plan_ = newest_plan_;
    }

    return is_updated;
  }

  // Report averaged statistics if num_calls_ > 1
  if (num_calls_ > 1) {
    std::cout << "Average vertices generated: "
              << total_vertices_generated / num_calls_ << std::endl;
    std::cout << "Average solve time: " << total_solve_time / num_calls_ << " s"
              << std::endl;
    std::cout << "Average path length: " << total_path_length / num_calls_
              << " s" << std::endl;
    std::cout << "Average path duration: " << total_path_duration / num_calls_
              << " s" << std::endl;
    std::cout << std::endl;
  }

  return true;
}

bool GlobalBodyPlanner::callPlanner2() {
  if (!replanning_allowed_ && !publish_after_reset_delay_) {
    newest_plan_.setComputedTimestamp(ros::Time::now());
    return false;
  }

  newest_plan_ = current_plan_;

  // Clear out old statistics
  solve_time_info_.clear();
  vertices_generated_info_.clear();
  cost_vector_.clear();
  cost_vector_times_.clear();

  // Copy start and goal states and adjust for ground height
  State start_state = fullStateToState(start_state_);
  State goal_state = fullStateToState(goal_state_);

  // Initialize statistics variables
  double plan_time, path_length, path_duration, total_solve_time,
      total_vertices_generated, total_path_length, total_path_duration,
      dist_to_goal;
  int vertices_generated;

  // Construct RRT object
  GBPL gbpl;

  // Loop through num_calls_ planner calls
  for (int i = 0; i < num_calls_; ++i) {
    // Exit if ros is down
    if (!ros::ok()) {
      return false;
    }

    PRM prm;
    bool current_collison = true;
    std::vector<State> current_state_sequence;
    for (const auto& full_state : current_plan_.getBodyPlan()) {
        current_state_sequence.push_back(fullStateToState(full_state));
    }
    if(current_state_sequence.size()!=0){
      current_collison = prm.checkTrajectoryCollsion(PRM_Graph, current_state_sequence, planner_config_);
      ROS_INFO("-------------------------------current collison: %d", current_collison);
    }
    if(current_state_sequence.size() != 0 && !current_collison){
      ROS_INFO("Not replanning, no collision");
      return false;
      
    }
    // Clear out previous solutions and initialize new statistics variables
    std::vector<State> state_sequence;
    std::vector<Action> action_sequence;

    // Call the planner method
    int plan_status = gbpl.findPlan3(planner_config_, PRM_Graph, start_state, goal_state,
                                    state_sequence, action_sequence, tree_pub_);
    newest_plan_.setComputedTimestamp(ros::Time::now());

    if (plan_status != VALID && plan_status != VALID_PARTIAL) {
      if (plan_status == INVALID_START_STATE) {
        ROS_WARN_THROTTLE(1, "Invalid start state, exiting");
      } else if (plan_status == INVALID_GOAL_STATE) {
        ROS_WARN_THROTTLE(1, "Invalid goal state, exiting");
      } else if (plan_status == INVALID_START_GOAL_EQUAL) {
        ROS_WARN_THROTTLE(1, "Start is sufficiently close to goal, exiting");
      } else if (plan_status == UNSOLVED) {
        ROS_WARN_THROTTLE(1,
                          "Planner was unable to make any progress, start "
                          "state likely trapped");
      }
      return false;
    }
    gbpl.getStatistics(plan_time, vertices_generated, path_length,
                       path_duration, dist_to_goal);

    // Add the existing path length to the new
    path_length += current_plan_.getLengthAtIndex(start_index_);

    // Handle the statistical data
    cost_vector_.push_back(path_length);
    cost_vector_times_.push_back(plan_time);

    total_solve_time += plan_time;
    total_vertices_generated += vertices_generated;
    total_path_length += path_length;
    total_path_duration += path_duration;

    solve_time_info_.push_back(plan_time);
    vertices_generated_info_.push_back(vertices_generated);

    newest_plan_.eraseAfterIndex(start_index_);
    newest_plan_.loadPlanData(plan_status, start_state_, dist_to_goal,
                              state_sequence, action_sequence, dt_,
                              replan_start_time_, planner_config_);

    
    bool is_updated = false;

    
    /* std::vector<State> current_state_sequence; */
    std::vector<State> new_state_sequence;

    // Convert FullState to State
    /* for (const auto& full_state : current_plan_.getBodyPlan()) {
        current_state_sequence.push_back(fullStateToState(full_state));
    } */
    for (const auto& full_state : newest_plan_.getBodyPlan()) {
        new_state_sequence.push_back(fullStateToState(full_state));
    }
    //ROS_INFO("-------------------------------current_state_sequence size: %d", current_state_sequence.size());
    ROS_INFO("-------------------------------new_state_sequence size: %d", new_state_sequence.size());
    //bool current_collison = true;
    bool new_collison = true;
    if (current_state_sequence.size() != 0 && new_state_sequence.size() != 0) {
      //current_collison = !prm.checkTrajectoryCollsion(PRM_Graph, current_state_sequence, planner_config_);
      new_collison = prm.checkTrajectoryCollsion(PRM_Graph, new_state_sequence, planner_config_);

      ROS_INFO("-------------------------------current collison: %d", current_collison);
      ROS_INFO("-------------------------------new collison: %d", new_collison);

      if (current_collison) {
        ROS_INFO("-------------------------------current plan set to new plan");
        is_updated = true;
      }
      if (current_collison && new_collison) {
        ROS_WARN("Both plans have collision");
      }
    }

    if (current_state_sequence.size() == 0) {
      is_updated = true;
    }

    // Check if this plan is better:
    // 1) If valid and shorter or previous plan not valid OR
    // 2) If partially valid and closer to the goal OR
    // 3) If goal has moved
    double eps = 0.985;  // Require significant improvement
    if ((plan_status == VALID) &&
        ((newest_plan_.getLength() / eps) < current_plan_.getLength() ||
         current_plan_.getStatus() != VALID)) {
      is_updated = true;
    } else if ((plan_status == VALID_PARTIAL) &&
               (current_plan_.getStatus() == UNSOLVED ||
                (poseDistance(state_sequence.back(), goal_state) <
                 current_plan_.getGoalDistance()))) {
      is_updated = true;
    }
    

    if (is_updated) {
      state_sequence_ = state_sequence;
      action_sequence_ = action_sequence;

      std::cout << "Solve time: " << plan_time << " s" << std::endl;
      std::cout << "Vertices generated: " << vertices_generated << std::endl;
      std::cout << "Path length: " << path_length << " m" << std::endl;
      std::cout << "Path duration: " << path_duration << " s" << std::endl;
      std::cout << std::endl;

      current_plan_ = newest_plan_;
    }

    return is_updated;
  }

  // Report averaged statistics if num_calls_ > 1
  if (num_calls_ > 1) {
    std::cout << "Average vertices generated: "
              << total_vertices_generated / num_calls_ << std::endl;
    std::cout << "Average solve time: " << total_solve_time / num_calls_ << " s"
              << std::endl;
    std::cout << "Average path length: " << total_path_length / num_calls_
              << " s" << std::endl;
    std::cout << "Average path duration: " << total_path_duration / num_calls_
              << " s" << std::endl;
    std::cout << std::endl;
  }

  return true;
}

void GlobalBodyPlanner::waitForData() {
  // Spin until terrain map message has been received and processed
  boost::shared_ptr<grid_map_msgs::GridMap const> shared_map;
  while ((shared_map == nullptr) && ros::ok()) {
    shared_map = ros::topic::waitForMessage<grid_map_msgs::GridMap>(
        terrain_map_topic_, nh_);
    ros::spinOnce();
  }

  boost::shared_ptr<quad_msgs::RobotState const> shared_robot_state;
  while ((shared_robot_state == nullptr) && ros::ok()) {
    shared_robot_state = ros::topic::waitForMessage<quad_msgs::RobotState>(
        robot_state_topic_, nh_);
    ros::spinOnce();
  }
  ROS_INFO("GBP has state and map information");
  reset_time_ = ros::Time::now();
}

void GlobalBodyPlanner::getInitialPlan() {
  // Keep track of when the planner started
  ros::Time start_time = ros::Time::now();

  bool success = false;
  ROS_INFO("---------------Starting initial plan");

  // Repeatedly call the planner until the startup delay has elapsed
  while (ros::ok() && ((ros::Time::now() - start_time) <
                       ros::Duration(reset_publish_delay_))) {
    success = callPlanner();
  }
}

void GlobalBodyPlanner::publishCurrentPlan() {
  // Conditions for publishing current plan:
  // 1) Plan not empty AND
  // 2) Reset publish delay has passed AND
  // 3) One of the following conditions is met:
  //    a) Current plan not yet published after reset
  //    b) The new plan is the best plan

  // Check conditions 1) and 2) return if false
  if (current_plan_.isEmpty() ||
      ((ros::Time::now() - reset_time_).toSec() <= reset_publish_delay_))
    return;

  // Check condition 3
  if (publish_after_reset_delay_ || newest_plan_ == current_plan_) {
    // If this is a reset, update the timestamp and switch back to refinement
    // mode
    if (publish_after_reset_delay_) {
      ROS_INFO("Switching to refinement mode");
      current_plan_.setPublishedTimestamp(ros::Time::now());
      planner_status_ = REFINE;
      publish_after_reset_delay_ = false;
    }

    // Declare the messages for interpolated body plan and discrete states,
    // initialize their headers
    quad_msgs::RobotPlan robot_plan_msg;
    quad_msgs::RobotPlan discrete_robot_plan_msg;
    robot_plan_msg.header.frame_id = map_frame_;
    robot_plan_msg.header.stamp = ros::Time::now();
    discrete_robot_plan_msg.header = robot_plan_msg.header;

    // Initialize the headers and types
    robot_plan_msg.global_plan_timestamp =
        current_plan_.getPublishedTimestamp();
    discrete_robot_plan_msg.global_plan_timestamp =
        current_plan_.getPublishedTimestamp();

    // Load the plan into the messages
    current_plan_.convertToMsg(robot_plan_msg, discrete_robot_plan_msg);

    // Publish both messages
    body_plan_pub_.publish(robot_plan_msg);
    discrete_body_plan_pub_.publish(discrete_robot_plan_msg);

    ROS_WARN("New plan published, stamp = %f",
             robot_plan_msg.global_plan_timestamp.toSec());
  }
}

void GlobalBodyPlanner::spin() {
  ros::Rate r(update_rate_);

  // Wait until we get map and state data
  waitForData();

  // Enter main spin
  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();

    // Set the start and goal states
    setStartState();
    setGoalState();

    // Call the planner
    //ROS_INFO("----------In SPIN Calling planner");
    //ROS_INFO("planner status: %d", planner_status_);
    callPlanner2();
    ROS_INFO("Curreent plan state size: %d", current_plan_.getBodyPlan().size());
    // Publish the results if valid
    publishCurrentPlan();

    r.sleep();
  }
}
