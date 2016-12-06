/**
Software License Agreement (BSD)

\file      ant_bot.cpp
\authors   Jacob Perron <jperron@sfu.ca>
\copyright Copyright (c) 2016, Jacob Perron, All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <limits>
#include <vector>
#include <iostream>
#include <cassert>
#include <algorithm>

#include "ant_bot/crumb.h"
#include "ant_bot/ant_bot.h"

namespace ant_bot
{

AntBot::AntBot(Stg::ModelPosition* pose, const bool& so_lost)
  : pose_(pose),
    world_(pose->GetWorld()),
    ranger_((Stg::ModelRanger*) pose->GetChild("ranger:0")),
    fiducial_((Stg::ModelFiducial*) pose->GetUnusedModelOfType("fiducial")),
    state_(STATE_SEEK_FOOD),
    so_lost_(so_lost),
    x_vel_(0.0),
    yaw_vel_(0.0),
    min_range_left_(1000.0), //std::numeric_limits<double>::max()),
    avoid_count_(0),
    distance_travelled_(0.0),
    is_home_(false),
    is_food_(false),
    draw_crumbs_(false),
    is_first_broadcast_(true)
{
  assert(pose_);
  assert(world_);
  assert(ranger_);
  assert(fiducial_);

  setLoopHz(30.0);
  prev_update_time_ = world_->SimTimeNow();

  // Update controller on every pose update
  pose_->AddCallback(Stg::Model::CB_UPDATE, (Stg::model_callback_t) poseCallback, this);

  // Subscribe to sensors
  pose_->Subscribe();
  ranger_->Subscribe();
  fiducial_->Subscribe();

  // Initialize position
  curr_pose_ = pose_->GetPose();
  prev_pose_ = curr_pose_;

  // If this is the first AntBot, then give it some responsibility
  if (num_ants == 0)
  {
    draw_crumbs_ = true;
  }
  num_ants++;
}

AntBot::~AntBot()
{
  ;
}

int AntBot::update()
{
  // Throttle update rate
  if (((world_->SimTimeNow() - prev_update_time_) / 1000.0) < loop_interval_)
  {
    return 0;
  }
  else
  {
    prev_update_time_ = world_->SimTimeNow();
  }

  // Read position
  curr_pose_ = pose_->GetPose();

  // Add to distance travelled since last goal
  distance_travelled_ += computeEuclidDistance(curr_pose_, prev_pose_);

  // Read sensor data
  readRangerData();

  // Read fiducial data
  readFiducialData();

  // Drive straight unless attracted otherwise
  x_vel_ = CRUISE_X_VEL;
  yaw_vel_ = 0.0;

  // State machine
  switch (state_)
  {
    case STATE_SEEK_FOOD:
      pose_->Say(std::string("Food?"));

      // Have we found food?
      if (isFood())
      {
        if (distance_food_ < MIN_FOOD_RANGE)
        {
          // Get a resource!
          // TODO(jacobperron): use flags

          broadcastCrumbs();
          state_ = STATE_SEEK_HOME;
          break;
        }
        else
        {
          // Servo to food
          yaw_vel_ = YAW_VEL_P_GAIN * bearing_food_;
        }
      }
      else if (checkCrumbs(FIDUCIAL_CRUMB_FOOD))
      {
        // Servo to best looking crumb
        yaw_vel_ = YAW_VEL_P_GAIN * computeBearing(curr_pose_, nav_goal_pose_);
      }

      // Add to local list of crumbs directing towards food (assuming we find some)
      layCrumb(curr_pose_, distance_travelled_, FIDUCIAL_CRUMB_FOOD);
      break;

    case STATE_SEEK_HOME:
      pose_->Say(std::string("Home?"));
      if (isHome())
      {
        if (distance_home_ < MIN_HOME_RANGE)
        {
          // Drop off resource
          broadcastCrumbs();
          state_ = STATE_SEEK_FOOD;
          break;
        }
        else
        {
          // Servo home
          yaw_vel_ = YAW_VEL_P_GAIN * bearing_home_;
        }
      }
      else if (checkCrumbs(FIDUCIAL_CRUMB_HOME))
      {
        // Servo to crumb leading home
        yaw_vel_ = YAW_VEL_P_GAIN * computeBearing(curr_pose_, nav_goal_pose_);
      }

      // Lay crumbs directing towards food
      layCrumb(curr_pose_, distance_travelled_, FIDUCIAL_CRUMB_HOME);
      break;
  }  // end switch (state_)

  // Clamp velocities
  x_vel_ = clamp(x_vel_, MAX_X_VEL, -MAX_X_VEL);
  yaw_vel_ = clamp(yaw_vel_, MAX_YAW_VEL, -MAX_YAW_VEL);

  // If no obstacles, proceed as planned
  if (obstacleAvoid())
  {
    pose_->SetXSpeed(x_vel_);
    pose_->SetTurnSpeed(yaw_vel_);
  }

  prev_pose_ = curr_pose_;
  return 0;
}

bool AntBot::readRangerData()
{
  if (!ranger_)
  {
    std::cerr << "Ranger error: no ranger sensor detected" << std::endl;
    return false;
  }

  const std::vector<Stg::ModelRanger::Sensor>& sensors = ranger_->GetSensors();
  const size_t num_sensors = sensors.size();
  if (num_sensors < NUM_RANGE_SENSORS)
  {
    std::cout << "Ranger error: expected at least " << NUM_RANGE_SENSORS
              << " sensors, got " << num_sensors << std::endl;
    return false;
  }

  for (int i = 0; i < NUM_RANGE_SENSORS; i++)
  {
    ranges_[i] = sensors[i].ranges[0];
  }
  return true;
}

bool AntBot::readFiducialData()
{
  // Reset fiducial specific data
  is_home_ = false;
  is_food_ = false;
  distance_home_ = std::numeric_limits<double>::max();
  distance_food_ = std::numeric_limits<double>::max();

  if (!fiducial_)
  {
    std::cerr << "Fiducial error: no fiducial sensor detected" << std::endl;
    return false;
  }

  std::vector<Stg::ModelFiducial::Fiducial>& fids = fiducial_->GetFiducials();

  if (fids.size() == 0) return false;

  for (std::size_t i = 0; i < fids.size(); i++)
  {
    switch (fids[i].id)
    {
      case FIDUCIAL_HOME:
        is_home_ = true;
        bearing_home_ = fids[i].bearing;
        distance_home_ = fids[i].range;
        break;
      case FIDUCIAL_FOOD:
        is_food_ = true;
        // Choose closest food source
        if (fids[i].range < distance_food_)
        {
          bearing_food_ = fids[i].bearing;
          distance_food_ = fids[i].range;
        }
        break;
      case FIDUCIAL_CRUMB_HOME:
        break;
      case FIDUCIAL_CRUMB_FOOD:
        break;
    }
  }
  return  true;
}

bool AntBot::obstacleAvoid()
{
  // Check if obstacle front left
  const bool is_obstacle_front_left = (ranges_[1] < AVOID_FRONT_DISTANCE ||
                                       ranges_[2] < AVOID_FRONT_DISTANCE ||
                                       ranges_[3] < AVOID_FRONT_DISTANCE);

  // Check if obstacle front right
  const bool is_obstacle_front_right = (ranges_[4] < AVOID_FRONT_DISTANCE ||
                                        ranges_[5] < AVOID_FRONT_DISTANCE ||
                                        ranges_[6] < AVOID_FRONT_DISTANCE);

  // Check if obstacle side left
  const bool is_obstacle_side_left = (ranges_[0] < AVOID_SIDE_DISTANCE);

  // Check if obstacle side right
  const bool is_obstacle_side_right = (ranges_[7] < AVOID_SIDE_DISTANCE);

  const bool is_obstacle = is_obstacle_front_right || is_obstacle_side_right ||
                           is_obstacle_front_left  || is_obstacle_side_left;

  min_range_left_ = std::min(std::min(std::min(ranges_[1], ranges_[2]),
                                               ranges_[3]),
                                      ranges_[0]);
  const double min_range_right = std::min(std::min(std::min(ranges_[4], ranges_[5]),
                                                  ranges_[6]),
                                         ranges_[7]);

  bool stop = ((min_range_left_ < AVOID_STOP_DISTANCE) || min_range_right < AVOID_STOP_DISTANCE);

  const bool do_avoidence = (stop || is_obstacle || avoid_count_ > 0);
  if (do_avoidence)
  {
    pose_->SetXSpeed(((stop) ? 0.0 : AVOID_X_VEL));
    if (avoid_count_ < 1)
    {
      avoid_count_ = (random() % AVOID_DURATION) + AVOID_DURATION;
      pose_->SetTurnSpeed((min_range_left_ < min_range_right) ? -AVOID_YAW_VEL : AVOID_YAW_VEL);
    }
    avoid_count_--;
  }
  else
  {
    avoid_count_ = 0;
  }

  return !do_avoidence;
}

bool AntBot::senseCrumb(const Stg::Pose& crumb_pose) const
{
  // If crumb is outside sensor range, return false
  if (computeEuclidDistance(crumb_pose, curr_pose_) > MIN_CRUMB_FIDUCIAL_RANGE) return false;

  // If crumb is outside sensor FOV, return false
  if (fabs(computeBearing(curr_pose_, crumb_pose)) > CRUMB_FIDUCIAL_HALF_FOV) return false;

  return true;
}

bool AntBot::checkCrumbs(const int goal)
{
  int i;
  bool found_crumb = false;
  bool do_crumb_cleanup = false;
  bool do_anti_crumb_cleanup = false;
  double best_weight = std::numeric_limits<double>::max();

  Stg::Color color_home(0.65, 0.16, 0.16);
  Stg::Color color_food(0.0, 0.0, 1.0);

  // Clear reset crumbs to draw (waypoints) list
  if (draw_crumbs_)
  {
    // TODO(jacobperron): Select alpha based on staleness
    color_food.a = 0.7;
    color_home.a = 0.7;
    pose_->waypoints.clear();
  }

  // Select crumb list
  std::vector<Crumb> &crumbs = ((goal == FIDUCIAL_CRUMB_HOME) ? crumbs_to_home : crumbs_to_food);
  std::vector<Crumb> &anti_crumbs = ((goal == FIDUCIAL_CRUMB_HOME) ? crumbs_to_food : crumbs_to_home);

  // Iterate through crumbs from newest to oldest
  for (i = crumbs.size() - 1; i >= 0; i--)
  {
    const Crumb &crumb = crumbs[i];
    if (crumb.isStale())
    {
      do_crumb_cleanup = true;
      break;
    }

    if (draw_crumbs_)
    {
      // Drawing
      Stg::ModelPosition::Waypoint wp(crumb.getPose(),
                                     ((crumb.getGoal() == FIDUCIAL_CRUMB_HOME) ?
                                      color_home : color_food));
      pose_->waypoints.push_back(wp);
    }

    // If crumb leads to desired goal AND
    // It is within AntBot's sensor range AND
    // It is a better crumb THEN update navigation goal
    if ((crumb.getGoal() == goal) &&
        (senseCrumb(crumb.getPose())) &&
        (crumb.getWeight() <= best_weight))
    {
      best_weight = crumb.getWeight();
      nav_goal_pose_ = crumb.getPose();
      found_crumb = true;
    }
  }
  i++;

  // Loop over anti crumbs
  int j;
  bool shifted = false;
  for (j = anti_crumbs.size() - 1; j >= 0; j--)
  {
    const Crumb &anti_crumb = anti_crumbs[j];
    if (anti_crumb.isStale())
    {
      do_anti_crumb_cleanup = true;
      break;
    }

    if (draw_crumbs_)
    {
      // Drawing
      Stg::ModelPosition::Waypoint wp(anti_crumb.getPose(),
                                     ((anti_crumb.getGoal() == FIDUCIAL_CRUMB_HOME) ?
                                      color_home : color_food));
      pose_->waypoints.push_back(wp);
    }

    // SO-LOST: Check if near opposing crumbs and move navigation goal slightly away
    if (so_lost_ && found_crumb && !shifted)
    {
      Stg::Pose anti_crumb_pose = anti_crumb.getPose();

      const double dist_between_crumbs = computeEuclidDistance(anti_crumb_pose, nav_goal_pose_);
      if (dist_between_crumbs < SO_LOST_CRUMB_DISTANCE)
      {
        // NOTE(jacobperron): std::min crashes if given SO_LIST_CRUMB_DISTANCE directly, I guess because it is static
        const double so_lost_crumb_dist = SO_LOST_CRUMB_DISTANCE;
        const double shift_dist = std::min(min_range_left_, so_lost_crumb_dist) / 2.0;

        double shift_x;
        double shift_y;
        // Avoid case where nav goal and anti-crumb are in same position (divide by zero)
        if (dist_between_crumbs > 0.0001)
        {
          // Compute normalized vector from robot to crumb and rotate by 90 degrees to apply shift
          double robot_to_crumb_x = anti_crumb_pose.x - nav_goal_pose_.x;
          double robot_to_crumb_y = anti_crumb_pose.y - nav_goal_pose_.y;
          const double robot_to_crumb_mag = sqrt(robot_to_crumb_x * robot_to_crumb_x +
                                                 robot_to_crumb_y * robot_to_crumb_y);
          shift_y = (robot_to_crumb_x / robot_to_crumb_mag) * shift_dist;
          shift_x = (robot_to_crumb_y / robot_to_crumb_mag) * shift_dist;
        }
        else
        {
          shift_x = cos(nav_goal_pose_.a) * shift_dist;
          shift_y = sin(nav_goal_pose_.a) * shift_dist;
        }

        // Apply navigation goal shift
        nav_goal_pose_.x += shift_x;
        nav_goal_pose_.y += shift_y;
        shifted = true;
      }
    }
  }
  j++;

  if (do_crumb_cleanup)
  {
    // Erase all stale crumbs (calls Crumb destructor)
    crumbs.erase(crumbs.begin(), crumbs.begin() + i);
  }

  if (do_anti_crumb_cleanup)
  {
    // Erase all stale crumbs (calls Crumb destructor)
    anti_crumbs.erase(anti_crumbs.begin(), anti_crumbs.begin() + j);
  }

  return found_crumb;
}

bool AntBot::isFood() const
{
  return is_food_;
}

bool AntBot::isHome() const
{
  return is_home_;
}

void AntBot::broadcastCrumbs()
{
  if (local_crumbs_.size() < 1) return;

  // Infer global trail to broadcast to based on first crumb
  const int goal = local_crumbs_[0].getGoal();
  std::vector<Crumb> &crumbs = ((goal == FIDUCIAL_CRUMB_HOME) ? crumbs_to_home : crumbs_to_food);

  // If this is the first trail, then make it bidirectional
  if (is_first_broadcast_)
  {
    // Determine anti-goal
    const int anti_goal = (goal == FIDUCIAL_CRUMB_HOME) ? FIDUCIAL_CRUMB_FOOD : FIDUCIAL_CRUMB_HOME;

    // Determine anti-trail
    std::vector<Crumb> &anti_crumbs = ((anti_goal == FIDUCIAL_CRUMB_HOME) ? crumbs_to_home : crumbs_to_food);

    // Change current trail goal to anti-goal
    std::vector<Crumb> local_crumbs_copy = local_crumbs_;
    for (size_t i = 0; i < local_crumbs_copy.size(); i++)
    {
      local_crumbs_copy[i].setGoal(anti_goal);
    }

    // Merge local trail with global trail
    std::vector<Crumb> tmp_vec;
    std::merge(local_crumbs_copy.begin(), local_crumbs_copy.end(),
               anti_crumbs.begin(), anti_crumbs.end(),
               std::back_inserter(tmp_vec));

    // Refresh global crumbs
    anti_crumbs = tmp_vec;

    is_first_broadcast_ = false;
  }

  // Reverse distances of local crumbs so they approximate the distance to the destination
  for (size_t i = 0; i < (local_crumbs_.size() / 2); i++)
  {
    const size_t j = local_crumbs_.size() - i - 1;
    const double tmp_weight = local_crumbs_[i].getWeight();
    local_crumbs_[i].setWeight(local_crumbs_[j].getWeight());
    local_crumbs_[j].setWeight(tmp_weight);
  }

  // Merge local trail with global trail
  std::vector<Crumb> tmp_vec;
  std::merge(local_crumbs_.begin(), local_crumbs_.end(),
             crumbs.begin(), crumbs.end(),
             std::back_inserter(tmp_vec));

  // Refresh global crumbs
  crumbs = tmp_vec;

  // Clear local crumbs
  clearCrumbs();
}

void AntBot::clearCrumbs()
{
  local_crumbs_.clear();
  distance_travelled_ = 0.0;
}

void AntBot::layCrumb(const Stg::Pose& pose, const double& distance, const int& goal)
{
  // Make sure crumbs are spaced out
  if (computeEuclidDistance(prev_laid_crumb_pose_, pose) >= MIN_CRUMB_DISTANCE)
  {
    // TODO(jacobperron): Create custom CrumbModel
    // Create a new crumb and add it to the list
    Crumb* crumb = new Crumb(world_, pose, distance, goal, CRUMB_EXPIRE_DURATION);

    local_crumbs_.push_back(*crumb);

    prev_laid_crumb_pose_.x = pose.x;
    prev_laid_crumb_pose_.y = pose.y;
    prev_laid_crumb_pose_.a = pose.a;
  }
}

}  // namespace ant_bot

// Stage calls this when the model starts up
extern "C" int Init(Stg::Model* mod, Stg::CtrlArgs* args )
{
  // Only for first instance of controller
  if (ant_bot::num_ants == 0)
  {
    srandom(time(NULL));
  }
  ant_bot::AntBot* ant_bot = new ant_bot::AntBot((Stg::ModelPosition*) mod);
  return 0;
}
