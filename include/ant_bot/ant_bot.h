/**
Software License Agreement (BSD)

\file      ant_bot.h
\authors   Jacob Perron <jperron@sfu.ca>
\copyright Copyright (c) 2016, Jacob Perron, All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef ANT_BOT_ANT_BOT_H
#define ANT_BOT_ANT_BOT_H

#include <algorithm>
#include <vector>

#include "stage.hh"

#include "ant_bot/crumb.h"

namespace ant_bot
{

// Global lists of crumbs leading to food and home. Newest crumbs are appended to the end.
std::vector<Crumb> crumbs_to_food;
std::vector<Crumb> crumbs_to_home;

// Number of AntBot's in world
static int num_ants = 0;

/** Robot controller for AntBot
 *
 *  Assumption: AntBots start in a position such that their home is detectable
 */
class AntBot
{
public:
  // TODO(jacobperron): Pass parameters to system in configuration file
  // Velocity limits
  static const double MAX_X_VEL = 0.5;  // m/s
  static const double MAX_YAW_VEL = 4.0;  // Radians/s

  // Velocities
  static const double CRUISE_X_VEL = 0.4;  // m/s
  static const double AVOID_YAW_VEL = 0.5;  // Radians/s
  static const double AVOID_X_VEL = 0.05;  // m/s

  // Obstacle avoidence parameters
  static const int AVOID_DURATION = 10;
  static const double AVOID_FRONT_DISTANCE = 0.6;  // m
  static const double AVOID_SIDE_DISTANCE = 0.4;  // m
  static const double AVOID_STOP_DISTANCE = 0.3;  // m

  // Gain for servoing to objects
  static const double YAW_VEL_P_GAIN = 1.0;

  // Number of sonars equipped on robot
  static const int NUM_RANGE_SENSORS = 8;

  // Distance between crumbs laid
  static const double MIN_CRUMB_DISTANCE = 0.3;  // m

  // Crumbs evaporate once laid after this duration
  static const double CRUMB_EXPIRE_DURATION = 300000.0;  // ms

  // Distance to center of HOME to be considered arrived
  static const double MIN_HOME_RANGE = 0.2;  // m

  // Distance to center of a FOOD source to be considered arrived
  static const double MIN_FOOD_RANGE = 0.3;  // m

  // Range of crumb sensor
  static const double MIN_CRUMB_FIDUCIAL_RANGE = 2.0;  // m

  // Half FOV of crumb sensor (value of pi can detect crumbs with 360 degree FOV)
  static const double CRUMB_FIDUCIAL_HALF_FOV = M_PI;  // Radians

  // Distance to maintain from anti-crumbs
  static const double SO_LOST_CRUMB_DISTANCE = 1.0;

  static inline double clamp(const double val, const double max_val, const double min_val)
  {
    return std::max(std::min(val, max_val), min_val);
  }

  static inline double computeBearing(const Stg::Pose& from, const Stg::Pose& to)
  {
     return Stg::normalize(atan2(to.y - from.y, to.x - from.x) - from.a);
  }

  static inline double computeEuclidDistance(const Stg::Pose& p0, const Stg::Pose& p1)
  {
    const double x_diff = p0.x - p1.x;
    const double y_diff = p0.y - p1.y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
  }


  enum AntBotFiducial
  {
    FIDUCIAL_ANT_BOT = 30,
    FIDUCIAL_HOME = 31,
    FIDUCIAL_FOOD = 32,
    FIDUCIAL_CRUMB_HOME = 33,
    FIDUCIAL_CRUMB_FOOD = 34
  };

  enum AntBotState
  {
    STATE_SEEK_FOOD,
    STATE_SEEK_HOME
  };

public:
  AntBot(Stg::ModelPosition* pose, const bool& so_lost = true);
  ~AntBot();

  int update();

  inline void setLoopHz(const double& loop_hz)
  {
    assert(loop_hz > 0.0f);
    loop_hz_ = loop_hz;
    loop_interval_ = 1000.0 / loop_hz_;
  }

protected:
  Stg::ModelPosition* pose_;
  Stg::World* world_;
  Stg::ModelRanger* ranger_;
  Stg::ModelFiducial* fiducial_;
  AntBotState state_;

  bool readRangerData();
  bool readFiducialData();

  /**
   * Obstacle avoidence routine
   *
   * @return true if no obstacles, false otherwise
   */
  bool obstacleAvoid();
  bool senseCrumb(const Stg::Pose& crumb_pose) const;
  bool checkCrumbs(const int goal);
  bool isFood() const;
  bool isHome() const;

  /**
   * Lay a crumb as part of this AntBots local trail
   *
   *  @param pose: (x, y) world position of the crumb
   *  @param distance: Distance in meters travelled since the start of the current trail
   *  @param goal: The goal to where this trail leads
   */
  void layCrumb(const Stg::Pose& pose, const double& distance, const int& goal);

  /**
   * Broadcasts the local trail of crumbs to the global trail and then clears the local trail
   */
  void broadcastCrumbs();

  /**
   * Clears the local crumb trail and resets internal state accordingly
   */
  void clearCrumbs();

  // TODO(jacobperron): Would be nice to use these instead of requiring a static callback below
  //int poseCallback(Stg::ModelPosition* new_pose, AntBot* robot);
  //int rangerCallback(Stg::ModelRanger* ranger, AntBot* robot);
  //int fiducialCallback(Stg::ModelFiducial* fiducial, AntBot* robot);

private:
  // Local trail of crumbs
  std::vector<Crumb> local_crumbs_;

  // Should use SO-LOST algorithm
  bool so_lost_;

  // TODO(jacobperron): Simulated time is only incrementing every 100ms even though GUI shows higher resolution
  double loop_hz_;  // Hz
  double loop_interval_;  // milliseconds
  double prev_update_time_;  // microseconds

  double x_vel_;
  double yaw_vel_;
  double ranges_[NUM_RANGE_SENSORS];
  double min_range_left_;

  // Distance travelled since last goal
  double distance_travelled_;

  // Keep track of iterations reacting to obstacle
  int avoid_count_;

  // Can AntBot sense home
  bool is_home_;
  // Can AntBot sense food
  bool is_food_;

  // Last detected distance and bearing to home
  double distance_home_;
  double bearing_home_;

  // Last detected distance and bearing to nearest food
  double distance_food_;
  double bearing_food_;

  bool draw_crumbs_;

  bool is_first_broadcast_;

  // Current pose of AntBot
  Stg::Pose curr_pose_;

  // Previous pose of AntBot (for computing distance travelled)
  Stg::Pose prev_pose_;

  // Next navigation waypoint
  Stg::Pose nav_goal_pose_;

  Stg::Pose prev_laid_crumb_pose_;
};  // class AntBot

static int poseCallback(Stg::ModelPosition* new_pose, AntBot* robot)
{
  return robot->update();
}

}  // namespace ant_bot

#endif  // ANT_BOT_ANT_BOT_H
