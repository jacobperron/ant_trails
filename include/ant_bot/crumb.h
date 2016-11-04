/**
Software License Agreement (BSD)

\file      crumb.h
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
#ifndef ANT_BOT_CRUMB_H
#define ANT_BOT_CRUMB_H

#include "stage.hh"

namespace ant_bot
{

class Crumb
{
public:
  Crumb(Stg::World* world,
        const Stg::Pose pose,
        const double weight,
        const int goal,
        const double expire_duration);
  ~Crumb();

  bool isStale() const;

  inline double getWeight() const
  {
    return weight_;
  }

  inline void setWeight(const double& w)
  {
    weight_ = w;
  }

  inline int getGoal() const
  {
    return goal_;
  }

  inline void setGoal(const int& g)
  {
    goal_ = g;
  }

  inline double getCreationTime() const
  {
    return creation_time_;
  }

  inline Stg::Pose getPose() const
  {
    return pose_;
  }

  bool operator <(const Crumb& c)
  {
    return (creation_time_ < c.getCreationTime());
  }

private:
  Stg::World* world_;  // For getting time
  Stg::Pose pose_;
  double weight_;
  int goal_;
  double creation_time_;  // microseconds
  double expire_duration_;  // milliseconds

};  // class Crumb

}  // namespace ant_bot

#endif  // ANT_BOT_CRUMB_H
