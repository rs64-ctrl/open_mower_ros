/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Roesmann
 *********************************************************************/

#ifndef OSCILLATION_DETECTOR_H__
#define OSCILLATION_DETECTOR_H__

#include <boost/circular_buffer.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ftc_local_planner
{

/**
 * @class FailureDetector
 * @brief This class implements methods in order to detect if the robot got stucked or is oscillating
 *
 * The StuckDetector analyzes the last N commanded velocities in order to detect whether the robot
 * might got stucked or oscillates between left/right/forward/backwards motions.
 *
 * Taken from teb_local_planner
 */
class FailureDetector
{
public:
    FailureDetector() {}
    ~FailureDetector() {}

    void setBufferLength(int length) { buffer_.set_capacity(length); }

    void update(const geometry_msgs::msg::TwistStamped & twist,
                double v_max, double v_backwards_max, double omega_max,
                double v_eps, double omega_eps);

    bool isOscillating() const;

    void clear();

protected:
    struct VelMeasurement
    {
        double v = 0;
        double omega = 0;
    };

    bool detect(double v_eps, double omega_eps);

private:
    boost::circular_buffer<VelMeasurement> buffer_;
    bool oscillating_ = false;
    static inline int sign(double x);
};

}  // namespace ftc_local_planner

#endif /* OSCILLATION_DETECTOR_H__ */
