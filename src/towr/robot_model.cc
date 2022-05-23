/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/models/robot_model.h>

#include <towr/models/examples/monoped_model.h>
#include <towr/models/examples/biped_model.h>
#include <towr/models/examples/hyq_model.h>
#include <towr/models/examples/anymal_model.h>
#include <towr/models/examples/dogbot_model.h>
namespace towr {


RobotModel::RobotModel(Robot robot, const double z_value, const double z_dev, const double Ixx_value, const double Iyy_value, const double Izz_value, const double Ixy_value, const double Ixz_value, const double Iyz_value)
{
  switch (robot) {
    case Monoped:
      dynamic_model_   = std::make_shared<MonopedDynamicModel>();
      kinematic_model_ = std::make_shared<MonopedKinematicModel>();
      break;
    case Biped:
      dynamic_model_   = std::make_shared<BipedDynamicModel>();
      kinematic_model_ = std::make_shared<BipedKinematicModel>();
      break;
    case Hyq:
      dynamic_model_   = std::make_shared<HyqDynamicModel>();
      kinematic_model_ = std::make_shared<HyqKinematicModel>();
      break;
    case Anymal:
      dynamic_model_   = std::make_shared<AnymalDynamicModel>();
      kinematic_model_ = std::make_shared<AnymalKinematicModel>();
      break;
    case Dogbot:
      dynamic_model_   = std::make_shared<DogbotDynamicModel>(  Ixx_value, Iyy_value,  Izz_value, Ixy_value,  Ixz_value,  Iyz_value);
      kinematic_model_ = std::make_shared<DogbotKinematicModel>(z_value, z_dev);
      break;
    default:
      assert(false); // Error: Robot model not implemented.
      break;
  }
}


} // namespace towr


