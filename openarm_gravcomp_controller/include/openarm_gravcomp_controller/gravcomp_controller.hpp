#pragma once
#include "openarm_gravcomp_controller/visibility_control.hpp"
#include "controller_interface/controller_interface.hpp"

namespace gravcomp_controller {

class GravCompController : public controller_interface::ControllerInterface
{
public:
  GravCompController();
  ~GravCompController() = default;
};

}  // namespace gravcomp_controller
