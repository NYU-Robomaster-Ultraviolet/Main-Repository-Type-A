/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of taproot-template-project.
 *
 * taproot-template-project is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * taproot-template-project is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with taproot-template-project.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DRIVERS_HPP_
#define DRIVERS_HPP_

#include "tap/drivers.hpp"
#include "controls/standard/control_interface.hpp"
#include "controls/standard/imu_interface.hpp"

namespace src
{
class Drivers : public tap::Drivers
{
    friend class DriversSingleton;

#ifdef ENV_UNIT_TESTS
public:
#endif
    Drivers() : 
        tap::Drivers(),
        control_interface(this),
        imu_rad_interface(this)
         {}

public:
    src::control::ControlInterface control_interface;
    ImuRadInterface imu_rad_interface;
    
};  // class Drivers

}  // namespace src

#endif  // DRIVERS_HPP_
