/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

#include <realm_ros/stage_node.h>
#include <realm_ros/grabber_exiv2_node.h>


using namespace realm;

int main(int argc, char **argv)
{
  Exiv2GrabberNode exiv_grabber_node;
  StageNode stage_node;
  while (true)
  {
    Frame::Ptr frame = exiv_grabber_node.spin();
    if(frame != nullptr) 
    {
      std::cout << "GOT FRAME" << std::endl;
      stage_node.spin(frame);
    } 
    else { std::cout << "NULL POINTER" << std::endl; }
  } 
  
  return 0;
}

