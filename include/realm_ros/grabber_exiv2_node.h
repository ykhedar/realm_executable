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

#ifndef PROJECT_GRABBER_EXIV_NODE_H
#define PROJECT_GRABBER_EXIV_NODE_H

#include <iostream>
#include <time.h>
#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>

#include <realm_io/realm_import.h>
#include <realm_io/exif_import.h>
#include <realm_io/utilities.h>

namespace realm
{

/*!
 * @brief Simple "image from folder"-grabber for Exiv2 tagged images to convert into REALM input frames. Images can be
 * added dynamically, however the order accoding to the image names should be considered. Therefore DON'T add images in
 * wrong order. Additionally a pose file in TUM format can be provided to feed external pose data into the frame.
 * Exiv2 tags should at least contain the GNSS position. Tested name format is name_000000.suffix
 */

class Exiv2GrabberNode
{
  public:
    Exiv2GrabberNode();
    Frame::Ptr spin();
  private:

    bool _do_set_all_keyframes;

    double _fps;

    std::string _id_node;

    std::string _profile;
    std::string _topic_prefix;

    std::string _path_grab;
    std::string _path_working_directory;
    std::string _path_profile;

    std::string _file_settings_camera;

    io::Exiv2FrameReader _exiv2_reader;

    camera::Pinhole::Ptr _cam;

    size_t _id_curr_file;
    std::vector<std::string> _file_list;

    void readParams();
    void setPaths();
    std::vector<std::string> getFileList(const std::string& path);
};

} // namespace realm

#endif //PROJECT_GRABBER_EXIV_NODE
