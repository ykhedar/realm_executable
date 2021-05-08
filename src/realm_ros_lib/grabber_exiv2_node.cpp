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

#include <realm_ros/grabber_exiv2_node.h>

using namespace realm;

Exiv2GrabberNode::Exiv2GrabberNode()
: _do_set_all_keyframes(false),
  _fps(0.0),
  _id_curr_file(0),
  _cam(nullptr)
{
  readParams();
  setPaths();

  _exiv2_reader = io::Exiv2FrameReader(io::Exiv2FrameReader::FrameTags::loadFromFile(_path_profile + "/config/exif.yaml"));

  if (io::fileExists(_file_settings_camera))
    _cam = std::make_shared<camera::Pinhole>(io::loadCameraFromYaml(_file_settings_camera));
  else
    throw(std::invalid_argument("Error loading camera file: Provided path does not exist."));

  std::cout << 
      "Exiv2 Grabber Node: Successfully loaded camera: "
          << "\n\tcx = " << _cam->cx()
          << "\n\tcy = " << _cam->cy()
          << "\n\tfx = " << _cam->fx()
          << "\n\tfy = " << _cam->fy()
          << "\n\tk1 = " << _cam->k1()
          << "\n\tk2 = " << _cam->k2()
          << "\n\tp1 = " << _cam->p1()
          << "\n\tp2 = " << _cam->p2() << std::endl;

  // ROS related inits
  _topic_prefix = "/realm/" + _id_node;


  // Start grabbing images
  std::cout << "Exiv2 Grabber Node: Getting file list " << std::endl;
  _file_list = getFileList(_path_grab);

  // Check if the exif tags in the config exist
  std::cout << "Scanning input image for provided meta tags..." << std::endl;
  std::map<std::string, bool> tag_existence = _exiv2_reader.probeImage(_file_list[0]);
  for (const auto &tag : tag_existence)
  {
    if (tag.second)
      std::cout << "[FOUND]\t\t" << tag.first.c_str() << std::endl;
    else
      std::cout << "[NOT FOUND]\t" << tag.first.c_str() << std::endl;
  }
}

void Exiv2GrabberNode::readParams()
{
  _id_node = "exiv";
  _path_grab = "/home/shifty21/edm_big_overlap_50p";
  _fps = 10.0;
  _profile = "alexa_noreco";
  _path_working_directory = ".";

  if (_fps < 0.01)
    throw(std::invalid_argument("Error reading exiv2 grabber parameters: Frame rate is too low!"));
  if (_path_working_directory != "uninitialised" && !io::dirExists(_path_working_directory))
    throw(std::invalid_argument("Error: Working directory does not exist!"));
}

void Exiv2GrabberNode::setPaths()
{
  if (_path_working_directory == "uninitialised")
    _path_working_directory = ".";
  _path_profile = _path_working_directory + "/profiles/" + _profile;

  _file_settings_camera = _path_profile + "/camera/calib.yaml";

  if (!io::dirExists(_path_profile))
    throw(std::invalid_argument("Error: Config folder path '" + _path_profile + "' does not exist!"));
  if (!io::dirExists(_path_grab))
    throw(std::invalid_argument("Error grabbing Exiv2 images: Folder '" + _path_grab + "' does not exist!"));
}

std::vector<std::string> Exiv2GrabberNode::getFileList(const std::string& path)
{
  std::vector<std::string> file_names;
  if (!path.empty())
  {
    boost::filesystem::path apk_path(path);
    boost::filesystem::recursive_directory_iterator end;

    for (boost::filesystem::recursive_directory_iterator it(apk_path); it != end; ++it)
    {
      const boost::filesystem::path cp = (*it);
      file_names.push_back(cp.string());
    }
  }
  std::sort(file_names.begin(), file_names.end());
  return file_names;
}

Frame::Ptr Exiv2GrabberNode::spin()
{
  if (_id_curr_file < _file_list.size())
  {
    std::cout << "Image #" << _id_curr_file << ", image Path: " << _file_list[_id_curr_file] << std::endl;
    Frame::Ptr frame = _exiv2_reader.loadFrameFromExiv2(_id_node, _cam, _file_list[_id_curr_file]);

    _id_curr_file++;
    return frame;
  }

  std::vector<std::string> new_file_list = getFileList(_path_grab);
  if (new_file_list.size() != _file_list.size())
  {
    std::cout << "Processed images in folder: " << _file_list.size() << " / " << new_file_list.size() << std::endl;
    std::set_difference(new_file_list.begin(), new_file_list.end(), _file_list.begin(), _file_list.end(), std::back_inserter(_file_list));
    std::sort(_file_list.begin(), _file_list.end());
  }
  return nullptr;
}