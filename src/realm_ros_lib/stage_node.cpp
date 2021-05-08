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
#include <chrono>
#include <thread>

using namespace realm;

StageNode::StageNode()
: _nrof_msgs_rcvd(0),
  _do_shutdown(false)
{
  // Read basic launch file inputs
  setPaths();

  // Create stages
  createStagePoseEstimation("pose_estimation");
  createStageSurfaceGeneration("surface_generation");
  createStageOrthoRectification("ortho_rectification");
  createStageTileing("tileing");

  // Link the stage transport of all nodes.
  linkStageTransport();

  // add all stages to the vector
  all_stages.push_back(pose_stage);
  all_stages.push_back(surface_stage);
  all_stages.push_back(ortho_stage);
  all_stages.push_back(tiling_stage);
}

StageNode::~StageNode()
{
  // In case of an unproper shutdown, at least call the finish procedure
  if (!_do_shutdown)
  {
    for (size_t i = 0; i < all_stages.size(); i++)
    {
      StageBase::Ptr stage = all_stages[i];
      stage->requestFinish();
      stage->join();
    }
  }
}

StageSettings::Ptr StageNode::readStageSettings(std::string type_stage)
{
  std::string file_settings_stage = _path_profile + "/" + type_stage + "/stage_settings.yaml";
  StageSettings::Ptr settings_stage = StageSettingsFactory::load(type_stage, file_settings_stage);
  return settings_stage;
}

void StageNode::setPaths()
{
  //_path_output = "uninitialised";
  _profile = "alexa_noreco";
  _method = "uninitialised";
  _path_working_directory = ".";
  _id_camera = "cam";
  _path_profile = _path_working_directory + "/profiles/" + _profile;
  _path_output = _path_working_directory + "/output";
  _file_settings_camera = _path_profile + "/camera/calib.yaml";
  
  if (!io::dirExists(_path_profile))
    throw(std::runtime_error("Error: Profile folder '" + _path_profile + "' was not found!"));
  if (!io::dirExists(_path_output))
    io::createDir(_path_output);

  // Create sub directory with timestamp
  _dir_date_time = io::getDateTime();
  if (!io::dirExists(_path_output + "/" + _dir_date_time))
    io::createDir(_path_output + "/" + _dir_date_time);
  
  _settings_camera = CameraSettingsFactory::load(_file_settings_camera);
}

void StageNode::createStagePoseEstimation(std::string type_stage)
{
  StageSettings::Ptr settings_stage = readStageSettings(type_stage);

  ImuSettings::Ptr settings_imu = nullptr;
  if ((*settings_stage)["use_imu"].toInt() > 0)
  {
    settings_imu = std::make_shared<ImuSettings>();
    settings_imu->loadFromFile(_file_settings_imu);
  }
  
  std::string method("open_vslam"); // open_vslam,  orb_slam3
  std::string file_settings_method = _path_profile + "/" + type_stage + "/method/" + method + "_settings.yaml";
  // Pose estimation uses external frameworks, therefore load settings for that
  VisualSlamSettings::Ptr settings_vslam = VisualSlamSettingsFactory::load(file_settings_method, _path_profile + "/" + type_stage + "/method");

  // Topic and stage creation
  pose_stage = std::make_shared<stages::PoseEstimation>(settings_stage, settings_vslam, _settings_camera, settings_imu, (*_settings_camera)["fps"].toDouble());
  pose_stage->registerFrameTransport([=](const realm::Frame::Ptr & frame, const std::string &topic) {surface_stage->addFrame(std::move(frame));});
  
  if (!io::dirExists(_path_output + "/" + _dir_date_time))
      io::createDir(_path_output + "/" + _dir_date_time);
  pose_stage->initStagePath(_path_output + "/" + _dir_date_time);
  pose_stage->start();
}

void StageNode::createStageSurfaceGeneration(std::string type_stage)
{
  // 
  StageSettings::Ptr settings_stage = readStageSettings(type_stage);
  surface_stage = std::make_shared<stages::SurfaceGeneration>(settings_stage, (*_settings_camera)["fps"].toDouble());
  surface_stage->registerFrameTransport([=](const realm::Frame::Ptr & frame, const std::string &topic) {ortho_stage->addFrame(std::move(frame));});

  if (!io::dirExists(_path_output + "/" + _dir_date_time))
      io::createDir(_path_output + "/" + _dir_date_time);
  surface_stage->initStagePath(_path_output + "/" + _dir_date_time);
  surface_stage->start();
}

void StageNode::createStageOrthoRectification(std::string type_stage)
{
  // 
  StageSettings::Ptr settings_stage = readStageSettings(type_stage);
  ortho_stage = std::make_shared<stages::OrthoRectification>(settings_stage, (*_settings_camera)["fps"].toDouble());
  ortho_stage->registerFrameTransport([=](const realm::Frame::Ptr & frame, const std::string &topic) {tiling_stage->addFrame(std::move(frame));});
  if (!io::dirExists(_path_output + "/" + _dir_date_time))
      io::createDir(_path_output + "/" + _dir_date_time);
  ortho_stage->initStagePath(_path_output + "/" + _dir_date_time);
  ortho_stage->start();
}

void StageNode::createStageTileing(std::string type_stage)
{
  // 
  StageSettings::Ptr settings_stage = readStageSettings(type_stage);

  tiling_stage = std::make_shared<stages::Tileing>(settings_stage, (*_settings_camera)["fps"].toDouble());
  
  tiling_stage->registerFrameTransport([=](const realm::Frame::Ptr & frame, const std::string &topic) {std::cout << "Hi" << std::endl;});

  if (!io::dirExists(_path_output + "/" + _dir_date_time))
      io::createDir(_path_output + "/" + _dir_date_time);
  tiling_stage->initStagePath(_path_output + "/" + _dir_date_time);
  tiling_stage->start();
}

void StageNode::spin(const Frame::Ptr &frame)
{
  //pose_stage->addFrame(frame);
  //_nrof_msgs_rcvd++;
}

void StageNode::linkStageTransport()
{  
  // Start the thread for processing
  for (size_t i = 0; i < all_stages.size(); i++)
  {
    StageBase::Ptr stage = all_stages[i];

    stage->registerPoseTransport([=](const cv::Mat &pose, uint8_t zone, char band, const std::string &topic) {std::cout << topic << std::endl;});
    stage->registerPointCloudTransport([=](const cv::Mat &pts, const std::string &topic) {std::cout << topic << std::endl;});
    stage->registerImageTransport([=](const cv::Mat &img, const std::string &topic) {std::cout << topic << std::endl;});
    stage->registerDepthMapTransport([=](const cv::Mat &img, const std::string &topic) {std::cout << topic << std::endl;});
    stage->registerMeshTransport([=](const std::vector<Face> &faces, const std::string &topic) {std::cout << topic << std::endl;});
    stage->registerCvGridMapTransport([=](const CvGridMap &map, uint8_t zone, char band, const std::string &topic) {std::cout << topic << std::endl;});
  }
}

