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

#ifndef PROJECT_STAGE_NODE_H
#define PROJECT_STAGE_NODE_H

#include <mutex>
#include <unordered_map>
#include <string>

#include <OpenREALM/realm_core/structs.h>
#include <OpenREALM/realm_core/camera_settings_factory.h>
#include <OpenREALM/realm_vslam_base/visual_slam_settings_factory.h>
#include <OpenREALM/realm_densifier_base/densifier_settings_factory.h>
#include <OpenREALM/realm_io/utilities.h>

#include <OpenREALM/realm_stages/stage_settings_factory.h>
#include <OpenREALM/realm_stages/pose_estimation.h>
#include <OpenREALM/realm_stages/densification.h>
#include <OpenREALM/realm_stages/surface_generation.h>
#include <OpenREALM/realm_stages/ortho_rectification.h>
#include <OpenREALM/realm_stages/mosaicing.h>
#include <OpenREALM/realm_stages/tileing.h>

namespace realm
{

class StageNode 
{
  public:
    StageNode();
    ~StageNode();
    void spin(const Frame::Ptr &frame);
  private:
    // Set to true, to shut node down
    std::mutex _mutex_do_shutdown;
    bool _do_shutdown;

    // number of msgs that triggere ros CB
    uint32_t _nrof_msgs_rcvd;

    // camera info
    std::string _id_camera;

    // chosen profile related to all settings set
    std::string _profile;

    // chosen method related to the specific framework implementation
    std::string _method;

    // working paths
    std::string _path_working_directory;
    std::string _path_profile;
    std::string _path_output;

    // working directories
    std::string _dir_date_time;

    // filename of settings
    std::string _file_settings_stage;
    std::string _file_settings_method;
    std::string _file_settings_camera;
    std::string _file_settings_imu;

    // Settings of the stage
    CameraSettings::Ptr _settings_camera;

    // Handle for stage
    StageBase::Ptr pose_stage;
    StageBase::Ptr dense_stage;
    StageBase::Ptr surface_stage;
    StageBase::Ptr ortho_stage;
    StageBase::Ptr mosaic_stage;
    StageBase::Ptr tiling_stage;

    std::vector<StageBase::Ptr> all_stages;


    // Initialization
    StageSettings::Ptr readStageSettings(std::string type_stage);
    void setPaths();
    void createStagePoseEstimation(std::string type_stage);
    void createStageDensification(std::string type_stage);
    void createStageSurfaceGeneration(std::string type_stage);
    void createStageOrthoRectification(std::string type_stage);
    void createStageMosaicing(std::string type_stage);
    void createStageTileing(std::string type_stage);

    // Frame Transports
    void pubFramePoseToDense(const Frame::Ptr &frame, const std::string &topic);
    void pubFrameDenseToSurface(const Frame::Ptr &frame, const std::string &topic);
    void pubFrameSurfaceToOrtho(const Frame::Ptr &frame, const std::string &topic);
    void pubFrameOrthoToMosaic(const Frame::Ptr &frame, const std::string &topic);
    void pubFrameMosaicToTiling(const Frame::Ptr &frame, const std::string &topic);
    void pubFrameTilingToNone(const Frame::Ptr &frame, const std::string &topic);

    // Other message transports. Just dummy functions to pass onto the stages so they dont complain.
    void pubPose(const cv::Mat &pose, uint8_t zone, char band, const std::string &topic);
    void pubPointCloud(const cv::Mat &pts, const std::string &topic);
    void pubDepthMap(const cv::Mat &img, const std::string &topic);
    void pubImage(const cv::Mat &img, const std::string &topic);
    void pubMesh(const std::vector<Face> &faces, const std::string &topic);
    void pubCvGridMap(const CvGridMap &map, uint8_t zone, char band, const std::string &topic);

    void linkStageTransport();
};
} // namespace realm

#endif //PROJECT_STAGE_NODE_H
