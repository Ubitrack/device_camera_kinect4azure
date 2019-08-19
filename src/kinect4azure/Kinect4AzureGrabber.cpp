/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Synchronous capture of camera images using Intel's kinect4azure library.
 *
 * @author Ulrich Eck <ulrich.eck@tum.de>
 *
 */

#include "Kinect4AzureFrameGrabber.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <utDataflow/ComponentFactory.h>
#include <utUtil/OS.h>
#include <utUtil/CalibFile.h>
#include <boost/array.hpp>

#include <opencv2/opencv.hpp>

#include <utVision/Util/OpenCV.h>

#include <log4cpp/Category.hh>

// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.device_camera_kinect4azure.AzureKinectFrameGrabber" ) );

using namespace Ubitrack;
using namespace Ubitrack::Vision;
using namespace Ubitrack::Drivers;

bool profile_contains_stream(const std::map<std::string, rs2::stream_profile>& map, const std::string& key) {
    return (map.find(key) != map.end());
}

bool get_intrinsics_for_stream(std::map<std::string, rs2::stream_profile>& map, const std::string& key, Math::CameraIntrinsics<double>& value) {
    if (profile_contains_stream(map, key)) {
        auto& stream_profile = map[key];
        if (auto vp = stream_profile.as<rs2::video_stream_profile>()) {
            try {
                rs2_intrinsics intr = vp.get_intrinsics();

                Math::Matrix< double, 3, 3 > intrinsicMatrix = Math::Matrix3x3d::identity();
                intrinsicMatrix(0, 0) = intr.fx;
                intrinsicMatrix(1, 1) = intr.fy;
                intrinsicMatrix(0, 2) = -intr.ppx;
                intrinsicMatrix(1, 2) = -intr.ppy;
                intrinsicMatrix(2, 2) = -1.0;

                // [ k1, k2, p1, p2, k3 ]
                Math::Vector< double, 3 > radial(intr.coeffs[0],
                                                 intr.coeffs[1],
                                                 intr.coeffs[4]);
                Math::Vector< double, 2 > tangential(intr.coeffs[2],
                                                     intr.coeffs[3]);
                auto width = (std::size_t)intr.width;
                auto height = (std::size_t)intr.height;

                value = Math::CameraIntrinsics<double>(intrinsicMatrix, radial, tangential, width, height);

                // real sense has image origin of 0, ubitrack default is 1, flip needed of cy and tangential parameter needed
                value = Vision::Util::cv2::correctForOrigin(0, value);

                LOG4CPP_DEBUG(logger, key << " Camera Model: " << value);
                return true;
            } catch( rs2::error& e) {
                LOG4CPP_ERROR(logger, e.what());
            }
        }
    }
    value = Math::CameraIntrinsics<double>();
    return false;
}

bool get_pose_between_streams(std::map<std::string, rs2::stream_profile>& map, const std::string& source,
                              const std::string& destination, Math::Pose& value) {
    if ((profile_contains_stream(map, source)) && (profile_contains_stream(map, destination))) {
        auto& left_stream_profile = map[source];
        auto& color_stream_profile = map[destination];
        try {
            auto left2color = left_stream_profile.get_extrinsics_to(color_stream_profile);
            auto rot_mat = Math::Matrix3x3d::identity();

            // libkinect4azure and ubitrack store matrices column-major
            rot_mat( 0, 0 ) = left2color.rotation[0];
            rot_mat( 1, 0 ) = left2color.rotation[3];
            rot_mat( 2, 0 ) = left2color.rotation[6];

            rot_mat( 0, 1 ) = left2color.rotation[1];
            rot_mat( 1, 1 ) = left2color.rotation[4];
            rot_mat( 2, 1 ) = left2color.rotation[7];

            rot_mat( 0, 2 ) = left2color.rotation[2];
            rot_mat( 1, 2 ) = left2color.rotation[5];
            rot_mat( 2, 2 ) = left2color.rotation[8];

            Math::Quaternion ut_quat(rot_mat);

            Math::Vector3d ut_trans(
                    (double)left2color.translation[0],
                    (double)left2color.translation[1],
                    (double)left2color.translation[2]
            );
            value = Math::Pose(ut_quat, ut_trans);
            LOG4CPP_DEBUG(logger, "IR Left2Color Transform: " << value);
            return true;
        } catch (rs2::error& e) {
            LOG4CPP_ERROR(logger, e.what());
        }
    }
    value = Math::Pose();
    return false;
}

Ubitrack::Vision::Image::ImageFormatProperties getImageFormatPropertiesFromRS2Frame(const rs2::frame& f) {
    auto imageFormatProperties = Vision::Image::ImageFormatProperties();
    switch (f.get_profile().format()) {
        case RS2_FORMAT_BGRA8:
            imageFormatProperties.depth = CV_8U;
            imageFormatProperties.channels = 4;
            imageFormatProperties.matType = CV_8UC4;
            imageFormatProperties.bitsPerPixel = 32;
            imageFormatProperties.origin = 0;
            imageFormatProperties.imageFormat = Vision::Image::BGRA;
            break;

        case RS2_FORMAT_BGR8:
            imageFormatProperties.depth = CV_8U;
            imageFormatProperties.channels = 3;
            imageFormatProperties.matType = CV_8UC3;
            imageFormatProperties.bitsPerPixel = 24;
            imageFormatProperties.origin = 0;
            imageFormatProperties.imageFormat = Vision::Image::BGR;
            break;

        case RS2_FORMAT_RGBA8:
            imageFormatProperties.depth = CV_8U;
            imageFormatProperties.channels = 4;
            imageFormatProperties.matType = CV_8UC4;
            imageFormatProperties.bitsPerPixel = 32;
            imageFormatProperties.origin = 0;
            imageFormatProperties.imageFormat = Vision::Image::RGBA;
            break;

        case RS2_FORMAT_RGB8:
            imageFormatProperties.depth = CV_8U;
            imageFormatProperties.channels = 3;
            imageFormatProperties.matType = CV_8UC3;
            imageFormatProperties.bitsPerPixel = 24;
            imageFormatProperties.origin = 0;
            imageFormatProperties.imageFormat = Vision::Image::RGB;
            break;

        case RS2_FORMAT_Y16:
            imageFormatProperties.depth = CV_16U;
            imageFormatProperties.channels = 1;
            imageFormatProperties.matType = CV_16UC1;
            imageFormatProperties.bitsPerPixel = 16;
            imageFormatProperties.origin = 0;
            imageFormatProperties.imageFormat = Vision::Image::LUMINANCE;
            break;

        case RS2_FORMAT_Y8:
            imageFormatProperties.depth = CV_8U;
            imageFormatProperties.channels = 1;
            imageFormatProperties.matType = CV_8UC1;
            imageFormatProperties.bitsPerPixel = 8;
            imageFormatProperties.origin = 0;
            imageFormatProperties.imageFormat = Vision::Image::LUMINANCE;
            break;

        case RS2_FORMAT_Z16:
            imageFormatProperties.depth = CV_16U;
            imageFormatProperties.channels = 1;
            imageFormatProperties.matType = CV_16UC1;
            imageFormatProperties.bitsPerPixel = 16;
            imageFormatProperties.origin = 0;
            imageFormatProperties.imageFormat = Vision::Image::DEPTH;
            break;

        default:
            UBITRACK_THROW("kinect4azure frame format is not supported!");
    }
    return imageFormatProperties;
}


namespace Ubitrack { namespace Drivers {

    AzureKinectCameraComponent::AzureKinectCameraComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph  )
        : Dataflow::Component( sName )
        , m_outputColorImagePort("ColorImageOutput", *this)
        , m_outputGreyImagePort("GreyImageOutput", *this)
        , m_outputIRImagePort("IRImageOutput", *this)
        , m_outputDepthMapImagePort("DepthImageOutput", *this)
        , m_outputPointCloudPort("PointCloudOutput", *this)
        , m_outputColorCameraModelPort("ColorCameraModel", *this, boost::bind(&AzureKinectCameraComponent::getColorCameraModel, this, _1))
        , m_outputColorIntrinsicsMatrixPort("ColorIntrinsics", *this, boost::bind(&AzureKinectCameraComponent::getColorIntrinsic, this, _1))
        , m_outputDepthCameraModelPort("DepthCameraModel", *this, boost::bind(&AzureKinectCameraComponent::getDepthCameraModel, this, _1))
        , m_outputDepthIntrinsicsMatrixPort("DepthtIntrinsics", *this, boost::bind(&AzureKinectCameraComponent::getDepthIntrinsic, this, _1))
        , m_depthToColorTransformPort("DepthToColorTransform", *this, boost::bind(&AzureKinectCameraComponent::getDepthToColorTransform, this, _1))

		, m_hwsync_mode(K4A_WIRED_SYNC_MODE_STANDALONE)
		, m_colorImageFormat(K4A_IMAGE_FORMAT_COLOR_BGRA32)
		, m_depthImageFormat(K4A_IMAGE_FORMAT_IR16)
		, m_colorResolution(K4A_COLOR_RESOLUTION_720P)
		, m_depthMode(K4A_DEPTH_MODE_NFOV_2X2BINNED)
		, m_frameRate(K4A_FRAMES_PER_SECOND_30)
        , m_serialNumber("")
        //, m_depthLaserPower(150)
        //, m_depthEmitterEnabled(1)
        //, m_infraredGain(16)
        , m_autoGPUUpload(false)
    {

        if (subgraph->m_DataflowAttributes.hasAttribute("rsSerialNumber")) {
            m_serialNumber = subgraph->m_DataflowAttributes.getAttributeString("rsSerialNumber");
        }

        if ( subgraph->m_DataflowAttributes.hasAttribute( "rsColorVideoResolution" ) )
        {
            std::string sResolution = subgraph->m_DataflowAttributes.getAttributeString( "rsColorVideoResolution" );
            if ( kinect4azureStreamResolutionMap.find( sResolution ) == kinect4azureStreamResolutionMap.end() )
                UBITRACK_THROW( "unknown stream resolution: \"" + sResolution + "\"" );
            std::tuple<unsigned int, unsigned int> resolution = kinect4azureStreamResolutionMap[ sResolution ];
            m_colorImageWidth = std::get<0>(resolution);
            m_colorImageHeight = std::get<1>(resolution);
        }
        if ( subgraph->m_DataflowAttributes.hasAttribute( "rsColorVideoStreamFormat" ) )
        {
            std::string sStreamFormat = subgraph->m_DataflowAttributes.getAttributeString( "rsColorVideoStreamFormat" );
            if ( kinect4azureStreamFormatMap.find( sStreamFormat ) == kinect4azureStreamFormatMap.end() )
                UBITRACK_THROW( "unknown stream type: \"" + sStreamFormat + "\"" );
            m_colorStreamFormat = kinect4azureStreamFormatMap[ sStreamFormat ];
        }

        if ( subgraph->m_DataflowAttributes.hasAttribute( "rsInfraredVideoStreamFormat" ) )
        {
            std::string sStreamFormat = subgraph->m_DataflowAttributes.getAttributeString( "rsInfraredVideoStreamFormat" );
            if ( kinect4azureStreamFormatMap.find( sStreamFormat ) == kinect4azureStreamFormatMap.end() )
                UBITRACK_THROW( "unknown stream type: \"" + sStreamFormat + "\"" );
            m_infraredStreamFormat = kinect4azureStreamFormatMap[ sStreamFormat ];
        }

        if ( subgraph->m_DataflowAttributes.hasAttribute( "rsDepthResolution" ) )
        {
            std::string sResolution = subgraph->m_DataflowAttributes.getAttributeString( "rsDepthResolution" );
            if ( kinect4azureStreamResolutionMap.find( sResolution ) == kinect4azureStreamResolutionMap.end() )
                UBITRACK_THROW( "unknown stream type: \"" + sResolution + "\"" );
            std::tuple<unsigned int, unsigned int> resolution = kinect4azureStreamResolutionMap[ sResolution ];
            m_depthImageWidth = std::get<0>(resolution);
            m_depthImageHeight = std::get<1>(resolution);
        }

        if ( subgraph->m_DataflowAttributes.hasAttribute( "rsOperationMode" ) )
        {
            std::string sOperationMode = subgraph->m_DataflowAttributes.getAttributeString( "rsOperationMode" );
            if ( kinect4azureOperationModeMap.find( sOperationMode ) == kinect4azureOperationModeMap.end() )
                UBITRACK_THROW( "unknown operation mode: \"" + sOperationMode + "\"" );
            m_operation_mode = kinect4azureOperationModeMap[ sOperationMode ];
        }


        if (subgraph->m_DataflowAttributes.hasAttribute("rsRosBagFilename")){
            boost::filesystem::path m_rosbag_filename_expanded = Ubitrack::Util::getFilesystemPath(subgraph->m_DataflowAttributes.getAttributeString("rsRosBagFilename"));
            m_rosbag_filename = m_rosbag_filename_expanded.string();
        }
        if (subgraph->m_DataflowAttributes.hasAttribute("rsTimestampFilename")){
            boost::filesystem::path m_timestamp_filename_expanded = Ubitrack::Util::getFilesystemPath(subgraph->m_DataflowAttributes.getAttributeString("rsTimestampFilename"));
            m_timestamp_filename = m_timestamp_filename_expanded.string();
        }
        if (subgraph->m_DataflowAttributes.hasAttribute("rsCameraModelLeftFilename")){
            boost::filesystem::path m_cameramodel_left_filename_expanded = Ubitrack::Util::getFilesystemPath(subgraph->m_DataflowAttributes.getAttributeString("rsCameraModelLeftFilename"));
            m_cameramodel_left_filename = m_cameramodel_left_filename_expanded.string();
        }
        if (subgraph->m_DataflowAttributes.hasAttribute("rsCameraModelColorFilename")){
            boost::filesystem::path m_cameramodel_color_filename_expanded = Ubitrack::Util::getFilesystemPath(subgraph->m_DataflowAttributes.getAttributeString("rsCameraModelColorFilename"));
            m_cameramodel_color_filename = m_cameramodel_color_filename_expanded.string();
        }
        if (subgraph->m_DataflowAttributes.hasAttribute("rsDepth2ColorFilename")){
            boost::filesystem::path m_depth2color_filename_expanded = Ubitrack::Util::getFilesystemPath(subgraph->m_DataflowAttributes.getAttributeString("rsDepth2ColorFilename"));
            m_depth2color_filename = m_depth2color_filename_expanded.string();
        }

        if ( subgraph->m_DataflowAttributes.hasAttribute( "rsHardwareSync" ) )
        {
            std::string sHardwareSync = subgraph->m_DataflowAttributes.getAttributeString( "rsHardwareSync" );
            if ( kinect4azureHWSyndModeMap.find( sHardwareSync ) == kinect4azureHWSyndModeMap.end() )
                UBITRACK_THROW( "unknown hardware sync mode: \"" + sHardwareSync + "\"" );
            m_hwsync_mode = kinect4azureHWSyndModeMap[ sHardwareSync ];
        }

        subgraph->m_DataflowAttributes.getAttributeData( "rsFrameRate", m_frameRate );

        subgraph->m_DataflowAttributes.getAttributeData( "rsLaserPower", m_depthLaserPower);
        subgraph->m_DataflowAttributes.getAttributeData( "rsEmitterEnabled", m_depthEmitterEnabled);
        subgraph->m_DataflowAttributes.getAttributeData( "rsInfraredGain", m_infraredGain);

        // the following is an attempt to make the component configurable through the associated pattern.
        // if certain edges don't exist in the pattern, the streams will not be requested from the camera.
        if (subgraph->hasEdge("ColorImageOutput")) {
            LOG4CPP_INFO(logger, "Activate Color Stream.");
            m_stream_requests.push_back(
                    {rs2_stream::RS2_STREAM_COLOR, m_colorStreamFormat, m_colorImageWidth,
                     m_colorImageHeight, m_frameRate, 0, "ColorImageOutput"});
            m_haveColorStream = true;
        }

        if (subgraph->hasEdge("IRLeftImageOutput")) {
            LOG4CPP_INFO(logger, "Activate Left Infrared Stream.");
            m_stream_requests.push_back(
                    {rs2_stream::RS2_STREAM_INFRARED, m_infraredStreamFormat, m_depthImageWidth,
                     m_depthImageHeight, m_frameRate, 1, "IRLeftImageOutput"});
            m_haveIRLeftStream = true;
        }

//        if (subgraph->hasEdge("IRRightImageOutput")) {
//            LOG4CPP_INFO(logger, "Activate Right Infrared Stream.");
//            m_stream_requests.push_back(
//                    {rs2_stream::RS2_STREAM_INFRARED, m_infraredStreamFormat, m_depthImageWidth,
//                     m_depthImageHeight, m_frameRate, 2, "IRRightImageOutput"});
//            m_haveIRRightStream = true;
//        }
//
        if (subgraph->hasEdge("DepthImageOutput") || subgraph->hasEdge("PointCloudOutput")) {
            LOG4CPP_INFO(logger, "Activate Depth Stream.");
            m_stream_requests.push_back(
                    {rs2_stream::RS2_STREAM_DEPTH, rs2_format::RS2_FORMAT_Z16, m_depthImageWidth,
                     m_depthImageHeight, m_frameRate, 0, "DepthImageOutput"});
            m_haveDepthStream = true;
        }


        Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
        if (oclManager.isEnabled()) {
            if (subgraph->m_DataflowAttributes.hasAttribute("uploadImageOnGPU")){
                m_autoGPUUpload = subgraph->m_DataflowAttributes.getAttributeString("uploadImageOnGPU") == "true";
            }
            if (m_autoGPUUpload){
                oclManager.activate();
            }
        }
    }

    void AzureKinectCameraComponent::start()
    {
        if ( !m_running )
        {
            // check if oclmanager is active
            Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
            if ((oclManager.isEnabled()) && (oclManager.isActive()) && (!oclManager.isInitialized())) {
                LOG4CPP_INFO(logger, "Waiting for OpenCLManager Initialization callback.");
                oclManager.registerInitCallback(boost::bind(&AzureKinectCameraComponent::startCapturing, this));
            } else {
                startCapturing();
            }
            m_running = true;
        }
        Component::start();
    }

    void AzureKinectCameraComponent::setupDevice()
    {
        m_pipeline_config = rs2::config();
        m_pipeline = std::make_shared<rs2::pipeline>(m_ctx);

        switch(m_operation_mode) {
            case OPERATION_MODE_LIVESTREAM:
                break;

            case OPERATION_MODE_LIVESTREAM_RECORD:
                m_pipeline_config.enable_record_to_file(m_rosbag_filename.string());
                break;

            case OPERATION_MODE_PLAYBACK:
                m_pipeline_config.enable_device_from_file(m_rosbag_filename.string());
                m_pipeline_config.enable_all_streams();
                break;
        }

        if (m_operation_mode != OPERATION_MODE_PLAYBACK) {
            // if serialNumber != 0, ask for it and "reserve" it.
            if (m_serialNumber != "") {
                LOG4CPP_INFO(logger, "Require kinect4azure camera with serialnumber: " << m_serialNumber);
                m_pipeline_config.enable_device(m_serialNumber);
            }

            // only configure streams when accessing the sensor
            for (auto i = 0; i < m_stream_requests.size(); i++) {
                LOG4CPP_DEBUG(logger, "Enable Stream: " << m_stream_requests[i]._port_name << " Type: " << m_stream_requests[i]._stream_type
                << " Idx: " << m_stream_requests[i]._stream_idx << " Format: " << m_stream_requests[i]._stream_format << " ("
                << m_stream_requests[i]._width << "x" << m_stream_requests[i]._height << ")");

                m_pipeline_config.enable_stream(
                        m_stream_requests[i]._stream_type,
                        m_stream_requests[i]._stream_idx,
                        m_stream_requests[i]._width,
                        m_stream_requests[i]._height,
                        m_stream_requests[i]._stream_format,
                        m_stream_requests[i]._fps
                );
            }
        }

        try {
            Ubitrack::Util::sleep(2000);
            m_pipeline_profile = m_pipeline_config.resolve(*m_pipeline);
        } catch(rs2::error &e) {
            LOG4CPP_ERROR(logger, "Error while starting pipeline: " << e.what());
            UBITRACK_THROW("Cannot setup kinect4azure device");
        }

        // check sensor for compatible config and build map
        bool succeed = false;
        size_t expected_number_of_streams = m_stream_requests.size();
        m_stream_profile_map.clear();

        for (auto&& sensor : m_pipeline_profile.get_device().query_sensors()) {
            for (auto &profile : sensor.get_stream_profiles()) {
                // All requests have been resolved
                if (m_stream_requests.empty())
                    break;

                // Find profile matches
                auto fulfilled_request = std::find_if(m_stream_requests.begin(), m_stream_requests.end(),
                                                      [profile, this](const stream_request &req) {
                                                          bool res = false;
                                                          if ((profile.stream_type() == req._stream_type) &&
                                                              (profile.format() == req._stream_format) &&
                                                              (profile.stream_index() == req._stream_idx) &&
                                                              (profile.fps() == req._fps)) {
                                                              if (auto vp = profile.as<rs2::video_stream_profile>()) {
                                                                  if ((vp.width() != req._width) ||
                                                                      (vp.height() != req._height))
                                                                      return false;
                                                              }
                                                              res = true;
                                                              m_stream_profile_map[req._port_name] = profile;
                                                              LOG4CPP_DEBUG(logger, "kinect4azure camera streamprofile found for: " << req._port_name);
                                                          }

                                                          return res;
                                                      });

                // Remove the request once resolved
                if (fulfilled_request != m_stream_requests.end()) {
                    m_stream_requests.erase(fulfilled_request);
                }
            }
        }

        if (m_selected_stream_profiles.size() == expected_number_of_streams) {
            LOG4CPP_INFO(logger, "Found matching kinect4azure device.");
        } else {
            LOG4CPP_WARN(logger, "Not all stream requests could be satisfied !!!");
        }
    }

    void AzureKinectCameraComponent::retrieveCalibration() {

        if (m_haveColorStream) {
            std::string portname = "ColorImageOutput";
            if (m_operation_mode == OPERATION_MODE_PLAYBACK) {
                // we're in playback mode, read calibfile
                Ubitrack::Util::readCalibFile(m_cameramodel_color_filename.string(), m_colorCameraModel);
            } else {
                if (!get_intrinsics_for_stream(m_stream_profile_map, portname, m_colorCameraModel)) {
                    LOG4CPP_WARN(logger, "Intrinsics not found for " << portname);
                } else if (m_operation_mode == OPERATION_MODE_LIVESTREAM_RECORD) {
                    // we got a calibration from the camera and are in record mode, so write it.
                    Ubitrack::Util::writeCalibFile(m_cameramodel_color_filename.string(), m_colorCameraModel);
                }
            }
        }
        if (m_haveDepthStream) {
            std::string portname = "DepthImageOutput";
            if (m_operation_mode == OPERATION_MODE_PLAYBACK) {
                // we're in playback mode, read calibfile
                Ubitrack::Util::readCalibFile(m_cameramodel_left_filename.string(), m_infraredLeftCameraModel);
            } else {
                if (!get_intrinsics_for_stream(m_stream_profile_map, portname, m_infraredLeftCameraModel)) {
                    LOG4CPP_WARN(logger, "Intrinsics not found for " << portname);
                } else if (m_operation_mode == OPERATION_MODE_LIVESTREAM_RECORD) {
                    // we got a calibration from the camera and are in record mode, so write it.
                    Ubitrack::Util::writeCalibFile(m_cameramodel_left_filename.string(), m_infraredLeftCameraModel);
                }
            }
        } else if (m_haveIRLeftStream) {
            std::string portname = "IRLeftImageOutput";
            if (m_operation_mode == OPERATION_MODE_PLAYBACK) {
                // we're in playback mode, read calibfile
                Ubitrack::Util::readCalibFile(m_cameramodel_left_filename.string(), m_infraredLeftCameraModel);
            } else {
                if (!get_intrinsics_for_stream(m_stream_profile_map, portname, m_infraredLeftCameraModel)) {
                    LOG4CPP_WARN(logger, "Intrinsics not found for " << portname);
                } else if (m_operation_mode == OPERATION_MODE_LIVESTREAM_RECORD) {
                    // we got a calibration from the camera and are in record mode, so write it.
                    Ubitrack::Util::writeCalibFile(m_cameramodel_left_filename.string(), m_infraredLeftCameraModel);
                }
            }
        }

//        if (m_haveIRRightStream) {
//            std::string portname = "IRRightImageOutput";
//            if (m_operation_mode == OPERATION_MODE_PLAYBACK) {
//                // we're in playback mode, read calibfile
//                Ubitrack::Util::readCalibFile(m_cameramodel_right_filename.string(), m_infraredRightCameraModel);
//            } else {
//                if (!get_intrinsics_for_stream(m_stream_profile_map, portname, m_infraredRightCameraModel)) {
//                    LOG4CPP_WARN(logger, "Intrinsics not found for " << portname);
//                } else if (m_operation_mode == OPERATION_MODE_LIVESTREAM_RECORD) {
//                    // we got a calibration from the camera and are in record mode, so write it.
//                    Ubitrack::Util::writeCalibFile(m_cameramodel_right_filename.string(), m_infraredRightCameraModel);
//                }
//            }
//        }

//        if (m_haveIRRightStream) {
//            if (m_haveIRLeftStream) {
//                get_pose_between_streams(m_stream_profile_map, "IRLeftImageOutput", "IRRightImageOutput", m_leftToRightTransform);
//            } else if (m_haveDepthStream) {
//                get_pose_between_streams(m_stream_profile_map, "DepthImageOutput", "IRRightImageOutput", m_leftToRightTransform);
//            }
//        }
//
        if (m_haveColorStream) {

            std::string portname;
            if (m_haveDepthStream) {
                portname = "DepthImageOutput";
            } else if (m_haveIRLeftStream) {
                portname = "IRLeftImageOutput";
            }

            if (m_operation_mode == OPERATION_MODE_PLAYBACK) {
                // we're in playback mode, read calibfile
                Ubitrack::Util::readCalibFile(m_depth2color_filename.string(), m_leftToColorTransform);
            } else {
                if (!get_pose_between_streams(m_stream_profile_map, portname, "ColorImageOutput", m_leftToColorTransform)) {
                    LOG4CPP_WARN(logger, "IR Left2Color Transform cannot be determined: " << portname);
                } else if (m_operation_mode == OPERATION_MODE_LIVESTREAM_RECORD) {
                    // we got a calibration from the camera and are in record mode, so write it.
                    Ubitrack::Util::writeCalibFile(m_depth2color_filename.string(), m_leftToColorTransform);
                }
            }
        }

        if (m_haveDepthStream) {
            if (auto dpt_sensor = m_pipeline_profile.get_device().first<rs2::depth_sensor>()) {
                // retrieve depth scaling from sensor like this:
                //A Depth stream contains an image that is composed of pixels with depth information.
                //The value of each pixel is the distance from the camera, in some distance units.
                //To get the distance in units of meters, each pixel's value should be multiplied by the sensor's depth scale
                //Here is the way to grab this scale value for a "depth" sensor:
                float scale = dpt_sensor.get_depth_scale();
                LOG4CPP_INFO(logger, "Scale factor for the kinect4azure depth sensor is: " << scale);
            }
        }
    }

    void AzureKinectCameraComponent::setOptions() {
        /** D435 Options
            setting options works on sensors not on streams.
            not sure how to implement this in a useful way

            Options for Stereo Module
             Supported options:                                    min        max       step  default
                Exposure                                           : 20   ... 166000      20    8500
                Gain                                               : 16   ... 248         1     16
                Enable Auto Exposure                               : 0    ... 1           1     1
                Visual Preset                                      : 0    ... 6           1     0
                Laser Power                                        : 0    ... 360         30    150
                Emitter Enabled                                    : 0    ... 2           1     1
                Frames Queue Size                                  : 0    ... 32          1     16
                Error Polling Enabled                              : 0    ... 1           1     0
                Output Trigger Enabled                             : 0    ... 1           1     0
                Depth Units                                        : 0.0001... 0.01        1e-06 0.001
                Stereo Baseline                                    : 49.9954... 49.9954     0     49.9954

            Options for RGB Camera
             Supported options:                                    min        max       step  default
                Backlight Compensation                             : 0    ... 1           1     0
                Brightness                                         : -64  ... 64          1     0
                Contrast                                           : 0    ... 100         1     50
                Exposure                                           : 41   ... 10000       1     166
                Gain                                               : 0    ... 128         1     64
                Gamma                                              : 100  ... 500         1     300
                Hue                                                : -180 ... 180         1     0
                Saturation                                         : 0    ... 100         1     64
                Sharpness                                          : 0    ... 100         1     50
                White Balance                                      : 2800 ... 6500        10    4600
                Enable Auto Exposure                               : 0    ... 1           1     1
                Enable Auto White Balance                          : 0    ... 1           1     1
                Frames Queue Size                                  : 0    ... 32          1     16
                Power Line Frequency                               : 0    ... 2           1     3
                Auto Exposure Priority                             : 0    ... 1           1     0
         */

        for (auto&& sensor : m_pipeline_profile.get_device().query_sensors()) {
            if (rs2::depth_sensor dpt_sensor = sensor.as<rs2::depth_sensor>())
            {
                // depth sensor options
                dpt_sensor.set_option(rs2_option::RS2_OPTION_LASER_POWER, m_depthLaserPower);
                dpt_sensor.set_option(rs2_option::RS2_OPTION_EMITTER_ENABLED, m_depthEmitterEnabled);
                dpt_sensor.set_option(rs2_option::RS2_OPTION_GAIN, m_infraredGain);

                // set hw sync mode
                dpt_sensor.set_option(rs2_option::RS2_OPTION_INTER_CAM_SYNC_MODE, m_hwsync_mode);
            } else {
                // color sensor options ?
            }
        }

    }

    void AzureKinectCameraComponent::startCapturing() {

        setupDevice();
        retrieveCalibration();
        if (m_operation_mode != OPERATION_MODE_PLAYBACK) {
            setOptions();
        }

        if (m_operation_mode == OPERATION_MODE_LIVESTREAM_RECORD) {
            LOG4CPP_DEBUG(logger, "Open Timestamp Filename:" << m_timestamp_filename.string());
            m_timestamp_filebuffer.open ( m_timestamp_filename.string().c_str(), std::ios::out );
            if( !m_timestamp_filebuffer.is_open()  ) {
                UBITRACK_THROW("Error opening timestamp file" );
            }
        }

        // Start streaming
        m_pipeline->start(m_pipeline_config, [this](rs2::frame f)
        {
            handleFrame(f);
        });

    }

    void AzureKinectCameraComponent::handleFrame(rs2::frame frame) {

        // convert from frame timestamp (milliseconds, double) to Measurement::Timestamp (nanoseconds, unsigned long long)
        auto ts = (Measurement::Timestamp)(frame.get_timestamp() * 1000000);

        // write the corresponding timestamp to a file
        std::ostream os( &m_timestamp_filebuffer );
        os << ts << " " << frame.get_frame_number() << std::endl;

        if (rs2::frameset fs = frame.as<rs2::frameset>())
        {
            // With callbacks, all synchronized stream will arrive in a single frameset
            for (auto&& f : fs) {
                rs2_stream stream_type = f.get_profile().stream_type();
                int stream_index = f.get_profile().stream_index();

                LOG4CPP_TRACE(logger, "Received Frame type: " << stream_type << " idx: " << stream_index);

                if (stream_type == rs2_stream::RS2_STREAM_COLOR) {
                    if (auto vf = f.as<rs2::video_frame>()) {
                        auto imageFormatProperties = getImageFormatPropertiesFromRS2Frame(f);

                        int w = vf.get_width();
                        int h = vf.get_height();

                        // need to copy image here.
                        auto image = cv::Mat(cv::Size(w, h), imageFormatProperties.matType, (void *) f.get_data(),
                                             cv::Mat::AUTO_STEP).clone();

                        if (m_outputColorImagePort.isConnected()) {
                            boost::shared_ptr<Vision::Image> pColorImage(new Vision::Image(image));
                            pColorImage->set_pixelFormat(imageFormatProperties.imageFormat);
                            pColorImage->set_origin(imageFormatProperties.origin);

                            if (m_autoGPUUpload) {
                                Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
                                if (oclManager.isInitialized()) {
                                    //force upload to the GPU
                                    pColorImage->uMat();
                                }
                            }
                            m_outputColorImagePort.send(Measurement::ImageMeasurement(ts, pColorImage));
                        }

                        if (m_outputGreyImagePort.isConnected()) {
                            cv::Mat grayImage;
                            cv::cvtColor(image, grayImage, cv::COLOR_RGB2GRAY);
                            boost::shared_ptr<Vision::Image> pGreyImage(new Vision::Image(grayImage));
                            pGreyImage->set_pixelFormat(imageFormatProperties.imageFormat);
                            pGreyImage->set_origin(imageFormatProperties.origin);

                            if (m_autoGPUUpload) {
                                Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
                                if (oclManager.isInitialized()) {
                                    //force upload to the GPU
                                    pGreyImage->uMat();
                                }
                            }
                            m_outputGreyImagePort.send(Measurement::ImageMeasurement(ts, pGreyImage));
                        }

                        
                    } else {
                        LOG4CPP_WARN(logger, "Expected Video-Frame but cannot cast.");
                    }
                } else if (stream_type == rs2_stream::RS2_STREAM_INFRARED) {
                    if (auto vf = f.as<rs2::video_frame>()) {
                        auto imageFormatProperties = getImageFormatPropertiesFromRS2Frame(f);

                        int w = vf.get_width();
                        int h = vf.get_height();

                        // need to copy image here.
                        auto image = cv::Mat(cv::Size(w, h), imageFormatProperties.matType, (void *) f.get_data(),
                                             cv::Mat::AUTO_STEP).clone();

                        // Left IR Image
                        if (m_outputIRLeftImagePort.isConnected() && (stream_index == 1)) {
                            boost::shared_ptr<Vision::Image> pInfraredImage(new Vision::Image(image));
                            pInfraredImage->set_pixelFormat(imageFormatProperties.imageFormat);
                            pInfraredImage->set_origin(imageFormatProperties.origin);

                            if (m_autoGPUUpload) {
                                Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
                                if (oclManager.isInitialized()) {
                                    //force upload to the GPU
                                    pInfraredImage->uMat();
                                }
                            }
                            m_outputIRLeftImagePort.send(Measurement::ImageMeasurement(ts, pInfraredImage));
                        }

                        // Right IR Image
//                if (m_outputIRRightImagePort.isConnected() && (stream_index == 2)) {
//                    boost::shared_ptr<Vision::Image> pInfraredImage(new Vision::Image(image));
//                    pInfraredImage->set_pixelFormat(imageFormatProperties.imageFormat);
//                    pInfraredImage->set_origin(imageFormatProperties.origin);
//
//                    if (m_autoGPUUpload) {
//                        Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
//                        if (oclManager.isInitialized()) {
//                            //force upload to the GPU
//                            pInfraredImage->uMat();
//                        }
//                    }
//                    m_outputIRRightImagePort.send(Measurement::ImageMeasurement(ts, pInfraredImage));
//                }
                    }
                } else if (stream_type == rs2_stream::RS2_STREAM_DEPTH) {
                    if (auto df = f.as<rs2::depth_frame>())
                    {
                        if (m_outputPointCloudPort.isConnected()) {
                            // Declare pointcloud object, for calculating pointclouds and texture mappings
                            rs2::pointcloud pc;

                            // Generate the pointcloud and texture mappings
                            rs2::points points = pc.calculate(df);

                            // Tell pointcloud object to map to this color frame
                            // @todo: currently no access to the color image .. now sure how to achieve this with the current structure ..
                            // pc.map_to(color);

                            auto vertices = points.get_vertices();

                            Math::Vector3d init_pos(0, 0, 0);
                            boost::shared_ptr < std::vector<Math::Vector3d> > pPointCloud = boost::make_shared< std::vector<Math::Vector3d> >(points.size(), init_pos);

                            for (size_t i = 0; i < points.size(); i++) {
                                Math::Vector3d& p = pPointCloud->at(i);

                                if (vertices[i].z != 0.)
                                {
                                    p[0] = vertices[i].x;
                                    p[1] = vertices[i].y;
                                    p[2] = vertices[i].z;
                                } else {
                                    p[0] = p[1] = p[2] = 0.;
                                }
                            }
                            m_outputPointCloudPort.send(Measurement::PositionList(ts, pPointCloud));
                        }

                        if (m_outputDepthMapImagePort.isConnected()) {

                            auto imageFormatProperties = getImageFormatPropertiesFromRS2Frame(f);

                            int w = df.get_width();
                            int h = df.get_height();

                            // need to copy image here.
                            auto image = cv::Mat(cv::Size(w, h), imageFormatProperties.matType, (void*)f.get_data(), cv::Mat::AUTO_STEP).clone();

                            boost::shared_ptr< Vision::Image > pDepthImage(new Vision::Image(image));
                            pDepthImage->set_pixelFormat(imageFormatProperties.imageFormat);
                            pDepthImage->set_origin(imageFormatProperties.origin);

                            if (m_autoGPUUpload) {
                                Vision::OpenCLManager &oclManager = Vision::OpenCLManager::singleton();
                                if (oclManager.isInitialized()) {
                                    //force upload to the GPU
                                    pDepthImage->uMat();
                                }
                            }
                            m_outputDepthMapImagePort.send(Measurement::ImageMeasurement(ts, pDepthImage));
                        }
                    }

                } else {
                    LOG4CPP_WARN(logger, "Stream type is not known.");
                }
            }
        }
    }

    void AzureKinectCameraComponent::stop()
    {
        if ( m_running )
        {
            m_running = false;
            LOG4CPP_INFO( logger, "Trying to stop kinect4azure module");

            m_pipeline->stop();

            teardownDevice();
        }
    }

    void AzureKinectCameraComponent::teardownDevice()
    {
        m_pipeline.reset();
    }


// register component at factory
    UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf )
    {
        cf->registerComponent< Ubitrack::Drivers::AzureKinectCameraComponent > ( "kinect4azureCamera" );
    }

} } // namespace Ubitrack::Drivers
