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
 * @ingroup driver_components
 * @file
 * kinect4azure driver
 * This file contains the driver component to
 * synchronously capture camera images using kinect4azure.
 *
 * The received data is sent via a push interface.
 *
 * @author Ulrich Eck <ulrick.eck@tum.de>
 */

#ifndef __kinect4azureFramegrabber_h_INCLUDED__
#define __kinect4azureFramegrabber_h_INCLUDED__


#include <string>
#include <cstdlib>

#include <iostream>
#include <map>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/filesystem.hpp>

#include <utUtil/Filesystem.h>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/Component.h>
#include <utDataflow/Module.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utVision/OpenCLManager.h>
#include <utUtil/TracingProvider.h>

#include <utVision/Image.h>
#include <opencv2/opencv.hpp>

#include <libkinect4azure2/rs.hpp>


namespace Ubitrack { namespace Drivers {

        enum kinect4azureSensorOperationMode {
            OPERATION_MODE_LIVESTREAM = 0,
            OPERATION_MODE_LIVESTREAM_RECORD,
            OPERATION_MODE_PLAYBACK
        };

    }
}

namespace {

    class kinect4azureOperationModeMap : public std::map< std::string, Ubitrack::Drivers::kinect4azureSensorOperationMode > {
    public:
        kinect4azureOperationModeMap() {
            (*this)["LIVESTREAM"] = Ubitrack::Drivers::OPERATION_MODE_LIVESTREAM;
            (*this)["LIVESTREAM_RECORD"] = Ubitrack::Drivers::OPERATION_MODE_LIVESTREAM_RECORD;
            (*this)["PLAYBACK"] = Ubitrack::Drivers::OPERATION_MODE_PLAYBACK;
        }
    };
    static kinect4azureOperationModeMap kinect4azureOperationModeMap;


    class kinect4azureHWSyndModeMap : public std::map< std::string, unsigned int > {
    public:
        kinect4azureHWSyndModeMap() {
            (*this)["DEFAULT"] = 0;
            (*this)["MASTER"] = 1;
            (*this)["SLAVE"] = 2;
        }
    };
    static kinect4azureHWSyndModeMap kinect4azureHWSyndModeMap;


    class kinect4azureStreamFormatMap : public std::map< std::string, rs2_format > {
    public:
        kinect4azureStreamFormatMap() {
            (*this)["Z16"] = rs2_format::RS2_FORMAT_Z16;
            (*this)["DISPARITY16"] = rs2_format::RS2_FORMAT_DISPARITY16;
            (*this)["XYZ32F"] = rs2_format::RS2_FORMAT_XYZ32F;
            (*this)["YUYV"] = rs2_format::RS2_FORMAT_YUYV;
            (*this)["RGB8"] = rs2_format::RS2_FORMAT_RGB8;
            (*this)["BGR8"] = rs2_format::RS2_FORMAT_BGR8;
            (*this)["RGBA8"] = rs2_format::RS2_FORMAT_RGBA8;
            (*this)["BGRA8"] = rs2_format::RS2_FORMAT_BGRA8;
            (*this)["Y8"] = rs2_format::RS2_FORMAT_Y8;
            (*this)["Y16"] = rs2_format::RS2_FORMAT_Y16;
            (*this)["RAW10"] = rs2_format::RS2_FORMAT_RAW10;
            (*this)["RAW16"] = rs2_format::RS2_FORMAT_RAW16;
            (*this)["RAW8"] = rs2_format::RS2_FORMAT_RAW8;
        }
    };
    static kinect4azureStreamFormatMap kinect4azureStreamFormatMap;

    class kinect4azureStreamResolutionMap : public std::map< std::string, std::tuple< unsigned int, unsigned int> > {
    public:
        kinect4azureStreamResolutionMap() {
            (*this)["1920x1080"] = std::make_tuple(1920, 1080);
            (*this)["1280x800"] = std::make_tuple(1280, 800);
            (*this)["1280x720"] = std::make_tuple(1280, 720);
            (*this)["960x540"] = std::make_tuple(960, 540);
            (*this)["848x480"] = std::make_tuple(848, 480);
            (*this)["640x480"] = std::make_tuple(640, 480);
            (*this)["640x400"] = std::make_tuple(640, 400);
            (*this)["640x360"] = std::make_tuple(640, 360);
            (*this)["480x270"] = std::make_tuple(480, 270);
            (*this)["424x240"] = std::make_tuple(424, 240);
            (*this)["320x240"] = std::make_tuple(320, 240);
            (*this)["320x180"] = std::make_tuple(320, 180);
        }
    };
    static kinect4azureStreamResolutionMap kinect4azureStreamResolutionMap;


} // anonymous namespace



namespace Ubitrack { namespace Drivers {
using namespace Dataflow;

    struct stream_request
    {
        rs2_stream   _stream_type;
        rs2_format   _stream_format;
        unsigned int _width;
        unsigned int _height;
        unsigned int _fps;
        unsigned int _stream_idx;
        std::string  _port_name;
    };

    class kinect4azureCameraComponent : public Dataflow::Component {
    public:
        kinect4azureCameraComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

        void setupDevice();

        void retrieveCalibration();

        void setOptions();

        void start();

        void startCapturing();

        void handleFrame(rs2::frame f);

        void stop();

        virtual void teardownDevice();

    protected:

        Measurement::CameraIntrinsics getColorCameraModel(Measurement::Timestamp t) {
            return Measurement::CameraIntrinsics(t, m_colorCameraModel);
        }
        
        Measurement::Matrix3x3 getColorIntrinsic(Measurement::Timestamp t) {
            return Measurement::Matrix3x3(t, m_colorCameraModel.matrix);
        }
        
        Measurement::CameraIntrinsics getIRLeftCameraModel(Measurement::Timestamp t) {
            return Measurement::CameraIntrinsics(t, m_infraredLeftCameraModel);
        }
        
        Measurement::Matrix3x3 getIRLeftIntrinsic(Measurement::Timestamp t) {
            return Measurement::Matrix3x3(t, m_infraredLeftCameraModel.matrix);
        }
        
        Measurement::CameraIntrinsics getIRRightCameraModel(Measurement::Timestamp t) {
            return Measurement::CameraIntrinsics(t, m_infraredRightCameraModel);
        }
        
        Measurement::Matrix3x3 getIRRightIntrinsic(Measurement::Timestamp t) {
            return Measurement::Matrix3x3(t, m_infraredRightCameraModel.matrix);
        }
        
        Measurement::Pose getLeftToRightTransform(Measurement::Timestamp t) {
            return Measurement::Pose(t, m_leftToRightTransform);
        }
        
        Measurement::Pose getLeftToColorTransform(Measurement::Timestamp t) {
            return Measurement::Pose(t, m_leftToColorTransform);
        }

        Math::CameraIntrinsics<double> m_colorCameraModel;
        Math::CameraIntrinsics<double> m_infraredLeftCameraModel;
        Math::CameraIntrinsics<double> m_infraredRightCameraModel;
        Math::Pose m_leftToRightTransform;
        Math::Pose m_leftToColorTransform;

        bool m_haveColorStream;
        bool m_haveIRLeftStream;
//        bool m_haveIRRightStream;
        bool m_haveDepthStream;

        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputColorImagePort;
        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputGreyImagePort;
        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputIRLeftImagePort;
//        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputIRRightImagePort;
        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputDepthMapImagePort;
        Dataflow::PushSupplier <Measurement::PositionList>     m_outputPointCloudPort;

        Dataflow::PullSupplier <Measurement::CameraIntrinsics> m_outputColorCameraModelPort;
        Dataflow::PullSupplier <Measurement::Matrix3x3>        m_outputColorIntrinsicsMatrixPort;
        Dataflow::PullSupplier <Measurement::CameraIntrinsics> m_outputIRLeftCameraModelPort;
        Dataflow::PullSupplier <Measurement::Matrix3x3>        m_outputIRLeftIntrinsicsMatrixPort;
//        Dataflow::PullSupplier <Measurement::CameraIntrinsics> m_outputIRRightCameraModelPort;
//        Dataflow::PullSupplier <Measurement::Matrix3x3>        m_outputIRRightIntrinsicsMatrixPort;

//        Dataflow::PullSupplier <Measurement::Pose> m_leftIRToRightIRTransformPort;
        Dataflow::PullSupplier <Measurement::Pose> m_leftIRToColorTransformPort;

        // sensor configuration
        unsigned int m_hwsync_mode;
        unsigned int m_colorImageWidth;
        unsigned int m_colorImageHeight;
        unsigned int m_depthImageWidth;
        unsigned int m_depthImageHeight;

        unsigned int m_frameRate;

        rs2_format m_colorStreamFormat;
        rs2_format m_infraredStreamFormat;
        rs2_format m_depthStreamFormat;

        std::string m_serialNumber;

        unsigned int m_depthLaserPower;
        unsigned int m_depthEmitterEnabled;
        unsigned int m_infraredGain;

        /** libkinect4azure context for managing devices **/
        rs2::context m_ctx;

        /** the associated kinect4azure device **/
        rs2::config m_pipeline_config;
        std::shared_ptr<rs2::pipeline> m_pipeline;
        rs2::pipeline_profile m_pipeline_profile;
        std::vector<stream_request> m_stream_requests;
        std::map<std::string, rs2::stream_profile> m_stream_profile_map;
        std::vector<rs2::stream_profile> m_selected_stream_profiles;


        // sensor operation mode
        kinect4azureSensorOperationMode m_operation_mode;
        boost::filesystem::path m_rosbag_filename;
        boost::filesystem::path m_timestamp_filename;
        boost::filesystem::path m_cameramodel_left_filename;
        boost::filesystem::path m_cameramodel_color_filename;
        boost::filesystem::path m_depth2color_filename;

        std::filebuf m_timestamp_filebuffer;

        bool m_autoGPUUpload;
    };


} } // namespace Ubitrack::Drivers

#endif // __kinect4azureFramegrabber_h_INCLUDED__
