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

#include <k4a/k4a.hpp>

#include <string>
#include <cstdlib>

#include <iostream>
#include <map>
#include <boost/array.hpp>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
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



namespace {

    class Kinect4azureHWSyndModeMap : public std::map< std::string, k4a_wired_sync_mode_t > {
    public:
        Kinect4azureHWSyndModeMap() {
        (*this)["STANDALONE"] = K4A_WIRED_SYNC_MODE_STANDALONE; 
        (*this)["MASTER"] = K4A_WIRED_SYNC_MODE_MASTER;     
        (*this)["SUBORDINATE"] = K4A_WIRED_SYNC_MODE_SUBORDINATE;
        }
    };
    static Kinect4azureHWSyndModeMap kinect4azureHWSyndModeMap;


    class Kinect4azureDepthStreamFormatMap : public std::map< std::string, k4a_depth_mode_t > {
    public:
        Kinect4azureDepthStreamFormatMap() {
            (*this)["OFF"] = K4A_DEPTH_MODE_OFF;        /**< Depth sensor will be turned off with this setting. */
            (*this)["NFOV_2X2BINNED"] = K4A_DEPTH_MODE_NFOV_2X2BINNED; /**< Depth captured at 320x288. Passive IR is also captured at 320x288. */
            (*this)["NFOV_UNBINNED"] = K4A_DEPTH_MODE_NFOV_UNBINNED;  /**< Depth captured at 640x576. Passive IR is also captured at 640x576. */
            (*this)["WFOV_2X2BINNED"] = K4A_DEPTH_MODE_WFOV_2X2BINNED; /**< Depth captured at 512x512. Passive IR is also captured at 512x512. */
            (*this)["WFOV_UNBINNED"] = K4A_DEPTH_MODE_WFOV_UNBINNED;  /**< Depth captured at 1024x1024. Passive IR is also captured at 1024x1024. */
            (*this)["PASSIVE_IR"] = K4A_DEPTH_MODE_PASSIVE_IR;     /**< Passive IR only, captured at 1024x1024. */
        }
    };
    static Kinect4azureDepthStreamFormatMap kinect4azureDepthStreamFormatMap;


    class Kinect4azureColorStreamResolutionMap : public std::map< std::string, k4a_color_resolution_t > {
    public:
        Kinect4azureColorStreamResolutionMap() {
        (*this)["OFF"] = K4A_COLOR_RESOLUTION_OFF; /**< Color camera will be turned off with this setting */
        (*this)["720P"] = K4A_COLOR_RESOLUTION_720P;    /**< 1280 * 720  16:9 */
        (*this)["1080P"] = K4A_COLOR_RESOLUTION_1080P;   /**< 1920 * 1080 16:9 */
        (*this)["1440P"] = K4A_COLOR_RESOLUTION_1440P;   /**< 2560 * 1440 16:9 */
        (*this)["1536P"] = K4A_COLOR_RESOLUTION_1536P;   /**< 2048 * 1536 4:3  */
        (*this)["2160P"] = K4A_COLOR_RESOLUTION_2160P;   /**< 3840 * 2160 16:9 */
        (*this)["3072P"] = K4A_COLOR_RESOLUTION_3072P;   /**< 4096 * 3072 4:3  */
        }
    };
    static Kinect4azureColorStreamResolutionMap kinect4azureColorStreamResolutionMap;

    class Kinect4azureImageFormatMap : public std::map< std::string, k4a_image_format_t > {
    public:
        Kinect4azureImageFormatMap() {
        (*this)["COLOR_MJPG"] = K4A_IMAGE_FORMAT_COLOR_MJPG;
        (*this)["COLOR_NV12"] = K4A_IMAGE_FORMAT_COLOR_NV12;
        (*this)["COLOR_YUY2"] = K4A_IMAGE_FORMAT_COLOR_YUY2;
        (*this)["COLOR_BGRA32"] = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        (*this)["DEPTH16"] = K4A_IMAGE_FORMAT_DEPTH16;
        (*this)["IR16"] = K4A_IMAGE_FORMAT_IR16;
        (*this)["CUSTOM8"] = K4A_IMAGE_FORMAT_CUSTOM8;
        (*this)["CUSTOM16"] = K4A_IMAGE_FORMAT_CUSTOM16;
        (*this)["CUSTOM"] = K4A_IMAGE_FORMAT_CUSTOM;
        
        }
    };
    static Kinect4azureImageFormatMap kinect4azureImageFormatMap;


    class Kinect4azureFrameRateMap : public std::map< std::string, k4a_fps_t > {
    public:
        Kinect4azureFrameRateMap() {
        (*this)["5"] = K4A_FRAMES_PER_SECOND_5; /**< 5 FPS */
        (*this)["15"] = K4A_FRAMES_PER_SECOND_15;    /**< 15 FPS */
        (*this)["30"] = K4A_FRAMES_PER_SECOND_30;    /**< 30 FPS */
        }
    };
    static Kinect4azureFrameRateMap kinect4azureFrameRateMap;

} // anonymous namespace



namespace Ubitrack { namespace Drivers {
using namespace Dataflow;


    class AzureKinectCameraComponent : public Dataflow::Component {
    public:
        AzureKinectCameraComponent( const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >  );

        void setupDevice();

        void retrieveCalibration();

        void setOptions();

        void start();

        void startCapturing();

        void handleDepthFrame(k4a::image f);

		void handleColorFrame(k4a::image f);
		
		void stop();

        virtual void teardownDevice();

    protected:

		void threadFunc();

        Measurement::CameraIntrinsics getColorCameraModel(Measurement::Timestamp t) {
            return Measurement::CameraIntrinsics(t, m_colorCameraModel);
        }
        
        Measurement::Matrix3x3 getColorIntrinsic(Measurement::Timestamp t) {
            return Measurement::Matrix3x3(t, m_colorCameraModel.matrix);
        }
        
        Measurement::CameraIntrinsics getDepthCameraModel(Measurement::Timestamp t) {
            return Measurement::CameraIntrinsics(t, m_depthCameraModel);
        }
        
        Measurement::Matrix3x3 getDepthIntrinsic(Measurement::Timestamp t) {
            return Measurement::Matrix3x3(t, m_depthCameraModel.matrix);
        }
        
        Measurement::Pose getDepthToColorTransform(Measurement::Timestamp t) {
            return Measurement::Pose(t, m_depthToColorTransform);
        }

        Math::CameraIntrinsics<double> m_colorCameraModel;
        Math::CameraIntrinsics<double> m_depthCameraModel;
        Math::Pose m_depthToColorTransform;

        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputColorImagePort;
        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputGreyImagePort;
        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputIRImagePort;
        Dataflow::PushSupplier <Measurement::ImageMeasurement> m_outputDepthMapImagePort;
        Dataflow::PushSupplier <Measurement::PositionList>     m_outputPointCloudPort;

        Dataflow::PullSupplier <Measurement::CameraIntrinsics> m_outputColorCameraModelPort;
        Dataflow::PullSupplier <Measurement::Matrix3x3>        m_outputColorIntrinsicsMatrixPort;
        Dataflow::PullSupplier <Measurement::CameraIntrinsics> m_outputDepthCameraModelPort;
        Dataflow::PullSupplier <Measurement::Matrix3x3>        m_outputDepthIntrinsicsMatrixPort;

        Dataflow::PullSupplier <Measurement::Pose> m_depthToColorTransformPort;

        // sensor configuration
		k4a_wired_sync_mode_t m_hwsync_mode;
		k4a_image_format_t m_colorImageFormat;
		k4a_color_resolution_t m_colorResolution;
		k4a_depth_mode_t m_depthMode;
		k4a_fps_t m_frameRate;

		bool m_synchronizedImagesOnly;
		int32_t m_depthDelayOffcolorUsec;
		uint32_t m_subordinateDelayOffMasterUsec;
		bool m_disableStreamingIndicator;

        std::string m_serialNumber;

        //unsigned int m_depthLaserPower;
        //unsigned int m_depthEmitterEnabled;
        //unsigned int m_infraredGain;

		boost::shared_ptr< boost::thread > m_pCaptureThread;

        /** libkinect4azure instances **/

		k4a::device m_device;
		k4a_device_configuration_t m_device_config;

        bool m_autoGPUUpload;
    };


} } // namespace Ubitrack::Drivers

#endif // __kinect4azureFramegrabber_h_INCLUDED__
