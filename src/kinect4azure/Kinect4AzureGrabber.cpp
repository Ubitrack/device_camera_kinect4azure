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

bool get_intrinsics_for_camera(const k4a_calibration_camera_t& k4a_calib, Math::CameraIntrinsics<double>& value) {
	auto& p = k4a_calib.intrinsics.parameters;

    Math::Matrix< double, 3, 3 > intrinsicMatrix = Math::Matrix3x3d::identity();
    intrinsicMatrix(0, 0) = (double)p.param.fx;
    intrinsicMatrix(1, 1) = (double)p.param.fy;
    intrinsicMatrix(0, 2) = -(double)p.param.cx;
    intrinsicMatrix(1, 2) = -(double)p.param.cy;
    intrinsicMatrix(2, 2) = -1.0;

	Math::Vector< double, 6 > radial;
	radial(0) = (double)p.param.k1;
	radial(1) = (double)p.param.k2;
	radial(2) = (double)p.param.k3;
	radial(3) = (double)p.param.k4;
	radial(4) = (double)p.param.k5;
	radial(5) = (double)p.param.k6;

    Math::Vector< double, 2 > tangential(
		(double)p.param.p1,
		(double)p.param.p2);
    
	auto width = (std::size_t)k4a_calib.resolution_width;
    auto height = (std::size_t)k4a_calib.resolution_height;

    value = Math::CameraIntrinsics<double>(intrinsicMatrix, radial, tangential, width, height);

    // Does this apply to Kinect Azure ??
	// this camera has image origin of 0, ubitrack default is 1, flip needed of cy and tangential parameter needed
    //value = Vision::Util::cv2::correctForOrigin(0, value);
    //LOG4CPP_DEBUG(logger, key << " Camera Model: " << value);

	return true;
}

bool get_pose_from_extrinsics(const k4a_calibration_extrinsics_t& extrinsics, Math::Pose& value) {

    auto rot_mat = Math::Matrix3x3d::identity();

    // libkinect4azure is row major and ubitrack store matrices column-major
    rot_mat( 0, 0 ) = (double)extrinsics.rotation[0];
    rot_mat( 1, 0 ) = (double)extrinsics.rotation[1];
    rot_mat( 2, 0 ) = (double)extrinsics.rotation[2];

    rot_mat( 0, 1 ) = (double)extrinsics.rotation[3];
    rot_mat( 1, 1 ) = (double)extrinsics.rotation[4];
    rot_mat( 2, 1 ) = (double)extrinsics.rotation[5];

    rot_mat( 0, 2 ) = (double)extrinsics.rotation[6];
    rot_mat( 1, 2 ) = (double)extrinsics.rotation[7];
    rot_mat( 2, 2 ) = (double)extrinsics.rotation[8];

    Math::Quaternion ut_quat(rot_mat);

    // ubitrack measurements are in meters, k4a reports extrinsics in mm
    Math::Vector3d ut_trans(
            (double)extrinsics.translation[0] / 1000.,
            (double)extrinsics.translation[1] / 1000.,
            (double)extrinsics.translation[2] / 1000.
    );
    value = Math::Pose(ut_quat, ut_trans);
    return true;
}


static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
	k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

	int width = calibration->depth_camera_calibration.resolution_width;
	int height = calibration->depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
		}
	}
}


Ubitrack::Vision::Image::ImageFormatProperties getImageFormatPropertiesFromK4AImage(const k4a::image& f) {
    auto imageFormatProperties = Vision::Image::ImageFormatProperties();
    switch (f.get_format()) {
		case K4A_IMAGE_FORMAT_COLOR_BGRA32:
            imageFormatProperties.depth = CV_8U;
            imageFormatProperties.channels = 4;
            imageFormatProperties.matType = CV_8UC4;
            imageFormatProperties.bitsPerPixel = 32;
            imageFormatProperties.origin = 0;
            imageFormatProperties.imageFormat = Vision::Image::BGRA;
            break;

		case K4A_IMAGE_FORMAT_DEPTH16:
			imageFormatProperties.depth = CV_16U;
			imageFormatProperties.channels = 1;
			imageFormatProperties.matType = CV_16UC1;
			imageFormatProperties.bitsPerPixel = 16;
			imageFormatProperties.origin = 0;
			imageFormatProperties.imageFormat = Vision::Image::DEPTH;
			break;

		case K4A_IMAGE_FORMAT_IR16:
			imageFormatProperties.depth = CV_16U;
			imageFormatProperties.channels = 1;
			imageFormatProperties.matType = CV_16UC1;
			imageFormatProperties.bitsPerPixel = 16;
			imageFormatProperties.origin = 0;
			imageFormatProperties.imageFormat = Vision::Image::LUMINANCE;
			break;

		case K4A_IMAGE_FORMAT_CUSTOM8:
			imageFormatProperties.depth = CV_8U;
			imageFormatProperties.channels = 1;
			imageFormatProperties.matType = CV_8UC1;
			imageFormatProperties.bitsPerPixel = 8;
			imageFormatProperties.origin = 0;
			imageFormatProperties.imageFormat = Vision::Image::LUMINANCE;
			break;
		case K4A_IMAGE_FORMAT_COLOR_MJPG:
		case K4A_IMAGE_FORMAT_COLOR_NV12:
		case K4A_IMAGE_FORMAT_COLOR_YUY2:
		case K4A_IMAGE_FORMAT_CUSTOM16:
		default:
            UBITRACK_THROW("kinect4azure frame format is (currently) not supported!");
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
		, m_colorResolution(K4A_COLOR_RESOLUTION_1536P)
		, m_depthMode(K4A_DEPTH_MODE_NFOV_2X2BINNED)
		, m_frameRate(K4A_FRAMES_PER_SECOND_30)
		, m_synchronizedImagesOnly(true)
	    , m_depthDelayOffcolorUsec(0)
	    , m_subordinateDelayOffMasterUsec(0)
	    , m_disableStreamingIndicator(false)
		, m_device_config(K4A_DEVICE_CONFIG_INIT_DISABLE_ALL)
		, m_serialNumber("")
        //, m_depthLaserPower(150)
        //, m_depthEmitterEnabled(1)
        //, m_infraredGain(16)
		, m_xytable_initialized(false)
        , m_autoGPUUpload(false)
    {

        if (subgraph->m_DataflowAttributes.hasAttribute("k4aSerialNumber")) {
            m_serialNumber = subgraph->m_DataflowAttributes.getAttributeString("k4aSerialNumber");
        }

		if (subgraph->m_DataflowAttributes.hasAttribute("k4aDepthMode"))
		{
			std::string sDepthMode = subgraph->m_DataflowAttributes.getAttributeString("k4aDepthMode");
			if (kinect4azureDepthStreamFormatMap.find(sDepthMode) == kinect4azureDepthStreamFormatMap.end())
				UBITRACK_THROW("unknown depth mode: \"" + sDepthMode + "\"");
			m_depthMode = kinect4azureDepthStreamFormatMap[sDepthMode];
		}
		
		if ( subgraph->m_DataflowAttributes.hasAttribute( "k4aColorVideoResolution" ) )
        {
            std::string sResolution = subgraph->m_DataflowAttributes.getAttributeString( "k4aColorVideoResolution" );
            if ( kinect4azureColorStreamResolutionMap.find( sResolution ) == kinect4azureColorStreamResolutionMap.end() )
                UBITRACK_THROW( "unknown color stream resolution: \"" + sResolution + "\"" );
            m_colorResolution = kinect4azureColorStreamResolutionMap[ sResolution ];
        }
        
		if ( subgraph->m_DataflowAttributes.hasAttribute( "k4aColorImageFormat" ) )
        {
            std::string sStreamFormat = subgraph->m_DataflowAttributes.getAttributeString( "k4aColorImageFormat" );
            if (kinect4azureImageFormatMap.find( sStreamFormat ) == kinect4azureImageFormatMap.end() )
                UBITRACK_THROW( "unknown color stream type: \"" + sStreamFormat + "\"" );
            m_colorImageFormat = kinect4azureImageFormatMap[ sStreamFormat ];
        }

		if (subgraph->m_DataflowAttributes.hasAttribute("k4aFrameRate"))
		{
			std::string sFrameRate = subgraph->m_DataflowAttributes.getAttributeString("k4aFrameRate");
			if (kinect4azureFrameRateMap.find(sFrameRate) == kinect4azureFrameRateMap.end())
				UBITRACK_THROW("unknown frame rate: \"" + sFrameRate + "\"");
			m_frameRate = kinect4azureFrameRateMap[sFrameRate];
		}

		if (subgraph->m_DataflowAttributes.hasAttribute("k4aSynchronizedImagesOnly")) {
			m_synchronizedImagesOnly = subgraph->m_DataflowAttributes.getAttributeString("k4aSynchronizedImagesOnly") == "true";
		}

		if (subgraph->m_DataflowAttributes.hasAttribute("k4aDisableStreamingIndicator")) {
			m_disableStreamingIndicator = subgraph->m_DataflowAttributes.getAttributeString("k4aDisableStreamingIndicator") == "true";
		}

		subgraph->m_DataflowAttributes.getAttributeData("k4aDelayOffColorUsec", m_depthDelayOffcolorUsec);
		subgraph->m_DataflowAttributes.getAttributeData("k4aSubordinateDelayOffMaster", m_subordinateDelayOffMasterUsec);


        if ( subgraph->m_DataflowAttributes.hasAttribute( "k4aWiredSyncMode" ) )
        {
            std::string sWiredSyncMode = subgraph->m_DataflowAttributes.getAttributeString( "k4aWiredSyncMode" );
            if (kinect4azureHWSyndModeMap.find( sWiredSyncMode ) == kinect4azureHWSyndModeMap.end() )
                UBITRACK_THROW( "unknown wired sync mode: \"" + sWiredSyncMode + "\"" );
            m_hwsync_mode = kinect4azureHWSyndModeMap[ sWiredSyncMode ];
        }

        //subgraph->m_DataflowAttributes.getAttributeData( "rsLaserPower", m_depthLaserPower);
        //subgraph->m_DataflowAttributes.getAttributeData( "rsEmitterEnabled", m_depthEmitterEnabled);
        //subgraph->m_DataflowAttributes.getAttributeData( "rsInfraredGain", m_infraredGain);

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

		// Find the device
		//

		const uint32_t device_count = k4a::device::get_installed_count();
		if (device_count == 0)
		{
			UBITRACK_THROW("No Azure Kinect devices detected!");
		}

		bool found_device = false;
		LOG4CPP_DEBUG(logger, "Found " << int(device_count) << " connected devices:");

		if (m_serialNumber == "") {
			m_device = k4a::device::open(0);
			found_device = true;
		}
		else {
			for (uint8_t deviceIndex = 0; deviceIndex < device_count; deviceIndex++)
			{
				auto candidate = k4a::device::open(deviceIndex);
				auto serialnr = candidate.get_serialnum();
				if (m_serialNumber == serialnr) {
					LOG4CPP_INFO(logger, "Found Azure Kinect with Serial: " << serialnr);
					found_device = true;
					m_device = std::move(candidate);
					break;
				}
				else {
					LOG4CPP_DEBUG(logger, "Found Azure Kinect with NON MATCHING Serial: " << serialnr);
				}
			}
		}

		if (!found_device) {
			UBITRACK_THROW("No Azure Kinect device with matching serial number detected!");
		}

		// Start the device
		//

		m_device_config.camera_fps = m_frameRate;
		m_device_config.depth_mode = m_depthMode;
		m_device_config.color_format = m_colorImageFormat;
		m_device_config.color_resolution = m_colorResolution;
		m_device_config.synchronized_images_only = m_synchronizedImagesOnly;
		m_device_config.depth_delay_off_color_usec = m_depthDelayOffcolorUsec;
		m_device_config.wired_sync_mode = m_hwsync_mode;
		m_device_config.subordinate_delay_off_master_usec = m_subordinateDelayOffMasterUsec;
		m_device_config.disable_streaming_indicator = m_disableStreamingIndicator;

    }

    void AzureKinectCameraComponent::retrieveCalibration() {

		auto calibration = m_device.get_calibration(m_device_config.depth_mode, m_device_config.color_resolution);

		get_intrinsics_for_camera(calibration.color_camera_calibration, m_colorCameraModel);
		m_color_undistorter.reset(new Vision::Undistortion(m_colorCameraModel));

		if (calibration.depth_mode != K4A_DEPTH_MODE_OFF) {
			get_intrinsics_for_camera(calibration.depth_camera_calibration, m_depthCameraModel);
			get_pose_from_extrinsics(calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR], m_depthToColorTransform);

			m_depth_undistorter.reset(new Vision::Undistortion(m_depthCameraModel));
		}

		if (m_depthMode != K4A_DEPTH_MODE_OFF) {
			k4a_image_t xy_table = NULL;
			k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
				calibration.depth_camera_calibration.resolution_width,
				calibration.depth_camera_calibration.resolution_height,
				calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
				&xy_table);

			create_xy_table(&calibration, xy_table);

			m_xytable = k4a::image(xy_table);
			m_xytable_initialized = true;

		}
    }

    void AzureKinectCameraComponent::setOptions() {
        /** Kinect for Azure Options:
			K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE = 0,
			K4A_COLOR_CONTROL_AUTO_EXPOSURE_PRIORITY,
			K4A_COLOR_CONTROL_BRIGHTNESS,
			K4A_COLOR_CONTROL_CONTRAST,
			K4A_COLOR_CONTROL_SATURATION,
			K4A_COLOR_CONTROL_SHARPNESS,
			K4A_COLOR_CONTROL_WHITEBALANCE,
			K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION,
			K4A_COLOR_CONTROL_GAIN,
			K4A_COLOR_CONTROL_POWERLINE_FREQUENCY
         */

		// not yet implemented

    }

    void AzureKinectCameraComponent::startCapturing() {

        setupDevice();
        retrieveCalibration();
        setOptions();

		LOG4CPP_DEBUG(logger, "Started opening K4A device...");
		m_device.start_cameras(&m_device_config);
		LOG4CPP_DEBUG(logger, "Finished opening K4A device.");

        // Start streaming
		LOG4CPP_DEBUG(logger, "Starting capturing thread");
		m_running = true;
		m_pCaptureThread = boost::shared_ptr< boost::thread >(new boost::thread(boost::bind(&AzureKinectCameraComponent::threadFunc, this)));

    }


	void AzureKinectCameraComponent::threadFunc() {


		while (m_running) {

			// decide based on framerate how long we should wait.
			std::chrono::milliseconds waitms = std::chrono::milliseconds(0);
			switch (m_frameRate) {
			case K4A_FRAMES_PER_SECOND_5:
				waitms = std::chrono::milliseconds(1000 / 5);
			case K4A_FRAMES_PER_SECOND_15:
				waitms = std::chrono::milliseconds(1000 / 15);
			case K4A_FRAMES_PER_SECOND_30:
				waitms = std::chrono::milliseconds(1000 / 30);
			default:
				break;
			}

			k4a::capture capture;
			if (m_device.get_capture(&capture, waitms))
			{
			
				// should check if output is connected here ..	
				const k4a::image depthImage = capture.get_depth_image();
				if (depthImage) { 
					handleDepthFrame(depthImage);
				} else { 
					/* ignore the image */ 
				}

				// should check if output is connected here ..
				const k4a::image colorImage = capture.get_color_image();
				if (colorImage) {
					handleColorFrame(colorImage);
				}
				else {
					/* ignore the image */
				}

				// should handle IR images as well (capture.get_ir_image())
			}
		}
	}

	void AzureKinectCameraComponent::handleDepthFrame(k4a::image frame) {

		auto ts = Measurement::Timestamp(frame.get_system_timestamp().count());

		if (m_outputPointCloudPort.isConnected()) {
			int width = frame.get_width_pixels();
			int height = frame.get_height_pixels();
			
			uint16_t *depth_data = (uint16_t *)(void *)frame.get_buffer();
			k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)m_xytable.get_buffer();

			int number_of_points = width * height;
			Math::Vector3d init_pos(0, 0, 0);
			boost::shared_ptr < std::vector<Math::Vector3d> > pPointCloud = boost::make_shared< std::vector<Math::Vector3d> >(number_of_points, init_pos);

			for (int i = 0; i < width * height; i++)
			{
				Math::Vector3d& p = pPointCloud->at(i);

				if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
				{
					p[0] = xy_table_data[i].xy.x * (float)depth_data[i];
					p[1] = xy_table_data[i].xy.y * (float)depth_data[i];
					p[2] = (float)depth_data[i];
				}
				else
				{
					p[0] = nanf("");
					p[1] = nanf("");
					p[1] = nanf("");
				}
			}

			m_outputPointCloudPort.send(Measurement::PositionList(ts, pPointCloud));
		}

		if (m_outputDepthMapImagePort.isConnected()) {

			auto imageFormatProperties = getImageFormatPropertiesFromK4AImage(frame);

			int w = frame.get_width_pixels();
			int h = frame.get_height_pixels();

			// clone if not undistort !!!
			auto image = cv::Mat(cv::Size(w, h), imageFormatProperties.matType, (void*)frame.get_buffer(), cv::Mat::AUTO_STEP).clone();

			boost::shared_ptr< Vision::Image > pDepthImage(new Vision::Image(image));
			// @todo: undistort seems not to be working with depth images
			// pDepthImage = m_color_undistorter->undistort( pDepthImage );

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






	void AzureKinectCameraComponent::handleColorFrame(k4a::image frame) {

		auto ts = Measurement::Timestamp(frame.get_system_timestamp().count());
		auto imageFormatProperties = getImageFormatPropertiesFromK4AImage(frame);

		int w = frame.get_width_pixels();
		int h = frame.get_height_pixels();

		auto image = cv::Mat(cv::Size(w, h), imageFormatProperties.matType, (void *)frame.get_buffer(), cv::Mat::AUTO_STEP);
		boost::shared_ptr< Vision::Image > pColorImage;

		pColorImage.reset(new Vision::Image(image));
		pColorImage = m_color_undistorter->undistort( pColorImage );

		pColorImage->set_pixelFormat(imageFormatProperties.imageFormat);
		pColorImage->set_origin(imageFormatProperties.origin);

		if (m_outputColorImagePort.isConnected()) {

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
			cv::cvtColor(pColorImage->Mat(), grayImage, cv::COLOR_RGB2GRAY);
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


		// not implemented IR Stream ??

    }

    void AzureKinectCameraComponent::stop()
    {
        if ( m_running )
        {
            m_running = false;
            LOG4CPP_INFO( logger, "Trying to stop kinect4azure module");

			m_pCaptureThread->join();

            teardownDevice();
        }
    }

    void AzureKinectCameraComponent::teardownDevice()
    {
		m_device.close();
    }


// register component at factory
    UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf )
    {
        cf->registerComponent< Ubitrack::Drivers::AzureKinectCameraComponent > ( "Kinect4AzureCamera" );
    }

} } // namespace Ubitrack::Drivers
