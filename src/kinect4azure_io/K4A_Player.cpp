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
 * The player component for playback of recorded kinect 4 azure mkv files.
 * Based on the standard ubitrack player component and the k4a example
 * https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/examples/playback_external_sync/main.c
 *
 * @author Frieder Pankratz <pankratz@in.tum.de>
 */

// Ubitrack
#include <utUtil/OS.h>
#include <utUtil/CalibFile.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/Timestamp.h>
#include <utDataflow/Module.h>
#include <utDataflow/PushSupplier.h>
#include <utVision/Image.h>

// std
#include <deque> // for image queue
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

// Boost
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

// K4A
#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>



typedef struct
{
    k4a_playback_t handle;
    k4a_record_configuration_t record_config;
    k4a_capture_t capture;
} recording_t;


static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.Player" ) );

namespace Ubitrack { namespace Drivers {

// forward decls
class K4APlayerComponentBase;


/**
 * Component key for PlayerProducer/Consumer components.
 */
class K4APlayerComponentKey
	: public Dataflow::DataflowConfigurationAttributeKey< std::string >
{
public:
	/** extract the "file" parameter of the edge config. */
	K4APlayerComponentKey( boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
		: Dataflow::DataflowConfigurationAttributeKey< std::string >( pConfig, "file" )
	{}
};


/**
 * Module used by player components, maintains a single main loop for all player components
 */
class K4APlayerModule
	: public Dataflow::Module< Dataflow::SingleModuleKey, K4APlayerComponentKey, K4APlayerModule, K4APlayerComponentBase >
{
public:
	/** simplifies our life afterwards */
	typedef Dataflow::Module< Dataflow::SingleModuleKey, K4APlayerComponentKey, K4APlayerModule, K4APlayerComponentBase > BaseClass;

	K4APlayerModule( const Dataflow::SingleModuleKey& key, boost::shared_ptr< Graph::UTQLSubgraph >, FactoryHelper* fh )
		: BaseClass( key, fh )
		, m_bStop( false )
	{
		LOG4CPP_INFO( logger, "created K4APlayerModule" );
	}

	~K4APlayerModule()
	{
		// stop main loop
		m_bStop = true;

		// wait for thread
		if ( m_pMainLoopThread )
			m_pMainLoopThread->join();

		LOG4CPP_INFO( logger, "destroyed K4APlayerModule" );
	}

	void startThread()
	{
		LOG4CPP_DEBUG( logger, "starting thread" );

		// start mainloop
		if ( !m_pMainLoopThread )
			m_pMainLoopThread.reset( new boost::thread( boost::bind( &K4APlayerModule::mainloop, this ) ) );
	}

protected:

	/** the main loop thread */
	boost::shared_ptr< boost::thread > m_pMainLoopThread;

	/** stop the main loop? */
	bool m_bStop;

	/** method that runs the main loop */
	void mainloop();

	/** create new components */
	boost::shared_ptr< K4APlayerComponentBase > createComponent( const std::string& type, const std::string& name,
		boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const ComponentKey& key, K4APlayerModule* pModule );
};


/**
 * Base class for all player components.
 */
class K4APlayerComponentBase
	: public K4APlayerModule::Component
{
public:
	K4APlayerComponentBase( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph >, const K4APlayerComponentKey& key,
		K4APlayerModule* module )
		: K4APlayerModule::Component( name, key, module )
	{}

	virtual ~K4APlayerComponentBase()
	{}

	/** returns the timestamp of the first event */
	virtual Measurement::Timestamp getFirstTime() const
	{ assert( false ); return 0; }

	/** return real time of the next measurement to be played or 0 if no events */
	virtual Measurement::Timestamp getNextTime( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ assert( false ); return 0; }

	/** send the next event with the given offset */
	virtual void sendNext( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ assert( false ); }
	
	virtual void sendFinished() {

		//assert( false );
		// @todo how to gracefully exit the player ?
		LOG4CPP_WARN( logger, "Player reached end of recording..");
	}

	virtual bool isMaster()
	{return false;}

	virtual void start()
	{
		// for some reason, the default startModule mechanism does not work here...
		K4APlayerModule::Component::start();
		getModule().startThread();
	}
};


class K4APlayerComponentImage
	: public K4APlayerComponentBase
{
public:
	/** loads the file */
	K4APlayerComponentImage( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const K4APlayerComponentKey& key,
		K4APlayerModule* module )
		: K4APlayerComponentBase( name, pConfig, key, module )
		, m_tsFile( "" )
		, m_offset( 0 )
		, m_speedup( 1.0 )
		, m_outPortRGB( "Output", *this )
		, m_outPortDepth( "OutputDepth", *this )
		, m_outPortIR( "OutputIR", *this )
		, m_isMaster(false)
	{
		LOG4CPP_INFO( logger, "Created K4APlayerComponentImage using file \"" << key.get() << "\"." );

		// read configuration
		pConfig->m_DataflowAttributes.getAttributeData("offset", m_offset );
        pConfig->m_DataflowAttributes.getAttributeData( "speedup", m_speedup );

		// get the file which describes the timestamps and filenames
        pConfig->m_DataflowAttributes.getAttributeData( "file", m_tsFile );


        // start loading images in a thread
        if ( !m_pLoadThread )
            m_pLoadThread.reset( new boost::thread( boost::bind( &K4APlayerComponentImage::loadImages, this ) ) );

        // before starting the component let the tread do some work on loading
        Util::sleep( 300 );
	}


	
	void loadImages()
	{
		boost::filesystem::path tsFile( m_tsFile );
		if( !boost::filesystem::exists( tsFile ) )
			UBITRACK_THROW( "K4A mkv file not found: \"" + m_tsFile + "\"");


        k4a_result_t result = K4A_RESULT_SUCCEEDED;
        result = k4a_playback_open(m_tsFile.c_str(), &m_k4a_record.handle);

        if (result != K4A_RESULT_SUCCEEDED)
			UBITRACK_THROW( "Could not open file \"" + m_tsFile  + "\"." );

		result = k4a_playback_get_record_configuration(m_k4a_record.handle, &m_k4a_record.record_config);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            UBITRACK_THROW( "Failed to get record configuration for file: \"" + m_tsFile  + "\"." );
        }

        if (m_k4a_record.record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER)
        {
            LOG4CPP_INFO( logger, "Opened master recording file: \"" << m_tsFile << "\"." );
            m_isMaster = true;
        }
        else if (m_k4a_record.record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE)
        {
            LOG4CPP_INFO( logger,"Opened subordinate recording file: " << m_tsFile);
        }
        else
        {
            // not necessarily an error in our case as unsychronized image streams are allowed
            LOG4CPP_WARN( logger, "Recording file was not recorded in master/sub mode: " << m_tsFile);
        }

        // Read the first capture of each recording into memory.
        k4a_stream_result_t stream_result = k4a_playback_get_next_capture(m_k4a_record.handle, &m_k4a_record.capture);
        if (stream_result == K4A_STREAM_RESULT_EOF)
        {
            LOG4CPP_ERROR( logger, "Recording file is empty: " << m_tsFile);
            result = K4A_RESULT_FAILED;
        }
        else if (stream_result == K4A_STREAM_RESULT_FAILED)
        {
            LOG4CPP_ERROR( logger, "Failed to read first capture from file: " << m_tsFile);
            result = K4A_RESULT_FAILED;
        }

        int imageCount = 0;

		while( result == K4A_RESULT_SUCCEEDED )
        {
            // handle and store it
            // isConnected does currently not work.
            // the thread is started too soon, if the start() function is used to start the thread after the connections are made then the main thread does not work
			//if(m_outPortRGB.isConnected())
			{
                k4a_image_t image = k4a_capture_get_color_image(m_k4a_record.capture);
                Measurement::ImageMeasurement e = handleK4AImage(image);
                if(!e.invalid())
                    m_eventsRGB.push_back( e );
                k4a_image_release(image);
			}
            //if(m_outPortDepth.isConnected())
            {
                k4a_image_t image = k4a_capture_get_depth_image(m_k4a_record.capture);
                Measurement::ImageMeasurement e = handleK4AImage(image);
                if(!e.invalid())
                    m_eventsDepth.push_back( e );
                k4a_image_release(image);
            }
            //if(m_outPortIR.isConnected())
            {
                k4a_image_t image = k4a_capture_get_ir_image(m_k4a_record.capture);
                Measurement::ImageMeasurement e = handleK4AImage(image);
                if(!e.invalid())
                    m_eventsIR.push_back( e );
                k4a_image_release(image);
            }

			




			// Advance recording
            k4a_stream_result_t stream_result = k4a_playback_get_next_capture(m_k4a_record.handle, &m_k4a_record.capture);
            if (stream_result == K4A_STREAM_RESULT_EOF){
                // all images loaded
                result = K4A_RESULT_FAILED;
            }else if (stream_result == K4A_STREAM_RESULT_FAILED)
            {
                LOG4CPP_ERROR( logger,"Failed to read next capture from file: %s" << m_tsFile);
                result = K4A_RESULT_FAILED;
            }
            imageCount++;

		}

        k4a_playback_close(m_k4a_record.handle);
		LOG4CPP_INFO( logger, "Done loading " << imageCount << " images defined in file \"" << m_tsFile << "\"." );
		
	}

	Measurement::Timestamp getFirstTime() const
    {
		if ( !m_eventsRGB.empty() )
			return m_eventsRGB.begin()->time() + 1000000LL * m_offset;
		else
			return 0;
	}

	/** return time of the next measurement to be played or 0 if no events */
	Measurement::Timestamp getNextTime( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		if( !m_eventsRGB.empty() )
			return recordTimeToReal( m_eventsRGB.begin()->time(), recordStart, playbackStart );
		else
			return 0;
	}

	/** send the next event */
	void sendNext( Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{
		if( !m_eventsRGB.empty() )
		{
			m_outPortRGB.send( Measurement::ImageMeasurement( recordTimeToReal( m_eventsRGB.begin()->time(), recordStart, playbackStart ), m_eventsRGB.front() ) );
            m_eventsRGB.pop_front();
		}
        if( !m_eventsDepth.empty() )
        {
            m_outPortDepth.send( Measurement::ImageMeasurement( recordTimeToReal( m_eventsDepth.begin()->time(), recordStart, playbackStart ), m_eventsDepth.front() ) );
            m_eventsDepth.pop_front();
        }
        if( !m_eventsIR.empty() )
        {
            m_outPortIR.send( Measurement::ImageMeasurement( recordTimeToReal( m_eventsIR.begin()->time(), recordStart, playbackStart ), m_eventsIR.front() ) );
            m_eventsIR.pop_front();
        }
	}

	bool isMaster()
	{
	    return m_isMaster;
	}

protected:

	/**
	 * converts a recorded time to a real time.
	 * @param t timestamp to convert
	 * @param recordStart time the recording was started
	 * @param realStart time the playback was started
	 */
	Measurement::Timestamp recordTimeToReal( Measurement::Timestamp t, Measurement::Timestamp recordStart, Measurement::Timestamp playbackStart )
	{ return static_cast< Measurement::Timestamp >( ( t - recordStart + m_offset * 1000000LL ) / m_speedup + playbackStart ); }

	/** file which defines timestamps and images */
	std::string m_tsFile;
    recording_t m_k4a_record;
    bool m_isMaster;
	
	/** offset if the event should be sent at some other time than its timestamp */
	int m_offset;

	/** speedup factor */
	double m_speedup;
	
	/** a thread for loading images */
	boost::shared_ptr< boost::thread > m_pLoadThread;

	/** output port */
	Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPortRGB;
    Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPortDepth;
    Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPortIR;

	/** queue for the images being loaded, @todo add mutex for accessing m_events */
	std::deque< Measurement::ImageMeasurement > m_eventsRGB;
    std::deque< Measurement::ImageMeasurement > m_eventsDepth;
    std::deque< Measurement::ImageMeasurement > m_eventsIR;

	Measurement::ImageMeasurement handleK4AImage(k4a_image_t image) {
        cv::Mat img;
        Vision::Image::ImageFormatProperties imgFormat;
        uint64_t ts = k4a_image_get_device_timestamp_usec(image)*1000LL;

        try
        {

            k4a_image_format_t k4aFormat =  k4a_image_get_format(image);

            int w = k4a_image_get_width_pixels(image );
            int h = k4a_image_get_height_pixels(image );
            uint8_t* bufferp = k4a_image_get_buffer(image);
            size_t bufferSize = k4a_image_get_size(image);

            if (k4aFormat == K4A_IMAGE_FORMAT_COLOR_BGRA32) {
                img = cv::Mat(cv::Size(w, h), CV_8UC4, static_cast<void *>(bufferp), cv::Mat::AUTO_STEP).clone();

                imgFormat.imageFormat = Vision::Image::BGRA;
                imgFormat.depth = CV_8U;
                imgFormat.channels = 4;
                imgFormat.matType = CV_8UC4;
                imgFormat.bitsPerPixel = 32;
                imgFormat.origin = 0;
                imgFormat.align = 4;

            } else if (k4aFormat == K4A_IMAGE_FORMAT_COLOR_MJPG) {
                cv::Mat rawData(1,bufferSize,CV_8SC1,static_cast<void *>(bufferp));
                img = cv::imdecode(rawData, cv::IMREAD_UNCHANGED );

                imgFormat.imageFormat = Vision::Image::BGR;
                imgFormat.depth = CV_8U;
                imgFormat.channels = 3;
                imgFormat.matType = CV_8UC3;
                imgFormat.bitsPerPixel = 24;
                imgFormat.origin = 0;
                imgFormat.align = 4;


            } else if (k4aFormat == K4A_IMAGE_FORMAT_DEPTH16) {
                int stride = k4a_image_get_stride_bytes(image);
                img = cv::Mat(cv::Size(w, h), CV_16UC1, static_cast<void *>(bufferp), static_cast<size_t>(stride)).clone();

                imgFormat.imageFormat = Vision::Image::DEPTH;
                imgFormat.depth = CV_16U;
                imgFormat.channels = 1;
                imgFormat.matType = CV_16UC1;
                imgFormat.bitsPerPixel = 16;
                imgFormat.origin = 0;
                imgFormat.align = 4;


            } else if (k4aFormat == K4A_IMAGE_FORMAT_IR16) {
                int stride = k4a_image_get_stride_bytes(image);
                img = cv::Mat(cv::Size(w, h), CV_16UC1, static_cast<void *>(bufferp), static_cast<size_t>(stride)).clone();

                imgFormat.imageFormat = Vision::Image::LUMINANCE;
                imgFormat.depth = CV_16U;
                imgFormat.channels = 1;
                imgFormat.matType = CV_16UC1;
                imgFormat.bitsPerPixel = 16;
                imgFormat.origin = 0;
                imgFormat.align = 4;

            } else {
                LOG4CPP_ERROR(  logger,  "Received color frame with unexpected format: " << k4aFormat);
                return Measurement::ImageMeasurement(0);
            }

        }
        catch( std::exception& e )
        {
            LOG4CPP_ERROR( logger, "loading image failed: " << e.what() );
            return Measurement::ImageMeasurement(0);
        }
        catch ( ... )
        {
            LOG4CPP_ERROR(logger, "loading image file failed: other reason");
            return Measurement::ImageMeasurement(0);
        }

        if( img.total() == 0 )
        {
            LOG4CPP_ERROR( logger, "loading image file failed. no data" );
            return Measurement::ImageMeasurement(0);
        }


        // convert loaded image into the required pImage class
        boost::shared_ptr< Vision::Image > pImage( new Vision::Image( img, imgFormat ) );
        // Building the event and packing timestamp and image into it
        Measurement::ImageMeasurement e( static_cast< Measurement::Timestamp>(  ts ), pImage );
        return e;
	}
};


void K4APlayerModule::mainloop()
{
	// find time of first recorded event in queue
	Measurement::Timestamp recordStart( 0 );
	{
		ComponentList l( getAllComponents() );
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			Measurement::Timestamp t = (*it)->getFirstTime();
			if ( t && ( recordStart == 0 || t < recordStart ) )
				recordStart = t;
		}
	}
	LOG4CPP_DEBUG( logger, "recordStart = " << recordStart );

	// delay start for 3s to allow other components to start
	Measurement::Timestamp playbackStart( Measurement::now() + 2000000000LL );
	LOG4CPP_DEBUG( logger, "playbackStart = " << playbackStart );

	// find playback time of first event in queue
	Measurement::Timestamp nextEventTime( 0 );
	{
		ComponentList l( getAllComponents() );
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			Measurement::Timestamp t = (*it)->getNextTime( recordStart, playbackStart );
			if ( t && ( nextEventTime == 0 || t < nextEventTime ) )
				nextEventTime = t;
		}
	}
	LOG4CPP_INFO( logger, "Starting main loop" );
	LOG4CPP_DEBUG( logger, "Starting main loop at " << nextEventTime );

	// main loop
	while ( !m_bStop && nextEventTime )
	{
		LOG4CPP_DEBUG( logger, "nextEventTime = " << nextEventTime );

		// sleep until next event
		Measurement::Timestamp now( Measurement::now() );
		long long int sleepdur( nextEventTime - now );
		if ( sleepdur > 0 )
		{
			LOG4CPP_DEBUG( logger, "sleeping " << sleepdur / 1000000 << "ms" );
			Util::sleep( int( sleepdur / 1000000 ), int( sleepdur % 1000000 ) );
		}

		now = Measurement::now();
		nextEventTime = 0;

		// iterate all components
		ComponentList l( getAllComponents() );
		int count = 0;
		for ( ComponentList::iterator it = l.begin(); it != l.end(); it++ )
		{
			// send all events due until now
			Measurement::Timestamp t = (*it)->getNextTime( recordStart, playbackStart );
			
			for ( ; t && t <= now; )
			{
				//LOG4CPP_DEBUG( logger,  (*it)->getName() << "sending " << t  );
				(*it)->sendNext( recordStart, playbackStart );
				t = (*it)->getNextTime( recordStart, playbackStart );
				if ( !t ){
					LOG4CPP_NOTICE( logger, (*it)->getName() << " reached end of recording" );
					(*it)->sendFinished();
				}
				count++;
			}
			

			// update next event time
			if ( t && ( nextEventTime == 0 || t < nextEventTime ) )
				nextEventTime = t;
		}
		//LOG4CPP_NOTICE(logger, "count of send events:"<< count);
	}
}


// has to be here, after all class declarations
boost::shared_ptr< K4APlayerComponentBase > K4APlayerModule::createComponent( const std::string& type, const std::string& name,
	boost::shared_ptr< Graph::UTQLSubgraph > pConfig, const K4APlayerModule::ComponentKey& key, K4APlayerModule* pModule )
{
    if ( type == "K4APlayerImage" )
		return boost::shared_ptr< K4APlayerComponentBase >( new K4APlayerComponentImage( name, pConfig, key, pModule ) );


	UBITRACK_THROW( "Class " + type + " not supported by player module" );
}


} } // namespace Ubitrack::Drivers


UBITRACK_REGISTER_COMPONENT( Ubitrack::Dataflow::ComponentFactory* const cf ) {
	// create list of supported types
	std::vector< std::string > K4APlayerComponents;

	K4APlayerComponents.push_back( "K4APlayerImage" );


	cf->registerModule< Ubitrack::Drivers::K4APlayerModule > ( K4APlayerComponents );
}
