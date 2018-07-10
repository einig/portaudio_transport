//! portaudio_transport/recording_publisher allows publishing of live recordings via ROS topics.
/*!
    The recording publisher publishes live recorded data to the 'portaudio_transport' topic.
    Published data format is float32. Recording parameters such as sample_frequency, audio_channels
    and frame_size or frame_rate must be specified. The periodic file write function can be
    configured as well. This class is the recording buffer which reads data from the sound card
    input stream and publishes it to the ROS topic.
*/
#ifndef _RECORDING_PUBLISHER_H_
#define _RECORDING_PUBLISHER_H_
#include "ros/ros.h"

#include <vector>
#include <iostream>
#include <fstream>

#include <sndfile.hh>
#include <cstdio>
#include <cstring>

#include "boost/date_time/gregorian/gregorian.hpp"

#include "portaudiocpp/PortAudioCpp.hxx"

#include "portaudio_transport/AudioChannel.h"
#include "portaudio_transport/AudioTransport.h"

class RecordingPublisher {
    public:
        //! Constructor
        /*! Requires info about stream, device and file properties */
        RecordingPublisher(ros::Publisher audio_publisher, int audio_channels, int sample_frequency, int frame_rate, int frame_size, std::string filepath, int file_write_rate);
        ~RecordingPublisher();

        //! Function to read samples from the sound card callback and publish it to the ROS ropic
        int RecordCallback(const void* pInputBuffer,
                            void* pOutputBuffer,
                            unsigned long iFramesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags);

        //! Function to clear the internal sample buffer after writing to file or to reset the temporary recording
        void Clear();

        //! Function to write the internal sample buffer to file
        void WriteToFile();

        //! Function to express the sample format enum (float32) as a string.
        // TODO: deprecate this function?
        std::string SampleFormatToString();

    private:
        ros::Publisher                      audio_publisher_;
        int                                 audio_channels_;
        int                                 frame_size_;
        int                                 frame_rate_;
        int                                 sample_frequency_;
        portaudio::SampleDataFormat         sample_format_;
        std::vector<float>                  sample_vector_;
        std::vector<float>                  sample_vector_write_;
        portaudio_transport::AudioTransport transport_;
        std::string                         filename_ ;
        int                                 file_write_rate_;
};

#endif