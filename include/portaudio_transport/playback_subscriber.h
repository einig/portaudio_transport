//! portaudio_transport/playback_subscriber allows playback of live sound data via ROS topics.
/*!
    The playback subscriber plays back sound data from the 'portaudio_transport' topic. Accepted data
    format is float32. Other parameters such as sample_frequency and frame_size will be read from the
    audio stream and mapped to the output device. This class is the playback buffer which reads data
    from the topic and provides this data to the sound card output stream.
*/
#ifndef _PLAYBACK_SUBSCRIBER_H_
#define _PLAYBACK_SUBSCRIBER_H_
#include "ros/ros.h"

#include <vector>
#include <deque>

#include "portaudiocpp/PortAudioCpp.hxx"

#include "portaudio_transport/AudioChannel.h"
#include "portaudio_transport/AudioTransport.h"

class PlaybackSubscriber {
    public:
        //! Constructor
        /*! Requires info about stream and device properties to map the stream data to the output channels. */
        PlaybackSubscriber(int stream_channel_count, int stream_frame_size, double stream_sample_frequency, int device_output_channels, double device_sample_frequency);
        ~PlaybackSubscriber();

        //! Function to append sound data to buffer from topic
        void AddToBuffer(std::vector<portaudio_transport::AudioChannel> audio_channels);

        //! Function to get sound data from buffer for playback.
        int PlaybackCallback(const void* pInputBuffer,
                            void* pOutputBuffer,
                            unsigned long iFramesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags);

    private:
        int                                 stream_channel_count_;
        int                                 stream_frame_size_;
        double                              stream_sample_frequency_;

        int                                 device_output_channels_;
        double                              device_sample_frequency_;

        portaudio::SampleDataFormat         sample_format_;
        std::vector<std::deque<float>>      sample_queues_;
        long int                            sample_queue_max_;
};

#endif