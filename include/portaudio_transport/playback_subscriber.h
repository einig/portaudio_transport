
// Class that can be used to record, playback and save audio data
// to a file.  It is designed to be a producer/consumer with the
// portaudio library.
//
// This class expects mono audio in INT16 (short) format.
//
// Copyright 2007 by Keith Vertanen.

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
        PlaybackSubscriber(int stream_channel_count, int stream_frame_size, double stream_sample_frequency, int device_output_channels, double device_sample_frequency);
        ~PlaybackSubscriber();

        void AddToBuffer(std::vector<portaudio_transport::AudioChannel> audio_channels);

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
        bool tmp;
};

#endif