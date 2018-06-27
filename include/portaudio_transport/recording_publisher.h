
// Class that can be used to record, playback and save audio data
// to a file.  It is designed to be a producer/consumer with the
// portaudio library.
//
// This class expects mono audio in INT16 (short) format.
//
// Copyright 2007 by Keith Vertanen.

#ifndef _AUDIO_BUFFER_H_
#define _AUDIO_BUFFER_H_
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
        RecordingPublisher(ros::Publisher audio_publisher, int audio_channels, int sample_frequency, int frame_rate);
        ~RecordingPublisher();

        int RecordCallback(const void* pInputBuffer,
                            void* pOutputBuffer,
                            unsigned long iFramesPerBuffer,
                            const PaStreamCallbackTimeInfo* timeInfo,
                            PaStreamCallbackFlags statusFlags);
        void Clear();
        void WriteToFile(const std::string& strFilename);
        std::string SampleFormatToString();

    private:
        ros::Publisher                      audio_publisher_;
        int                                 audio_channels_;
        int                                 frame_size_;
        int                                 frame_rate_;
        int                                 sample_frequency_;
        portaudio::SampleDataFormat         sample_format_;
        std::vector<float>                  sample_vector_;
        portaudio_transport::AudioTransport transport_;
};

#endif