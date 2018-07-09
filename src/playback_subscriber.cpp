#include <portaudio_transport/playback_subscriber.h>

PlaybackSubscriber::PlaybackSubscriber(int stream_channel_count, int stream_frame_size, double stream_sample_frequency, int device_output_channels, double device_sample_frequency) {
    stream_channel_count_ = stream_channel_count;
    stream_frame_size_ = stream_frame_size;
    stream_sample_frequency_ = stream_sample_frequency;

    device_output_channels_ = device_output_channels;
    device_sample_frequency_ = device_sample_frequency;

    // TODO: handle these
    // TODO: more intelligent handling of mismatching stream sampling rate and stream channel count
    if (stream_channel_count_ != device_output_channels_) {
        ROS_WARN_STREAM("Mismatch between stream channels and output device channels. Cannot handle this yet!  " << stream_channel_count_ << " <-stream : device-> " << device_output_channels_);
    }
    if (stream_sample_frequency_ != device_sample_frequency_) {
        ROS_WARN_STREAM("Mismatch between stream channels and output device channels. Cannot handle this yet!  " << stream_sample_frequency_ << " <-stream : device-> " << device_sample_frequency_);
    }
    sample_queues_.resize(stream_channel_count_);
    for (int c_channel = 0; c_channel < sample_queues_.size(); c_channel++) {
        sample_queues_[c_channel] = std::deque<float>();
    }
}

PlaybackSubscriber::~PlaybackSubscriber() {}

void PlaybackSubscriber::AddToBuffer(std::vector<portaudio_transport::AudioChannel> audio_channels) {
    for (int c_channel = 0; c_channel < audio_channels.size(); c_channel++) {
        for (int c_sample = 0 ; c_sample < audio_channels[0].frame_data.size(); c_sample++) {
            sample_queues_[c_channel].push_back(audio_channels[c_channel].frame_data[c_sample]);
        }
    }
}

int PlaybackSubscriber::PlaybackCallback(const void* pInputBuffer,
                                void* pOutputBuffer,
                                unsigned long iFramesPerBuffer,
                                const PaStreamCallbackTimeInfo* timeInfo,
                                PaStreamCallbackFlags statusFlags) {
    float** pData   = (float**) pOutputBuffer;
    unsigned long iOutput = 0;

    // TODO: Handle buffer overflow and buffer underflow
    if (pOutputBuffer == NULL) {
        ROS_ERROR("PlaybackSubscriber::PlaybackCallback was NULL!");
        return paComplete;
    }

    while (iOutput < iFramesPerBuffer) {
        if (sample_queues_[0].size() <= 1) {
            ROS_WARN_STREAM("sample_queue_ empty " << iOutput); // TODO: Silently handle this
            // Fill out buffer with zeros
            while (iOutput < iFramesPerBuffer) {
                for (int i = 0; i < device_output_channels_; i++) {
                    pData[i][iOutput] = (float) 0;
                }
                iOutput++;
            }
            return paContinue; // TODO: keep going, handle overfill
            //return paComplete;
        }
        for (int c_channel = 0; c_channel < device_output_channels_; c_channel++) {
            pData[c_channel][iOutput] = (float) sample_queues_[c_channel].front();
            sample_queues_[c_channel].pop_front();
       }
        iOutput++;
    }
    return paContinue;
}