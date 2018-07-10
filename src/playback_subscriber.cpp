#include <portaudio_transport/playback_subscriber.h>

PlaybackSubscriber::PlaybackSubscriber(int stream_channel_count, int stream_frame_size, double stream_sample_frequency, int device_output_channels, double device_sample_frequency) {
    // Initialize stream properties
    stream_channel_count_ = stream_channel_count;
    stream_frame_size_ = stream_frame_size;
    stream_sample_frequency_ = stream_sample_frequency;

    // Initialize device properties
    device_output_channels_ = device_output_channels;
    device_sample_frequency_ = device_sample_frequency;

    // Map stream properties to device properties
    // TODO: handle these, maybe map with yaml file, dynamic reconfigure or perform automatic mapping
    if (stream_channel_count_ != device_output_channels_) {
        ROS_WARN_STREAM("Mismatch between stream channels and output device channels. Cannot handle this yet!  " << stream_channel_count_ << " <-stream : device-> " << device_output_channels_);
    }
    if (stream_sample_frequency_ != device_sample_frequency_) {
        ROS_WARN_STREAM("Mismatch between stream channels and output device channels. Cannot handle this yet!  " << stream_sample_frequency_ << " <-stream : device-> " << device_sample_frequency_);
    }

    // Initialize multidimensional sample queue with channels count
    sample_queues_.resize(stream_channel_count_);
    sample_queue_max_ = (stream_sample_frequency_*3.0); //keep a max of X seconds in buffer
    for (int c_channel = 0; c_channel < sample_queues_.size(); c_channel++) {
        sample_queues_[c_channel] = std::deque<float>();
    }
}

PlaybackSubscriber::~PlaybackSubscriber() {}

void PlaybackSubscriber::AddToBuffer(std::vector<portaudio_transport::AudioChannel> audio_channels) {
    for (int c_channel = 0; c_channel < audio_channels.size(); c_channel++) {
        for (int c_sample = 0 ; c_sample < audio_channels[0].frame_data.size(); c_sample++) {
            // For each channel and each sample in callback data, push data into global sample queue.
            // TODO: push back array instead of single data value
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
    unsigned long c_output = 0;

    // If callback has NULL buffer, exit
    // TODO: Can this work with paContinue and filling the buffer with zeroes
    if (pOutputBuffer == NULL) {
        ROS_ERROR("PlaybackSubscriber::PlaybackCallback was NULL!");
        return paComplete;
    }

    // Walk through the playback size and fill the playback buffer (pData) with samples from the front of the queue
    // TODO: Handle buffer overflow and buffer underflow
    while (c_output < iFramesPerBuffer) {
        // Handle empty global queue
        // TODO: handle other empty queues but the first one
        if (sample_queues_[0].size() <= 1) {
            ROS_WARN_STREAM("sample_queue_ empty " << c_output); // TODO: Silently handle this
            // Fill out buffer with zeros
            while (c_output < iFramesPerBuffer) {
                for (int i = 0; i < device_output_channels_; i++) {
                    pData[i][c_output] = (float) 0;
                }
                c_output++;
            }
            return paContinue; // TODO: keep going, handle overfill
            //return paComplete;
        }
        // This actually fills the data into the playback buffer
        for (int c_channel = 0; c_channel < device_output_channels_; c_channel++) {
            pData[c_channel][c_output] = (float) sample_queues_[c_channel].front();
            sample_queues_[c_channel].pop_front();
        }
        c_output++;
    }
    return paContinue;
}