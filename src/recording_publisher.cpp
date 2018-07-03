#include <portaudio_transport/recording_publisher.h>

RecordingPublisher::RecordingPublisher(ros::Publisher audio_publisher, int audio_channels, int sample_frequency, int frame_rate, int frame_size, std::string filepath, int file_write_rate) {
    audio_publisher_  = audio_publisher;
    audio_channels_   = audio_channels;
    sample_frequency_ = sample_frequency;
    file_write_rate_  = file_write_rate;
    if (frame_rate != 0) {
        frame_rate_   = frame_rate;
        frame_size_   = sample_frequency_/frame_rate_;
    } else if (frame_size != 0) {
        frame_size_   = frame_size;
        frame_rate_   = sample_frequency_/frame_size_;
    }
    transport_.channels.clear();
    transport_.channel_count  = audio_channels_;
    transport_.frame_size   = frame_size_;
    transport_.sample_frequency = sample_frequency_;
    transport_.channels.resize(audio_channels_);
    for (int c_channel = 0; c_channel < audio_channels_; c_channel++) {
        transport_.channels[c_channel].frame_data.resize(frame_size_);
    }

    // Initialize the file name for writing to keep editing the same file
    time_t t = time(0);
    struct tm *now = localtime(&t);
    char filebase [80];
    strftime(filebase,80,"%F_%H-%M-%S",now);
    filename_ = filepath + filebase + ".wav";

    // Reserve sample vector with audio_channels_* frame in one write cycle
    sample_vector_.reserve(audio_channels_*sample_frequency_/file_write_rate_*2);
    sample_vector_write_.reserve(audio_channels_*sample_frequency_/file_write_rate_*2);
}

RecordingPublisher::~RecordingPublisher() {}

int RecordingPublisher::RecordCallback(const void* pInputBuffer,
                                void* pOutputBuffer,
                                unsigned long iFramesPerBuffer,
                                const PaStreamCallbackTimeInfo* timeInfo,
                                PaStreamCallbackFlags statusFlags) {
    float** pData = (float**) pInputBuffer;

    if (iFramesPerBuffer != frame_size_) {
        ROS_ERROR("RecordingPublisher::RecordCallback: frame size mismatch!");
    }
    if (pInputBuffer == NULL) {
        ROS_WARN("RecordingPublisher::RecordCallback: input buffer was NULL!");
        return paContinue;
    }

    // Copy all the frames over to our internal vector of samples
    for (int c_sample = 0; c_sample < frame_size_; c_sample++) {
        for (int c_channel = 0; c_channel < audio_channels_; c_channel++) {
            sample_vector_.push_back(pData[c_channel][c_sample]);
            transport_.channels[c_channel].frame_data[c_sample] = pData[c_channel][c_sample];
        }
    }

    if (audio_publisher_ != NULL) {
        audio_publisher_.publish(transport_);
    }
    return paContinue;
}
// Clear out any data in the buffer and prepare for a new recording.
void RecordingPublisher::Clear() {
    sample_vector_.clear();
}

// Dump the samples to a raw file
void RecordingPublisher::WriteToFile() {
    sample_vector_write_ = std::move(sample_vector_);
    SndfileHandle file;
    file = SndfileHandle(filename_, SFM_RDWR, SF_FORMAT_WAV | SF_FORMAT_FLOAT, audio_channels_, sample_frequency_);
    file.write(sample_vector_write_.data(), (sf_count_t) sample_vector_write_.size());
}

std::string RecordingPublisher::SampleFormatToString() {
    switch(sample_format_) {
    case portaudio::INVALID_FORMAT:
        return "INVALID FORMAT";
    case portaudio::FLOAT32:
        return "float32";
    case portaudio::INT32:
        return "int32";
    case portaudio::INT24:
        return "int24";
    case portaudio::INT16:
        return "int16";
    case portaudio::INT8:
        return "int8";
    case portaudio::UINT8:
        return "uint8";
    }
}