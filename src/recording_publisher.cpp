#include <portaudio_transport/recording_publisher.h>

RecordingPublisher::RecordingPublisher(ros::Publisher audio_publisher, int audio_channels, int sample_frequency, int frame_rate) {
    audio_publisher_  = audio_publisher;
    audio_channels_   = audio_channels;
    sample_frequency_ = sample_frequency;
    frame_rate_     = frame_rate;
    frame_size_     = sample_frequency_/frame_rate_;
    transport_.channels.clear();
    transport_.channel_count  = audio_channels_;
    transport_.frame_size   = frame_size_;
    transport_.sample_frequency = sample_frequency_;
    transport_.channels.resize(audio_channels_);
    for (int c_channel = 0; c_channel < audio_channels_; c_channel++) {
        transport_.channels[c_channel].frame_data.resize(frame_size_);
    }
    // Initialize sample vector with audio_channels_*frame_size_
    // TODO: change initialization to sensible size once periodic writeback is working
    //sample_vector_.resize(audio_channels_);
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
    for (int c_channel = 0; c_channel < audio_channels_; c_channel++) {
        sample_vector_[c_channel].clear();
    }
}

// Dump the samples to a raw file
void RecordingPublisher::WriteToFile(const std::string& filepath) {
    float iSample;
    std::stringstream filename;

    time_t t = time(0);
    struct tm * now = localtime( & t );
    char filebase [80];
    strftime (filebase,80,"%F_%H-%M-%S",now);

    for (int c_channel = 0; c_channel < audio_channels_; c_channel++) {
        //std::string filename = filepath + filebase + "-channel-" + std::to_string(c_channel) + ".wav";
        std::string filename = "/tmp/test-channel-" + std::to_string(c_channel) + ".wav";
        std::fstream fout(filename.c_str(), std::ios::out|std::ios::binary);
        for (std::vector<float>::iterator iter = sample_vector_[c_channel].begin(); iter != sample_vector_[c_channel].end(); iter++)
        {
            iSample = (float) *iter;
            fout.write((char *) &iSample, sizeof(float));
        }
        fout.close();
    }
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