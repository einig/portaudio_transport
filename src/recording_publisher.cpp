#include <portaudio_transport/recording_publisher.h>

RecordingPublisher::RecordingPublisher(ros::Publisher audio_publisher, int audio_channels, int sample_frequency, int frame_rate, int frame_size, std::string filepath, int file_write_rate) {
    // Initialize stream properties and publisher
    audio_publisher_  = audio_publisher;
    audio_channels_ = audio_channels;
    sample_frequency_ = sample_frequency;
    file_write_rate_ = file_write_rate;
    // Calculate rate or size if only one is specified
    // TODO: check integrity if both are specified
    if (frame_rate != 0) {
        frame_rate_   = frame_rate;
        frame_size_   = sample_frequency_/frame_rate_;
    } else if (frame_size != 0) {
        frame_size_   = frame_size;
        frame_rate_   = sample_frequency_/frame_size_;
    }
    // Initialize the transport message used for publishing
    transport_.channels.clear();
    transport_.channel_count  = audio_channels_;
    transport_.frame_size = frame_size_;
    transport_.sample_frequency = sample_frequency_;
    // Resize the channel array and each channel array to audio_channels and frame_size
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

    // Reserve sample vector with audio_channels_*(samples in one write cycle)
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

    // Check for integrity of the recording callback
    // TODO: handle errors better
    if (iFramesPerBuffer != frame_size_) {
        ROS_ERROR("RecordingPublisher::RecordCallback: frame size mismatch!");
    }
    if (pInputBuffer == NULL) {
        ROS_WARN("RecordingPublisher::RecordCallback: input buffer was NULL!");
        return paContinue;
    }

    // Copy all the frames over to the internal vector of samples and to the transport message
    for (int c_sample = 0; c_sample < frame_size_; c_sample++) {
        for (int c_channel = 0; c_channel < audio_channels_; c_channel++) {
            sample_vector_.push_back(pData[c_channel][c_sample]); // append data
            // TODO: what happens when there was a mismatch, is old data kept?
            transport_.channels[c_channel].frame_data[c_sample] = pData[c_channel][c_sample]; // replace data
        }
    }

    // Publish the audio_transport if it exists
    if (audio_publisher_ != NULL) {
        audio_publisher_.publish(transport_);
    }
    return paContinue;
}

void RecordingPublisher::Clear() {
    // Clear out any data in the buffer and prepare for a new recording.
    sample_vector_.clear();
}

void RecordingPublisher::WriteToFile() {
    // Move the recorded data to a temporary vector
    // TODO: malloc required?
    sample_vector_write_ = std::move(sample_vector_);
    SndfileHandle file;
    // Create a libsndfile handle to write the data (RDWR means data will be appended to existing file)
    file = SndfileHandle(filename_, SFM_RDWR, SF_FORMAT_WAV | SF_FORMAT_FLOAT, audio_channels_, sample_frequency_);
    // Write data to file.
    file.write(sample_vector_write_.data(), (sf_count_t) sample_vector_write_.size());
}

// TODO: deprecate
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