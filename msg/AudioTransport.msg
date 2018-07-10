# Hold information about timing. Timing info should be gathered from the sound card instead of the ROS system.
Header header

# Hold the desired channel_count for the transport message. Can be used to verify the integrity of the audio data.
uint8 channel_count

# Holds the desired frame_size for the transport message. Can be used to verify the integrity of each audio channel.
uint16 frame_size

# Hold the sample_frequency used by the microphones. Required for playback and analysis.
uint16 sample_frequency

# Array of audio channels. Holds the recorded data and the location for each microphone.
portaudio_transport/AudioChannel[] channels