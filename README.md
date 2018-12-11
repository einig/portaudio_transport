# portaudio_transport
ROS node based on portaudio and libsndfile to transport live sound recording via ROS topics.

## Required components:
according to [this](https://ubuntuforums.org/showthread.php?t=1680154),
    install packages in this order will not remove any packages
 - `sudo apt install libsndfile1-dev`
 - `sudo apt install libjack-jackd2-dev`
 - `sudo apt install portaudio19-dev`

## Common problems:
 - ensure user is in audio group
