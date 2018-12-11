#include "ros/ros.h"

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include <portaudio_transport/recording_publisher.h>

#include <portaudiocpp/PortAudioCpp.hxx>

#include "portaudio_transport/AudioTransport.h"

//! Print info about input devices
/*! Print a list of available input devices and their properties, such as index, name, output channels and samplerate */
void PrintInputDevices(portaudio::System& audioSys) {
        std::stringstream available_devices;
        available_devices << "Available Recording Devices" << std::endl;
        available_devices << "\t*---*-------------------------------------------------------------*---------*------------*" << std::endl;
        available_devices << "\t| # | Name                                                        | Inputs  | Samplerate |" << std::endl;
        available_devices << "\t*---*-------------------------------------------------------------*---------*------------*" << std::endl;
        boost::format entry_line = boost::format("|%3i| %|-60|| %|=8|| %|=11|| ");
        for (portaudio::System::DeviceIterator device = audioSys.devicesBegin(); device != audioSys.devicesEnd(); ++device) {
            std::string deviceOptions = "";
            if (device->maxInputChannels() < 1) { continue; }
            if (device->isSystemDefaultInputDevice()) { deviceOptions += "!"; }
            if (device->isHostApiDefaultInputDevice()) { deviceOptions += "*"; }
            available_devices << "\t" << entry_line % device->index() % device->name() % device->maxInputChannels() % device->defaultSampleRate() << deviceOptions << std::endl;
        }
        available_devices << "\t*---*-------------------------------------------------------------*---------*------------*" << std::endl;
        available_devices << "\t      is default input device: system->!  hostAPI->*";
        ROS_INFO_STREAM(available_devices.str());
}

//! Main function of portaudio_transport_publisher node
/*!
    The function reads parameters from the commandline, initializes the
    recording device and creates the recording buffer.
*/
int main(int argc, char **argv) {
    ros::init(argc, argv, "portaudio_transport_publisher");
    ros::NodeHandle nh("~");
    ros::Publisher pub = nh.advertise<portaudio_transport::AudioTransport>("/portaudio_transport", 1);
    // Ensure correct size of float data type
    assert(CHAR_BIT * sizeof (float) == 32);

    // Initialize stream and device properties
    std::string input_device_name = "";
    int input_device_id = -1;
    int frame_rate = 0;
    int frame_size = 0;
    int max_channels = 0;
    int file_write_rate = 10;
    // TODO: should the file path be configurable?
    std::string file_path = "/tmp/portaudio_transport_";

    // Parse parameters from command line
    // _input_device can either be a number for the index or part of the name of the device. The first match (by index) will be used.
    if (nh.getParam("input_device", input_device_name)) {
        ROS_INFO("Matching input device name for: %s" , input_device_name.c_str());
        boost::algorithm::to_lower(input_device_name);
    } else if (nh.getParam("input_device", input_device_id)) {
        ROS_INFO("Using input device: %d", input_device_id);
    } else {
        ROS_WARN("_input_device not specified, using system default");
        input_device_name = "default";
    }
    // _frame_rate can be specified and must be non-zero
    if (nh.getParam("frame_rate", frame_rate)) {
        ROS_INFO("Using frame rate: %d", frame_rate);
    }
    // If _frame_rate was not parsed, parse the _frame_size, otherwise ignore the frame_size
    if (frame_rate == 0) {
        if (nh.getParam("frame_size", frame_size)) {
            ROS_INFO("Using frame size: %d", frame_size);
        }
    } else {
        ROS_WARN("Frame rate already specified, ignoring input for frame size");
    }
    // _max_channels limits the recording channels, can be used to reduce data flow if not all microphones are plugged in.
    // Input channels with higher index will not be used.
    if (nh.getParam("max_channels", max_channels)) {
        ROS_INFO("Limiting input channels to %d", max_channels);
    }
    // _file_write_rate defines how often per second the audio data will be written to file. Higher rates increase HDD load but decrease memory usage.
    if (nh.getParam("file_write_rate", file_write_rate)) {
        ROS_INFO("Changing write rate to %dHz", file_write_rate);
    }
    // Initialize write_rate
    ros::Rate write_rate(file_write_rate);

    // Initialize audio system
    portaudio::System::initialize();
    ROS_INFO("Initialized %s", portaudio::System::versionText());
    ROS_INFO("Changing file location is not yet supported. Recorded data will be saved in %s[date&time].wav", file_path.c_str());

     // Get an instance of the audio system
    portaudio::System &audioSys = portaudio::System::instance();
    // Print the list of available output devices
    PrintInputDevices(audioSys);

    // Select input device by ID or name (if ID undefined)
    if (input_device_id != -1) {
        if (audioSys.deviceByIndex(input_device_id).maxInputChannels() < 1) {
            ROS_ERROR("Input device is not a recording device. Exiting...");
            return 0;
        }
    } else if (!input_device_name.empty()) {
        for (portaudio::System::DeviceIterator device = audioSys.devicesBegin(); device != audioSys.devicesEnd(); ++device) {
            if (device->maxInputChannels() < 1) { continue; }
            std::string device_name = device->name();
            boost::algorithm::to_lower(device_name);
            if (device_name.find(input_device_name) != std::string::npos) {
                input_device_id = device->index();
                break;
            }
        }
    }
    if (input_device_id == -1) {
        ROS_ERROR("No matching input device found. Exiting...");
        return 0;
    }

    // Initialize the playback device and get its properties
    portaudio::Device& recordingDevice = audioSys.deviceByIndex(input_device_id);
    double sample_rate = recordingDevice.defaultSampleRate();
    int input_channels = recordingDevice.maxInputChannels();
    ROS_INFO_STREAM("Using device #" << recordingDevice.index() << " (" << recordingDevice.name() << ") with " << recordingDevice.maxInputChannels() << " input channels available and " << recordingDevice.hostApi().name() << " hostAPI");

    // Calculate frame_rate or frame_size depending on sample frequency and the frame_size or frame_rate accordingly
    if (frame_rate != 0) {
        frame_size = recordingDevice.defaultSampleRate() / frame_rate;
    } else if (frame_size != 0) {
        frame_rate = recordingDevice.defaultSampleRate() / frame_size;
    } else {
        ROS_WARN("Neither _frame_size nor _frame_rate specified. Using default frame rate of 100");
        frame_rate = 100;
        frame_size = recordingDevice.defaultSampleRate() / frame_rate;
    }

    // Limit the input channels if the device input channels exceed the maximum channels
    if (max_channels != 0) {
        input_channels = std::min(recordingDevice.maxInputChannels(), max_channels);
        if (recordingDevice.maxInputChannels() > max_channels) {
            ROS_WARN("Only transporting %d of %d input channels", max_channels, recordingDevice.maxInputChannels());
        }
    }
    // Initialize the recording publisher with device and file properties
    // Set the according stream parameters
    RecordingPublisher objRecordingPublisher(pub, input_channels, recordingDevice.defaultSampleRate(), frame_rate, frame_size, file_path, file_write_rate);
    portaudio::DirectionSpecificStreamParameters inParamsRecord(recordingDevice, input_channels, portaudio::FLOAT32, false, recordingDevice.defaultLowInputLatency(), NULL);
    portaudio::StreamParameters paramsRecord(inParamsRecord, portaudio::DirectionSpecificStreamParameters::null(), recordingDevice.defaultSampleRate(), frame_size, paClipOff);
    portaudio::MemFunCallbackStream<RecordingPublisher> streamRecord(paramsRecord, objRecordingPublisher, &RecordingPublisher::RecordCallback);

    ROS_INFO("Initialized transport publisher with [%d] channels, frame_size [%d], sample frequency [%.0lfHz] and a resulting frame rate [%dHz]", input_channels, frame_size, sample_rate, frame_rate);

    // Start reading the audio stream
    streamRecord.start();

    // Keep the system running and write to file periodically
    while ((nh.ok()) && (ros::ok)) {
//        objRecordingPublisher.WriteToFile();
        write_rate.sleep();
    }
    return 0;
}