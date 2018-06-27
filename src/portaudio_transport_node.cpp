#include "ros/ros.h"

#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <vector>

#include <portaudio_transport/recording_publisher.h>

#include <portaudiocpp/PortAudioCpp.hxx>

#include "portaudio_transport/AudioTransport.h"

// ---------------------------------------------------------------------

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "portaudio_transport_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);
    ros::Publisher pub = nh.advertise<portaudio_transport::AudioTransport>("/portaudio_transport", 1);
    assert(CHAR_BIT * sizeof (float) == 32);

    // Parse _input_device as string or int to check for the name or pick via id
    std::string input_device_name = "";
    int input_device_id = 0;
    int frame_rate = 100;
    int max_channels = 0;
    if (nh.getParam("input_device", input_device_name)) {
        ROS_INFO("Matching input device name for: %s" , input_device_name.c_str());
        boost::algorithm::to_lower(input_device_name);

    } else if (nh.getParam("input_device", input_device_id)) {
        ROS_INFO("Using input device: %d", input_device_id);
    } else {
        ROS_WARN("_input_device not specified, using system default");
        input_device_name = "default";
    }
    if (nh.getParam("frame_rate", frame_rate)) {
        ROS_INFO("Using frame rate: %d", frame_rate);
    }
    if (nh.getParam("max_channels", max_channels)) {
        ROS_INFO("Limiting input channels to %d", max_channels);
    }

    // Initialize audio system
    portaudio::System::initialize();
    ROS_INFO("Initialized %s", portaudio::System::versionText());

    portaudio::System &audioSys = portaudio::System::instance();
    PrintInputDevices(audioSys);

    // Select input device by ID or name
    if (input_device_id != 0) {
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
    if (input_device_id == 0) {
        ROS_ERROR("No matching input device found. Exiting...");
        return 0;
    }
    portaudio::Device& recordingDevice = audioSys.deviceByIndex(input_device_id);
    int inputChannels = recordingDevice.maxInputChannels();
    ROS_INFO_STREAM("Using device #" << recordingDevice.index() << " (" << recordingDevice.name() << ") with " << recordingDevice.maxInputChannels() << " input channels available and " << recordingDevice.hostApi().name() << " hostAPI");

    if (max_channels != 0) {
        inputChannels = std::min(recordingDevice.maxInputChannels(),max_channels);
        if (recordingDevice.maxInputChannels() > max_channels) {
            ROS_WARN("Only transporting %d of %d input channels", max_channels, recordingDevice.maxInputChannels());
        }
    }
    // Create the recording publisher
    RecordingPublisher objRecordingPublisher(pub, inputChannels, recordingDevice.defaultSampleRate(), frame_rate);
    portaudio::DirectionSpecificStreamParameters inParamsRecord(recordingDevice, inputChannels, portaudio::FLOAT32, false, recordingDevice.defaultLowInputLatency(), NULL);
    portaudio::StreamParameters paramsRecord(inParamsRecord, portaudio::DirectionSpecificStreamParameters::null(), recordingDevice.defaultSampleRate(), (recordingDevice.defaultSampleRate() / frame_rate), paClipOff);
    portaudio::MemFunCallbackStream<RecordingPublisher> streamRecord(paramsRecord, objRecordingPublisher, &RecordingPublisher::RecordCallback);

    // Start reading the audio stream
    streamRecord.start();

    //ros::spin();

    for (int i = 0; i < 1; ++i) { ros::Duration(5.0).sleep(); }
    streamRecord.stop();

    objRecordingPublisher.WriteToFile("/tmp/");
    ros::Duration(1.5).sleep();
    return 0;
}