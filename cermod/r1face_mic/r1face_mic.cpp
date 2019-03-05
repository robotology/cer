/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino <alberto.cardellino@iit.it>
 *          Francesco Diotalevi <francesco.diotalevi@iit.it>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <fcntl.h>
#include <string.h>

#ifdef WIN32
#define SIMULATE_ONLY
#pragma message ("WARNING: r1face_mic device is implemented on Linux only, r1face_mic will be simulated on WIN32")
#endif

#ifndef  SIMULATE_ONLY
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#include "r1face_mic.h"
#include <yarp/os/Time.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LockGuard.h>

#define SLEEP_TIME 0.005f

#define STEREO              2      // Got from HW specifications, do not change it!!
#define HW_STEREO_CHANNELS  9      // Got from HW specifications, do not change it!!
#define SAMPLING_RATE       16000  // Got from HW specifications, do not change it!!
#define CHUNK_SIZE          512    // Got from HW specifications, do not change it!!
#define TOT_SAMPLES         ((HW_STEREO_CHANNELS * STEREO) * CHUNK_SIZE)

#define BUFFER_SIZE         100    //chosen by user

// Low level driver configurations
#define MAGIC_NUM           100
#define IOCTL_SAMPLERATE    _IOW(MAGIC_NUM, 1, int )
#define IOCTL_SETBURST      _IOWR(MAGIC_NUM, 38, int )

using namespace cer::dev;
using namespace yarp::dev;
using namespace yarp::os;


struct inputData {
    int32_t data[TOT_SAMPLES];
};


R1faceMic::R1faceMic(): PeriodicThread(0),
                        shift(8),
                        m_isRecording(false),
                        channels(HW_STEREO_CHANNELS),
                        samplingRate(SAMPLING_RATE),
                        dev_fd(-1),
                        chunkSize(CHUNK_SIZE),
                        deviceFile("/dev/micif_dev"),
                        rawBuffer(nullptr),
                        selected_chan(-1),
                        userChannelsNum(8)
{
    tmpData     = new inputData;
    
#if CARDE
    inputBuffer = new CircularBuffer<inputData>(BUFFER_SIZE);
#else
    const size_t _channels = HW_STEREO_CHANNELS * STEREO;
    const size_t _samples = BUFFER_SIZE * CHUNK_SIZE;
    const size_t _depth = 4;
    yarp::dev::AudioBufferSize size (_samples, _channels, _depth);
    inputBuffer = new CircularAudioBuffer_32t("r1_face_mic",size);
    record_waiting_counter=0;
#endif
}

R1faceMic::~R1faceMic()
{
    if(tmpData)
        delete tmpData;
    if(inputBuffer)
        delete inputBuffer;
}

bool R1faceMic::open(yarp::os::Searchable &params)
{
    if(params.check("audioDevice") && params.find("audioDevice").isString())
        deviceFile = params.find("audioDevice").asString();

    shift = params.check("shift", yarp::os::Value(6)).asInt();

    bool singleChannel = params.check("channel");
    if (singleChannel)
    {
        selected_chan = params.find("channel").asInt32();
        if ((selected_chan < 0) || (selected_chan >= HW_STEREO_CHANNELS))
        {
            yError() << "Requested channel do not exists. Available channels are from 0 to " << HW_STEREO_CHANNELS - 2;
            return false;
        }
        userChannelsNum = 1;
    }

#ifndef  SIMULATE_ONLY
    dev_fd = ::open(deviceFile.c_str(), O_RDONLY);
    if (dev_fd < 0)
    {
        fprintf(stderr, "Error opening %s: %s\n", deviceFile.c_str(), strerror(errno));
        return false;
    }

    ioctl(dev_fd, IOCTL_SETBURST,   chunkSize);
    ioctl(dev_fd, IOCTL_SAMPLERATE, 16000);    // Actually this ioctl seems not to work... it sample at 16KHz anyway
#endif

    yDebug() << "Input configuration is " << params.toString();
    yInfo()  << "R1faceMic device opened, starting thread";
    return start();
}

bool R1faceMic::close()
{
    m_isRecording = false;
    stop();
    return true;
}


bool R1faceMic::threadInit()
{
    //better not to start the recording here!
    //you may get buffer overrun if you do not call getSound() quickly enough
    //startRecording();
    return true;
}

void R1faceMic::run()
{
    // when not recording, do nothing
    if(!m_isRecording)
    {
        yarp::os::Time::delay(0.01);
        return;
    }

#ifndef SIMULATE_ONLY
    // Just acquire raw data and place them in the buffer as fast as possible
    if (::read(dev_fd, tmpData->data, CHUNK_SIZE * HW_STEREO_CHANNELS * STEREO * sizeof(int32_t)) < 0)
    {
        yError() << "R1 face microphones: error reading data from HW";
        return;
    }
#endif

    for (size_t i = 0; i < TOT_SAMPLES; i++)
    {
        inputBuffer->write(tmpData->data[i]);
    }

//#define DEBUG_BUFFER_SIZE
#ifdef  DEBUG_BUFFER_SIZE
    yDebug() << inputBuffer->size().getBufferElements() << "/" << inputBuffer->getMaxSize().getBufferElements() << "Buffer Elements";

    yDebug() << inputBuffer->size().getSamples()<< "/" << inputBuffer->getMaxSize().getSamples() << "Samples";
#endif
}

void R1faceMic::threadRelease()
{
    stopRecording();
}

bool R1faceMic::getSound(yarp::sig::Sound& sound, size_t min_number_of_samples, size_t max_number_of_samples, double max_samples_timeout_s)
{
    //check for something_to_record
    {
#ifdef AUTOMATIC_REC_START
        if (m_isRecording == false)
        {
            this->startRecording();
        }
#else
        double debug_time = yarp::os::Time::now();
        while (m_isRecording == false)
        {
            if (yarp::os::Time::now() - debug_time > 5.0)
            {
                yInfo() << "getSound() is currently waiting. Use ::startRecording() to start the audio stream";
                debug_time = yarp::os::Time::now();
            }
            yarp::os::SystemClock::delaySystem(SLEEP_TIME);
        }
#endif
    }

    //check on input parameters
    if (max_number_of_samples < min_number_of_samples)
    {
        yError() << "max_number_of_samples must be greater than min_number_of_samples!";
        return false;
    }
    if (max_number_of_samples > this->inputBuffer->getMaxSize().getSamples())
    {
        yWarning() << "max_number_of_samples bigger than the internal audio buffer! It will be truncated to:" << this->inputBuffer->getMaxSize().getSamples();
        max_number_of_samples = this->inputBuffer->getMaxSize().getSamples();
    }

    //forcing device capabilities
    min_number_of_samples = 5 * CHUNK_SIZE;

    size_t buff_size = 0;
    double start_time = yarp::os::Time::now();
    double debug_time = yarp::os::Time::now();
    do
    {
        buff_size = inputBuffer->size().getSamples();
        if (buff_size > max_number_of_samples) break;
        if (buff_size > min_number_of_samples && yarp::os::Time::now() - start_time > max_samples_timeout_s) break;
        if (m_isRecording == false) { break; }

        if (yarp::os::Time::now() - debug_time > 1.0)
        {
            debug_time = yarp::os::Time::now();
            yDebug() << "R1faceMic::getSound() Buffer size is " << buff_size << "/" << max_number_of_samples << " after 1s";
        }

        yarp::os::SystemClock::delaySystem(0.005);
    } while (true);

    //prepare the sound data struct
    size_t samples_to_be_copied = buff_size;
    if (samples_to_be_copied > max_number_of_samples) samples_to_be_copied = max_number_of_samples;
    if (sound.getChannels() != userChannelsNum && sound.getSamples() != samples_to_be_copied)
    {
        sound.resize(samples_to_be_copied, userChannelsNum);
    }
    sound.setFrequency(samplingRate);
    sound.clear();
    
    ///////////////////////////////////////////////
    // Extract channels from acquired buffer and
    // manipulate them to get meaningful information
    ///////////////////////////////////////////////
    size_t sample_counter = 0;
    for (size_t sample_counter =0; sample_counter< samples_to_be_copied; sample_counter++)
    {
        for (size_t skip = 0; skip < 9 + 1; skip++)
        {
            int32_t dummy = inputBuffer->read();
        }
        for (size_t chan = 0; chan < 8; chan++)
        {
            int32_t value24 = inputBuffer->read();
            int16_t value16 = value24 >> shift;
            if (selected_chan == -1)
            {
                sound.set(value16, sample_counter, chan);
            }
            else if (selected_chan == chan)
            {
                sound.set(value16, sample_counter, 0);
            }
        }
    }
//#define DEBUG_BUFFER_SIZE
#ifdef  DEBUG_BUFFER_SIZE
    yDebug() << "sound size"<< sound.getSamples();
    //yDebug() << inputBuffer->size().getBufferElements() << "/" << inputBuffer->getMaxSize().getBufferElements() << "Buffer Elements";

     yDebug() << inputBuffer->size().getSamples() << "/" << inputBuffer->getMaxSize().getSamples() << "Samples";
#endif
    return true;
}

bool R1faceMic::startRecording()
{
    if (m_isRecording == true) return true;
    LockGuard lock(m_mutex);
    m_isRecording = true;
#ifdef BUFFER_AUTOCLEAR
    inputBuffer->clear();
#endif
    m_isRecording = true;
    yInfo() << "R1faceMic started recording";
    return true;
}

bool R1faceMic::stopRecording()
{
    if (m_isRecording == false) return true;
    LockGuard lock(m_mutex);
    m_isRecording = false;
#ifdef BUFFER_AUTOCLEAR
    inputBuffer->clear();
#endif
    yInfo() << "R1faceMic stopped recording";
    return true;
}

bool R1faceMic::startService()
{
    threadInit();
    return false;
}

bool R1faceMic::updateService()
{
    run();
    return false;
}

bool R1faceMic::stopService()
{
    threadRelease();
    return false;
}

bool R1faceMic::getRecordingAudioBufferMaxSize(yarp::dev::AudioBufferSize& size)
{
    //no lock guard is needed here
    size = this->inputBuffer->getMaxSize();
    return true;
}


bool R1faceMic::getRecordingAudioBufferCurrentSize(yarp::dev::AudioBufferSize& size)
{
    //no lock guard is needed here
    size = this->inputBuffer->size();
    return true;
}


bool R1faceMic::resetRecordingAudioBuffer()
{
    LockGuard lock(m_mutex);
    inputBuffer->clear();
    yDebug() << "R1faceMic::resetRecordingAudioBuffer";
    return true;
}
