//
// Created by discord on 2/20/18.
//

#ifndef MOTORDAEMON_HERMES_SERIALCONTROLLER_HPP
#define MOTORDAEMON_HERMES_SERIALCONTROLLER_HPP

#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <functional>
#include <thread>
#include <string>
#include <unistd.h>
#include <queue>
#include <iostream>
#include "cpu_com_structs.h"
#include "json.hpp"
#include "ControllerInterface.hpp"


#include <stdio.h>
#include <string.h>
#include <portaudio.h>
#include <string>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include "SerialController.hpp"

#define CHECK(x) { if(!(x)) { \
fprintf(stderr, "%s:%i: failure at: %s\n", __FILE__, __LINE__, #x); \
_exit(1); } }


class PWMPlayer {

private:

    static const char *file_path;
    static ControllerInterface *motion;
    static PaStream *stream;
    static FILE *wavfile;
    static int numChannels;
    static int sampleRate;
    static PaSampleFormat sampleFormat;
    static int bytesPerSample, bitsPerSample;

    static const int speed_per_dB = 2;
    static const int speed_max = 10;

    static int paStreamCallback(
            const void *input, void *output,
            unsigned long frameCount,
            const PaStreamCallbackTimeInfo* timeInfo,
            PaStreamCallbackFlags statusFlags,
            void *userData )
    {
        size_t numRead = fread(output, bytesPerSample * numChannels, frameCount, wavfile);
        on_sample(numRead, static_cast<uint8_t *>(output));
        output = (uint8_t*)output + numRead * numChannels * bytesPerSample;
        frameCount -= numRead;

        if(frameCount > 0) {
            memset(output, 0, frameCount * numChannels * bytesPerSample);
            return paComplete;
        }

        return paContinue;
    }

    static void on_sample(size_t count, uint8_t *buffer) {
        float K = 0;
        float sum = 0;
        float volume = 0;

        for (int i = 0; i < count; i++) {
            sum += pow(buffer[i] / 255., 2);
        }

        volume = 20 * log10(sqrt(sum / count)) + K;

        // printf("volume = % .02f\n", volume);
        // char *s[speed_max] = "";

        //const float origVolume = volume;

        // Make volume a number from 0 to number of dB
        volume += (speed_max * speed_per_dB);
        volume = fmax(0, volume);
        volume /= speed_per_dB;
        int v = fmin(volume, speed_max);
        //printf("%6.2f %d %s\r", origVolume, v, stars + (speed_max - v));
        //if(motion)
        //    motion->setNeonSpeed((unsigned char) v);
    }

    bool portAudioOpen() {
        CHECK(Pa_Initialize() == paNoError);

        PaStreamParameters outputParameters;

        outputParameters.device = Pa_GetDefaultOutputDevice();
        CHECK(outputParameters.device != paNoDevice);

        outputParameters.channelCount = numChannels;
        outputParameters.sampleFormat = sampleFormat;
        outputParameters.suggestedLatency = Pa_GetDeviceInfo( outputParameters.device )->defaultHighOutputLatency;

        PaError ret = Pa_OpenStream(
                &stream,
                NULL, // no input
                &outputParameters,
                sampleRate,
                paFramesPerBufferUnspecified, // framesPerBuffer
                0, // flags
                &paStreamCallback,
                NULL //void *userData
        );

        if(ret != paNoError) {
            fprintf(stderr, "Pa_OpenStream failed: (err %i) %s\n", ret, Pa_GetErrorText(ret));
            if(stream)
                Pa_CloseStream(stream);
            return false;
        }

        CHECK(Pa_StartStream(stream) == paNoError);
        return true;
    }

    std::string freadStr(FILE *f, size_t len) {
        std::string s(len, '\0');
        CHECK(fread(&s[0], 1, len, f) == len);
        return s;
    }

    template<typename T>
    T freadNum(FILE *f) {
        T value;
        CHECK(fread(&value, sizeof(value), 1, f) == 1);
        return value; // no endian-swap for now... WAV is LE anyway...
    }

    void readFmtChunk(uint32_t chunkLen) {
        CHECK(chunkLen >= 16);
        uint16_t fmttag = freadNum<uint16_t>(wavfile); // 1: PCM (int). 3: IEEE float
        CHECK(fmttag == 1 || fmttag == 3);
        numChannels = freadNum<uint16_t>(wavfile);
        CHECK(numChannels > 0);
        printf("%i channels\n", numChannels);
        sampleRate = freadNum<uint32_t>(wavfile);
        printf("%i Hz\n", sampleRate);
        uint32_t byteRate = freadNum<uint32_t>(wavfile);
        uint16_t blockAlign = freadNum<uint16_t>(wavfile);
        bitsPerSample = freadNum<uint16_t>(wavfile);
        bytesPerSample = bitsPerSample / 8;
        CHECK(byteRate == sampleRate * numChannels * bytesPerSample);
        CHECK(blockAlign == numChannels * bytesPerSample);
        if (fmttag == 1 /*PCM*/) {
            switch (bitsPerSample) {
                case 8:
                    sampleFormat = paInt8;
                    break;
                case 16:
                    sampleFormat = paInt16;
                    break;
                case 32:
                    sampleFormat = paInt32;
                    break;
                default: CHECK(false);
            }
            printf("PCM %ibit int\n", bitsPerSample);
        } else {
            CHECK(fmttag == 3 /* IEEE float */);
            CHECK(bitsPerSample == 32);
            sampleFormat = paFloat32;
            printf("32bit float\n");
        }
        if (chunkLen > 16) {
            uint16_t extendedSize = freadNum<uint16_t>(wavfile);
            CHECK(chunkLen == 18 + extendedSize);
            fseek(wavfile, extendedSize, SEEK_CUR);
        }
    }

public:

    PWMPlayer(const char* file_path, ControllerInterface *motion)
    {
        PWMPlayer::file_path = file_path;
        PWMPlayer::motion = motion;
    }

    int play() {
        wavfile = fopen(file_path, "r");
        CHECK(wavfile != NULL);

        CHECK(freadStr(wavfile, 4) == "RIFF");
        uint32_t wavechunksize = freadNum<uint32_t>(wavfile);
        CHECK(freadStr(wavfile, 4) == "WAVE");
        while (true) {
            std::string chunkName = freadStr(wavfile, 4);
            uint32_t chunkLen = freadNum<uint32_t>(wavfile);
            if (chunkName == "fmt ")
                readFmtChunk(chunkLen);
            else if (chunkName == "data") {
                CHECK(sampleRate != 0);
                CHECK(numChannels > 0);
                CHECK(bytesPerSample > 0);
                printf("len: %.0f secs\n", double(chunkLen) / sampleRate / numChannels / bytesPerSample);
                break; // start playing now
            } else {
                // skip chunk
                CHECK(fseek(wavfile, chunkLen, SEEK_CUR) == 0);
            }
        }

        printf("start playing...\n");
        CHECK(portAudioOpen());
        printf("start playing2...\n");

        // wait until stream has finished playing
        while (Pa_IsStreamActive(stream) > 0)
        {
            usleep(1000);
            if(motion && !motion->playerThreadStarted)
                break;
        }

        printf("finished\n");
        fclose(wavfile);
        Pa_CloseStream(stream);
        Pa_Terminate();
        if(motion)
            motion->setNeonSpeed((unsigned char) 1);
    }
};

typedef struct cpu_com_result Result;

#define SERIAL_BAUDRATE 115200

class SerialController : public ControllerInterface
{
private:
    static int fileDesc;
    static struct cpu_com_status* currentStatus;

    std::thread main;
    std::thread reader;

    static volatile unsigned int expectedAnswers;

    static volatile bool write_mutex;

    static std::queue<Result*> resultQueue;

    static int Read(char*, int);

    static int Read_until(char*, int, char);

    static void mainWorker(void);
    static void readWorker(void);

public:
    static bool on;
    SerialController(char*);
    void init();
    void destructor(void);
    static int Write(const char*, unsigned int);
    Result* waitForResult();
    struct cpu_com_status getStatus();
    static void order(std::string order);

    void stop(void) ;

    void orderTranslation(long i) ;

    void orderAngle(float d) ;

    void setSpeedTranslation(int i) ;

    void orderCurveRadius(long i) ;

    void setTranslationTunings(float d, float d1, float d2) ;

    void setCurveTunings(float d, float d1, float d2) ;

    void setLeftSpeedTunings(float d, float d1, float d2) ;

    void setRightSpeedTunings(float d, float d1, float d2) ;

    void setPosition(double xn, double yn) ;

    void setAngle(double o) ;

    const char *getTunings(void) ;

    void testPosition(void) ;

    void testSpeed(int i) ;

    void setTrajectory(std::vector<Cinematic> &vector, long i) ;

    const char *isMoving(void) ;

    bool isPhysicallyStopped(void) ;

    long getTranslationSetPoint(void) ;

    void go(void) ;

    void goR(void) ;

    void setControlled(bool b) ;

    void sweep(bool way) ;

    void stopSweep(void) ;

    long getCurveRadius(void) ;

    double getX(void) ;

    double getY(void) ;

    long getSpeed(void) ;

    long getSpeedL(void) ;

    long getSpeedR(void) ;

    long getCSpeedL(void) ;

    long getCSpeedR(void) ;

    double getAngle(void) ;

    void loadPos() ;

    void printTranslationError(void) ;

    const char *controlledStatus() ;

    std::vector<std::string> splitl(std::string str, char delimiter)
    {
        std::vector<std::string> internal;
        std::stringstream ss(str); // Turn the string into a stream.
        std::string tok;

        while(std::getline(ss, tok, delimiter)) {
            internal.push_back(tok);
        }

        return internal;
    }

    int set_interface_attribs (int fd, int speed, int parity)
    {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
            printf ("error from tcgetattr", errno);
            return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // ignore break signal
        tty.c_lflag = 0;                // no signaling chars, no echo,
        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
            printf ("error %d from tcsetattr", errno);
            return -1;
        }
        return 0;
    }

    void set_blocking (int fd, int should_block)
    {
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
            printf ("error %d from tggetattr", errno);
            return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            	// 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
            printf ("error %d setting term attributes", errno);
    }

    std::string floatToString(float f)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << f;
        return ss.str();
    }

    void setNeonSpeed(unsigned char s);

};




#endif //MOTORDAEMON_HERMES_SERIALCONTROLLER_HPP
