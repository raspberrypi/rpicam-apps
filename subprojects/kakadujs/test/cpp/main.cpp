// Copyright (c) Chris Hafey.
// SPDX-License-Identifier: MIT

#include <fstream>
#include <iostream>
#include <vector>
#include <iterator>
#include <time.h>
#include <algorithm>
#include <HTJ2KDecoder.hpp>
#include <HTJ2KEncoder.hpp>

/* ========================================================================= */
/*                         Set up messaging services                         */
/* ========================================================================= */

class kdu_stream_message : public kdu_core::kdu_thread_safe_message
{
public: // Member classes
    kdu_stream_message(std::ostream *stream)
    {
        this->stream = stream;
    }
    void put_text(const char *string)
    {
        (*stream) << string;
    }
    void flush(bool end_of_message = false)
    {
        stream->flush();
        kdu_thread_safe_message::flush(end_of_message);
    }

private: // Data
    std::ostream *stream;
};

static kdu_stream_message cout_message(&std::cout);
static kdu_stream_message cerr_message(&std::cerr);
static kdu_core::kdu_message_formatter pretty_cout(&cout_message);
static kdu_core::kdu_message_formatter pretty_cerr(&cerr_message);

#ifdef _WIN32
#define CLOCK_PROCESS_CPUTIME_ID 0
// struct timespec { long tv_sec; long tv_nsec; };    //header part
int clock_gettime(int, struct timespec *spec) // C-file part
{
    __int64 wintime;
    GetSystemTimeAsFileTime((FILETIME *)&wintime);
    wintime -= 116444736000000000i64;            // 1jan1601 to 1jan1970
    spec->tv_sec = wintime / 10000000i64;        // seconds
    spec->tv_nsec = wintime % 10000000i64 * 100; // nano-seconds
    return 0;
}
#endif

void readFile(std::string fileName, std::vector<uint8_t> &vec)
{
    // open the file:
    std::ifstream file(fileName, std::ios::in | std::ios::binary);
    if (file.fail())
    {
        printf("File %s does not exist\n", fileName.c_str());
        exit(1);
    }
    // Stop eating new lines in binary mode!!!
    file.unsetf(std::ios::skipws);

    // get its size:
    int fileSize;
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // reserve capacity
    vec.resize(0);
    vec.reserve(fileSize);

    // read the data:
    vec.insert(vec.begin(),
               std::istream_iterator<uint8_t>(file),
               std::istream_iterator<uint8_t>());
}

void writeFile(std::string fileName, const std::vector<uint8_t> &vec)
{
    std::ofstream file(fileName, std::ios::out | std::ofstream::binary);
    std::copy(vec.begin(), vec.end(), std::ostreambuf_iterator<char>(file));
}

enum
{
    NS_PER_SECOND = 1000000000
};

void sub_timespec(struct timespec t1, struct timespec t2, struct timespec *td)
{
    td->tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td->tv_sec = t2.tv_sec - t1.tv_sec;
    if (td->tv_sec > 0 && td->tv_nsec < 0)
    {
        td->tv_nsec += NS_PER_SECOND;
        td->tv_sec--;
    }
    else if (td->tv_sec < 0 && td->tv_nsec > 0)
    {
        td->tv_nsec -= NS_PER_SECOND;
        td->tv_sec++;
    }
}

std::vector<uint8_t> decodeFile(const char *path, size_t iterations = 1, bool silent = false)
{
    HTJ2KDecoder decoder;
    std::vector<uint8_t> &encodedBytes = decoder.getEncodedBytes();
    readFile(path, encodedBytes);

    timespec start, finish, delta;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);
    decoder.readHeader();

    for (int i = 0; i < iterations; i++)
    {
        decoder.decode();
    }

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &finish);
    sub_timespec(start, finish, &delta);
    auto frameInfo = decoder.getFrameInfo();

    auto ns = delta.tv_sec * 1000000000.0 + delta.tv_nsec;
    auto totalTimeMS = ns / 1000000.0;
    auto timePerFrameMS = ns / 1000000.0 / (double)iterations;
    auto pixels = (frameInfo.width * frameInfo.height);
    auto megaPixels = (double)pixels / (1024.0 * 1024.0);
    auto fps = 1000 / timePerFrameMS;
    auto mps = (double)(megaPixels)*fps;

    if (!silent)
    {
        printf("NATIVE decode %s TotalTime: %.3f s for %zu iterations; TPF=%.3f ms (%.2f MP/s, %.2f FPS)\n", path, totalTimeMS / 1000, iterations, timePerFrameMS, mps, fps);
    }

    // printf("Native-decode %s TotalTime= %.2f ms TPF=%.2f ms (%.2f MP/s, %.2f FPS)\n", path, totalTimeMS, timePerFrameMS, mps, fps);
    return decoder.getDecodedBytes();
}

void encodeFile(const char *inPath, const FrameInfo frameInfo, const char *outPath = NULL, size_t iterations = 1, bool silent = false)
{
    // printf("FrameInfo %dx%dx%d %d bpp\n", frameInfo.width, frameInfo.height, frameInfo.componentCount, frameInfo.bitsPerSample);
    HTJ2KEncoder encoder;
    encoder.setQuality(true, 0.0f);
    encoder.setDecompositions(5);
    encoder.setBlockDimensions(Size(64, 64));
    encoder.setProgressionOrder(0);
    std::vector<uint8_t> &rawBytes = encoder.getDecodedBytes(frameInfo);

    readFile(inPath, rawBytes);

    timespec start, finish, delta;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &start);

    for (int i = 0; i < iterations; i++)
    {
        encoder.encode();
    }

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &finish);
    sub_timespec(start, finish, &delta);

    auto ns = delta.tv_sec * 1000000000.0 + delta.tv_nsec;
    auto totalTimeMS = ns / 1000000.0;
    auto timePerFrameMS = ns / 1000000.0 / (double)iterations;
    auto pixels = (frameInfo.width * frameInfo.height);
    auto megaPixels = (double)pixels / (1024.0 * 1024.0);
    auto fps = 1000 / timePerFrameMS;
    auto mps = (double)(megaPixels)*fps;

    const std::vector<uint8_t> &encodedBytes = encoder.getEncodedBytes();
    if (!silent)
    {
        printf("NATIVE encode %s TotalTime: %.3f s for %zu iterations; TPF=%.3f ms (%.2f MP/s, %.2f FPS)\n", inPath, totalTimeMS / 1000, iterations, timePerFrameMS, mps, fps);
    }

    if (outPath)
    {
        writeFile(outPath, encodedBytes);
    }
}

int main(int argc, char **argv)
{
    kdu_customize_warnings(&pretty_cout);
    kdu_customize_errors(&pretty_cerr);

    const size_t iterations = (argc > 1) ? atoi(argv[1]) : 2000;

    //  warm up the decoder and encoder
    try
    {
        decodeFile("test/fixtures/j2c/CT1.j2c", 1, false);
        encodeFile("test/fixtures/raw/CT1.RAW", {.width = 512, .height = 512, .bitsPerSample = 16, .componentCount = 1, .isSigned = true}, NULL, 1, true);

        // benchmark
        decodeFile("test/fixtures/j2c/CT1.j2c", iterations);
        // decodeFile("test/fixtures/j2c/MG1.j2c", iterations);
        //  encodeFile("test/fixtures/raw/CT1.RAW", {.width = 512, .height = 512, .bitsPerSample = 16, .componentCount = 1, .isSigned = true}, NULL, iterations);

        // JPEG2000 color testing
        // deFile("test/fixtures/j2k/US1.j2k", 1);

        // encodeFile("test/fixtures/raw/CT1.RAW", {.width = 512, .height = 512, .bitsPerSample = 16, .componentCount = 1, .isSigned = true}, "test/fixtures/j2c/ignore.j2c");
        // decodeFile("test/fixtures/j2c/ignore.j2c", 1);
        // decodeFile("test/fixtures/j2c/ignore.j2c", 1);

        /*
            encodeFile("test/fixtures/raw/CT1.RAW", {.width = 512, .height = 512, .bitsPerSample = 16, .componentCount = 1, .isSigned = true}, "test/fixtures/j2c/ignore.j2c", iterations);
            printf("decoding test/fixtures/raw/CT1.RAW\n");
            std::vector<uint8_t> decodedRawBytes = decodeFile("test/fixtures/raw/CT1.RAW");
            std::vector<uint8_t> originalRawBytes;
            readFile("test/fixtures/raw/CT1.RAW", originalRawBytes);
            printf("matches = %d\n", decodedRawBytes == originalRawBytes);
        */
    }
    catch (const char *pError)
    {
        printf("ERROR: %s\n", pError);
    }
    return 0;
}
