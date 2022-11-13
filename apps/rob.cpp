#include <chrono>
#include <libcamera/libcamera.h>
#include "core/libcamera_app.hpp"
#include "core/options.hpp"
#include <iomanip>

using namespace std::placeholders;

// The main event loop for the application.

using namespace libcamera;

static std::shared_ptr<Camera> camera;

static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
    {
        return;
    }


    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();

    for (auto bufferPair : buffers)
    {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();
        std::cout << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence << " bytesused: ";

        unsigned int nplane = 0;
        for (const FrameMetadata::Plane &plane : metadata.planes())
        {
            std::cout << plane.bytesused;
            if (++nplane < metadata.planes().size()) std::cout << "/";
        }

        std::cout << std::endl;

    }



    request->reuse(Request::ReuseBuffers);
    camera->queueRequest(request);
}

int main(int argc, char *argv[])
{
    LibcameraApp app;

	std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
	cm->start();

    for (auto const &camera : cm->cameras())
    {
        std::cout << camera->id() << std::endl;
    }

    if (cm->cameras().empty() == true)
    {
        std::cout << "No cameras were identified on the system." << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    std::shared_ptr<Camera> myCam= cm->cameras()[0];
    std::string cameraId = myCam->id();

    camera = cm->get(cameraId);
    camera->acquire();

    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration( { StreamRole::StillCapture } );
    for(StreamConfiguration &cfg : *config)
    {
        std::cout << "Default viewfinder configuration is: " << cfg.toString() << std::endl;
    }

    StreamConfiguration &streamConfig = config->at(0);
    std::cout << "Default viewfinder configuration is: " << streamConfig.toString() << std::endl;

    camera->configure(config.get());

    // framebuffer allocate
    FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);
    for(StreamConfiguration &cfg : *config)
    {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0)
        {
            std::cerr << "Can't allocate buffers" << std::endl;
            return -ENOMEM;
        }
    
        size_t allocated = allocator->buffers(cfg.stream()).size();
        std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
    }


    // frame capture
    Stream *stream = streamConfig.stream();
    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
    std::vector<std::unique_ptr<Request>> requests;


    for (unsigned int i = 0; i < buffers.size(); ++i)
    {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request)
        {
            std::cerr << "Can't create request" << std::endl;
            return -ENOMEM;
        }

        const std::unique_ptr<FrameBuffer> &buffer = buffers[i];
        int ret = request->addBuffer(stream, buffer.get());
        if (ret < 0)
        {
            std::cerr << "Can't set buffer for request"
                << std::endl;
            return ret;
        }

        requests.push_back(std::move(request));
    }

    camera->requestCompleted.connect(requestComplete);

    camera->start();
    for (std::unique_ptr<Request> &request : requests)
    {
        camera->queueRequest(request.get());
    }

    std::this_thread::sleep_for(3000ms);

    camera->stop();
    allocator->free(stream);
    delete allocator;
    camera->release();
    camera.reset();

    cm->stop();

    return 0;

}