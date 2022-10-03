/*
* exp - experiments
*/
#include <chrono>

#include "core/libcamera_app.hpp"
#include "core/options.hpp"

// event loop
static void event_loop(LibcameraApp &app)
{
	app.OpenCamera();
	app.StartCamera();

	//auto start_time = std::chrono::high_resolution_clock::now();

	LibcameraApp::Msg msg = app.Wait();

	if (msg.type == LibcameraApp::MsgType::Timeout)
	{
		LOG_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
		app.StopCamera();
		app.StartCamera();
		//continue;
	}
	if (msg.type == LibcameraApp::MsgType::Quit)
		return;
	else if (msg.type != LibcameraApp::MsgType::RequestComplete)
		throw std::runtime_error("unrecognised message!");

	//auto now std::chrono::high_resolution_clock::now();
}

int main(int argc, char *argv[])
{
	try
	{
		LibcameraApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();
			event_loop(app);
		}
	}
	catch (const std::exception &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
		return -1;
	}
	return 0;
}