#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/mem_core/mem_env.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/high_performance/data_multiplexer_middleware_provider.h>

#include "control_service.h"
#include "application_service.h"
int main() {

    // Create the environment.
    kpsr::mem::MemEnv environment;
    // Set the dummy values of environment variables to test with.
    environment.setPropertyFloat("low_threshold", 5.0f);
    environment.setPropertyFloat("high_threshold", 15.0f);

    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);
    eventloop.start();

    kpsr::ControlService controlService(eventloop.getSubscriber<float> ("battery"),
                                        eventloop.getPublisher<kpsr::SystemEventData> ("status", 10, nullptr, nullptr),
                                        &environment);
    controlService.startup();
    kpsr::ApplicationService applicationService(&environment,
                                                eventloop.getSubscriber<kpsr::SystemEventData> ("status"),
                                                "application");

    auto batteryPublisher = eventloop.getPublisher<float>("battery", 10, nullptr, nullptr);
    std::thread t([&batteryPublisher]() {
                      for (int i = 0; i < 20; i ++) {
                          batteryPublisher->publish(static_cast<float>(i));
                          std::this_thread::sleep_for(std::chrono::milliseconds(1));
                      }
                  });
    t.join();
    controlService.shutdown();
    eventloop.stop();
}
