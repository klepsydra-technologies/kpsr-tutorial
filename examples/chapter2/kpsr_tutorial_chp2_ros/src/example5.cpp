#include <iostream>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <std_msgs/String.h>

int main()
{
    // Declare the eventloop and vector publisher
    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventloop(nullptr);

    eventloop.start();

    auto strpublisher = eventloop.getPublisher<std_msgs::String>("example5", 0, nullptr, nullptr);

    eventloop.getSubscriber<std_msgs::String>("example5")
        ->registerListener("listener", [](const std_msgs::String &message) {
            std::cout << "Message received: " << message.data << std::endl;
            std::cout << "Eventloop (subscriber) thread ID: " << std::this_thread::get_id()
                      << std::endl;
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(
        1)); // Ensures listener has been registered before publisher runs.

    // Set up sample publisher.
    std::thread t([&]() {
        std_msgs::String msg;
        msg.data = "Hello World!";
        strpublisher->publish(msg);
    });
    t.join();

    eventloop.getSubscriber<std_msgs::String>("example5")->removeListener("listener");
    eventloop.stop();

    return 0;
}