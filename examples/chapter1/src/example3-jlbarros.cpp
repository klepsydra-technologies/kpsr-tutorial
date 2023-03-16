#include <algorithm>
#include <iostream>
#include <thread>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

const int poolSize{16};
const int vectorSize{3};

int main(int argc, char *argv[])
{
    std::cout << "Main thread " << std::this_thread::get_id() << std::endl;

    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventLoop(nullptr);

    kpsr::Publisher<std::vector<int>> *vectorPublisher = eventLoop.getPublisher<std::vector<int>>(
        "vector", poolSize, [](std::vector<int> &v) { v.resize(vectorSize); }, nullptr);
    kpsr::Subscriber<std::vector<int>> *vectorSubscriber = eventLoop.getSubscriber<std::vector<int>>(
        "vector");
    kpsr::Publisher<int> *intPublisher = eventLoop.getPublisher<int>("max", 0, nullptr, nullptr);

    vectorSubscriber->registerListener("vector_listener", [&intPublisher](const std::vector<int> &v) {
        std::cout << "Vector received [" << v.front() << ",...," << v.back() << "] at thread "
                  << std::this_thread::get_id() << std::endl;
        int m = *std::max_element(std::begin(v), std::end(v));
        intPublisher->publish(m);
        std::cout << "Max published " << m << " at thread " << std::this_thread::get_id()
                  << std::endl;
    });

    kpsr::Subscriber<int> *intSubscriber = eventLoop.getSubscriber<int>("max");
    intSubscriber->registerListener("int_listener", [](const float &m) {
        std::cout << "Max received " << m << " at thread " << std::this_thread::get_id()
                  << std::endl;
    });

    eventLoop.start();
    std::thread t1([&vectorPublisher]() {
        std::vector<int> v{1, 2, 3};
        std::cout << "Vector published [" << v.front() << ",...," << v.back() << "] at thread "
                  << std::this_thread::get_id() << std::endl;
        vectorPublisher->publish(v);
    });
    t1.join();

    std::thread t2([&vectorPublisher]() {
        std::vector<int> v{4, 5, 6};
        std::cout << "Vector published [" << v.front() << ",...," << v.back() << "] at thread "
                  << std::this_thread::get_id() << std::endl;
        vectorPublisher->publish(v);
    });
    t2.join();

    std::thread t3([&vectorPublisher]() {
        // innefficient: vector size is bigger than pre-allocated
        std::vector<int> v{7, 8, 9, 10};
        std::cout << "Vector published [" << v.front() << ",...," << v.back() << "] at thread "
                  << std::this_thread::get_id() << std::endl;
        vectorPublisher->publish(v);
    });
    t3.join();

    std::this_thread::sleep_for(std::chrono::seconds(1));
    eventLoop.stop();
    intSubscriber->removeListener("vector_listener");
    vectorSubscriber->removeListener("int_listener");

    return 0;
}