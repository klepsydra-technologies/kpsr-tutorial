#include <algorithm>
#include <iostream>
#include <thread>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

const int poolSize{16};

struct MyData
{
    int a;
    int b;
};

int main(int argc, char *argv[])
{
    std::cout << "Main thread " << std::this_thread::get_id() << std::endl;

    kpsr::high_performance::EventLoopMiddlewareProvider<16> eventLoop(nullptr);

    kpsr::Publisher<MyData> *myDataPublisher = eventLoop.getPublisher<MyData>(
        "myData",
        poolSize,
        nullptr,
        //[](const MyData &src, MyData &dst) { dst.a = src.a; dst.b = src.b; }
        nullptr);
    kpsr::Subscriber<MyData> *myDataSubscriber = eventLoop.getSubscriber<MyData>("myData");
    kpsr::Publisher<int> *intPublisher = eventLoop.getPublisher<int>("max", 0, nullptr, nullptr);

    myDataSubscriber->registerListener("myData_listener", [&intPublisher](const MyData &v) {
        std::cout << "myData received [" << v.a << "," << v.b << "] at thread "
                  << std::this_thread::get_id() << std::endl;
        // imagine a heavy process
        std::this_thread::sleep_for(std::chrono::seconds(2));
        int m = v.a > v.b ? v.a : v.b;
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

    std::thread t1([&myDataPublisher]() {
        MyData v{1, 2};
        std::cout << "myData published [" << v.a << "," << v.b << "] at thread "
                  << std::this_thread::get_id() << std::endl;
        myDataPublisher->publish(v);
        // data updating while the subscriber is working
        MyData w = v;
        v.a = 3;
        std::cout << "v.a=" << v.a << ", w.a=" << w.a << std::endl;
    });
    t1.join();

    std::thread t2([&myDataPublisher]() {
        MyData v{4, 5};
        std::cout << "myData published [" << v.a << "," << v.b << "] at thread "
                  << std::this_thread::get_id() << std::endl;
        myDataPublisher->publish(v);
    });
    t2.join();

    std::this_thread::sleep_for(std::chrono::seconds(3));
    eventLoop.stop();
    intSubscriber->removeListener("myData_listener");
    myDataSubscriber->removeListener("int_listener");

    return 0;
}