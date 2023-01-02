/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
*
****************************************************************************/

#include <random>

#include <spdlog/spdlog.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "test_result_data.h"
#include <input_parser.h>

class PerformanceTest
{
public:
    PerformanceTest(kpsr::EventEmitterType eventEmitterType,
                    int sleepUS,
                    int poolSize,
                    int numListeners,
                    int numIterations,
                    bool bufferChecks)
        : totalProcessed(0)
        , totalLatency(0)
        , _eventEmitterType(eventEmitterType)
        , _unsafeBufferPointer(!bufferChecks)
        , _poolSize(poolSize)
        , _numListeners(numListeners)
        , _sleepUS(sleepUS)
        , _numIterations(numIterations)
        , _eventLoop(nullptr,
                     "event_loop",
                     kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                     {},
                     _eventEmitterType)
    {
        for (int i = 0; i < _numListeners; i++) {
            std::string name = "channel_" + std::to_string(i);
            std::string listenerName = "listener_" + std::to_string(i);
            _eventLoop.getSubscriber<unsigned long long>(name, _eventEmitterType)
                ->registerListener(listenerName, [&, listenerName](const unsigned long long &event) {
                    unsigned long long now = kpsr::TimeUtils::getCurrentNanosecondsAsLlu();
                    totalLatency += now - event;
                    totalProcessed++;
                    spdlog::debug("{}, Latency: {}", __PRETTY_FUNCTION__, (now - event));
                });
        }

        _eventLoop.start();
    }

    ~PerformanceTest()
    {
        for (int i = 0; i < _numListeners; i++) {
            std::string name = "channel_" + std::to_string(i);
            std::string listenerName = "listener_" + std::to_string(i);
            _eventLoop.getSubscriber<unsigned long long>(name, _eventEmitterType)
                ->removeListener(listenerName);
        }
        _eventLoop.stop();
    }

    long test()
    {
        long totalExecutionTime = 0;
        totalProcessed = 0;
        totalLatency = 0;
        long before = kpsr::TimeUtils::getCurrentMilliseconds();

        spdlog::debug("starting benchmark...");

        std::vector<std::thread> threads(0);
        for (int index = 0; index < _numListeners; index++) {
            threads.push_back(std::thread([this, index]() {
                std::string name = "channel_" + std::to_string(index);
                kpsr::Publisher<unsigned long long> *publisher =
                    _eventLoop.getPublisher<unsigned long long>(name,
                                                                _poolSize,
                                                                nullptr,
                                                                nullptr/*,
                                                                _unsafeBufferPointer*/);
                for (int i = 0; i < _numIterations; i++) {
                    unsigned long long event = kpsr::TimeUtils::getCurrentNanosecondsAsLlu();
                    publisher->publish(event);
                    std::this_thread::sleep_for(std::chrono::microseconds(_sleepUS));
                }
            }));
        }
        long long unsigned int discardedMessages(0);
        for (int i = 0; i < _numListeners; i++) {
            threads[i].join();
            std::string name = "channel_" + std::to_string(i);
            auto eventLoopPublisher =
                dynamic_cast<kpsr::high_performance::EventLoopPublisher<unsigned long long, 2048> *>(
                    _eventLoop.getPublisher<unsigned long long>(name,
                                                                _poolSize,
                                                                nullptr,
                                                                nullptr/*,
                                                                _unsafeBufferPointer*/));
            discardedMessages += eventLoopPublisher->_discardedMessages;
        }

        while (totalProcessed < (_numListeners * _numIterations - discardedMessages)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        spdlog::debug("finishing benchmark.");

        long after = kpsr::TimeUtils::getCurrentMilliseconds();
        totalExecutionTime = after - before;
        return totalExecutionTime;
    }

    unsigned long long totalProcessed;
    unsigned long long totalLatency;

private:
    kpsr::EventEmitterType _eventEmitterType;
    bool _unsafeBufferPointer;
    int _poolSize;
    int _numListeners;
    int _sleepUS;
    int _numIterations;
    kpsr::high_performance::EventLoopMiddlewareProvider<2048> _eventLoop;
};

int main(int argc, char **argv)
{
    InputParser inputParser;
    InputData inputData;
    bool parsedOk = inputParser.getConfFileFromParams(argc, argv, inputData);
    if (!parsedOk) {
        return 0;
    }

    std::tuple<std::string, kpsr::EventEmitterType> scenario;
    if (inputData.testUnsafeMulti) {
        scenario = {"Unsafe Multi EventEmitter", kpsr::EventEmitterType::UNSAFE_MULTI};
    } else {
        scenario = {"Full Event Emitter", kpsr::EventEmitterType::SAFE};
    }

    std::vector<long> totalExecutionTimes;
    std::vector<long> averageLatencies;

    spdlog::info("Starting. {}\tSleep time: {}\tPool size: {}\tNumber of listeners: {}\tNumber of "
                 "iterations: {} \t Buffer Safety Checks = {}",
                 std::get<0>(scenario),
                 inputData.sleepTime,
                 inputData.poolSize,
                 inputData.numListeners,
                 inputData.iterations,
                 inputData.bufferChecks);

    PerformanceTest performanceTest(std::get<1>(scenario),
                                    inputData.sleepTime,
                                    inputData.poolSize,
                                    inputData.numListeners,
                                    inputData.iterations,
                                    inputData.bufferChecks);

    for (int i = 0; i < 8; i++) {
        int totalExecutionTime = performanceTest.test();
        totalExecutionTimes.push_back(totalExecutionTime);
        int avgLatencyUS = performanceTest.totalLatency / (1000 * performanceTest.totalProcessed);

        spdlog::info(
            "{}\ttotalExecutionTime (ms): {}\ttotalLatency (us): {:6}\tavgLatencyUS: {:4}.",
            std::get<0>(scenario),
            totalExecutionTime,
            (performanceTest.totalLatency / 1000),
            avgLatencyUS);

        averageLatencies.push_back(avgLatencyUS);
    }

    TestResults totalExecutionTimesResult(totalExecutionTimes);
    TestResults averageLatenciesResult(averageLatencies);

    spdlog::info(
        "Finished. {}\nSleep time: {:4}\tPool size: {:4}\tNumber of listeners: {:4}\tNumber of "
        "iterations: {:5}\ttotal "
        "execution time: {:5}\tAvg Latency: {:5}\tStdev Latency: {:5}",
        std::get<0>(scenario),
        inputData.sleepTime,
        inputData.poolSize,
        inputData.numListeners,
        inputData.iterations,
        totalExecutionTimesResult.average,
        averageLatenciesResult.average,
        averageLatenciesResult.stddev);
}