/****************************************************************************
 *
 *                           Klepsydra Core Modules
 *              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/
#include <benchmark/benchmark.h>
#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
// Considerations:
// 1) Vector resized and published are the same vector.
// 2) Vector resized is previously created before EventEmitterMiddlewareProvider() call
//    with the same size which will be requested to be resized.
// 3) Ranges.
//    1st range: poolsize of the smartpool.
//    2nd range: vectorSize of the vector to be resized and published.
//    3rd range: initialization bool. true-> initialize vector. false-> do not init vector.
// ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
static void BM_KCE_21_SmartPool_Test(benchmark::State &state)
{
    // Perform setup here
    const int poolSize = state.range(0);
    const int vectorSize(state.range(1));
    const bool initialize = state.range(2);

    // This vector initialization seems be deterministic for lambda's resize() to be
    // done in EventEmitterMiddlewareProvider() initialization. Must be created with
    // the same size as the one to be resized? or Do we create it with no size, and
    // lambda's SHOULD resize() DO WHOLE WORK???
    std::vector<float> vf_vec(vectorSize);
    uint16_t published_count = 0;
    std::function<void(std::vector<float> &)> fx_initializerFunction;
    if (initialize) {
        fx_initializerFunction = ([vectorSize](std::vector<float> &vf_vector) {
            vf_vector.resize(vectorSize);
        });
    } else {
        fx_initializerFunction = nullptr;
    }
    
    kpsr::EventEmitterMiddlewareProvider<std::vector<float>> vectorProvider(nullptr,
                                                                            "vector_provider",
                                                                            poolSize,
                                                                            fx_initializerFunction,
                                                                            nullptr);
    vectorProvider.getSubscriber()->registerListener("vector listener",
        [&published_count](const std::vector<float> & event) { 
            published_count++;
        });
    // This code gets timed
    for (auto _ : state) {
        vectorProvider.getPublisher()->publish(vf_vec);
        
    }
    // end timed code.
    vectorProvider.getSubscriber()->removeListener("vector listener");

    state.counters["poolSz"] = poolSize;
    state.counters["vectorSz"] = vectorSize;
    state.counters["init"] = initialize;
    state.counters["pblshCnt"] = published_count;
}

// BENCHMARK(BM_KCE_21_SmartPool_Test)
//     ->Name("hss BM_KCE_21_SmartPool_Test")
//     ->UseRealTime()
//     ->ArgsProduct({{0, 16, 32, 64}, {0, 512, 1024, 2048}, {true, false}});

BENCHMARK(BM_KCE_21_SmartPool_Test)
    ->Name("hss BM_KCE_21_SmartPool_Test")
    ->UseRealTime()
    ->ArgsProduct({
        {0, 16, 32, 64},                            // poolSize
        benchmark::CreateDenseRange(0, 2048, 1<<9),  // vectoSize
        //{0, 512, 1024, 2048},                     // vectoSize (other valid range declaration)
        {true, false}});                            // initialize

BENCHMARK_MAIN();

//https://github.com/google/benchmark/blob/main/docs/user_guide.md#running-benchmarks

// Now arguments generated are [ 0, 16, 32, 48, 64 ].
//BENCHMARK(BM_NominalTest)->DenseRange(0, 64, 16);

// We can rename the benchamrk name as follows.
//BENCHMARK(BM_NominalTest)->Name("hidalgoss Benchmark Name")->DenseRange(0, 64, 16);

// Now arguments generated are [ 0, 128, 256, 384, 512, 640, 768, 896, 1024 ].
//BENCHMARK(BM_DenseRange)->DenseRange(0, 1024, 128);

// If the benchmarked code itself uses threads and you want to compare it to single-threaded code,
// you may want to use real-time ("wallclock") measurements for latency comparisons:
// Without UseRealTime, CPU time is used by default.
//BENCHMARK(BM_test)->Range(8, 8<<10)->UseRealTime();

//BENCHMARK(BM_NominalTest)->Threads(10)->UseRealTime()->Ranges({{0, 1}, {0, 16}});
