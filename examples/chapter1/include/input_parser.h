/*****************************************************************************
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
*****************************************************************************/
#ifndef INPUT_PARSER_H
#define INPUT_PARSER_H

#include <getopt.h>
#include <string>

#include <cstring>

class InputData
{
public:
    InputData() {}

    bool testUnsafeMulti = false;
    bool bufferChecks = true;
    int poolSize = 0;
    int numListeners = 1;
    int sleepTime = 0;
    int iterations = 100;
};

class InputParser
{
public:
    static bool getConfFileFromParams(int argc, char *argv[], InputData &inputData)
    {
        char c;
        while ((c = getopt(argc, argv, "u:p:n:s:i:b:")) != -1) {
            if (c == 255)
                break;
            switch (c) {
            case 'u':
                printf("Enabled test unsafe multi eventemitter: %s.\n", optarg);
                inputData.testUnsafeMulti = strncmp(optarg, "true", 4) == 0;
                break;
            case 'b':
                printf("Enabled buffer pointer checks : %s\n", optarg);
                inputData.bufferChecks = strncmp(optarg, "true", 4) == 0;
                break;
            case 'p':
                inputData.poolSize = std::atoi(optarg);
                printf("inputData.poolSize: %d\n", inputData.poolSize);
                break;
            case 'n':
                inputData.numListeners = std::atoi(optarg);
                printf("inputData.numListeners: %d\n", inputData.numListeners);
                break;
            case 's':
                inputData.sleepTime = std::atoi(optarg);
                printf("inputData.sleepTime: %d\n", inputData.sleepTime);
                break;
            case 'i':
                inputData.iterations = std::atol(optarg);
                printf("inputData.iterations: %d\n", inputData.iterations);
                break;
            default:
                printHelp();
                return false;
                break;
            }
        }

        return true;
    }

private:
    void static printHelp()
    {
        printf("Usage: kpsr_ai_time_example [OPTIONS]\n");
        printf("    -u enable unsafe multi eventemitter : default true\n");
        printf("    -b enable buffer pointer safety checks : default true\n");
        printf("    -p <pool size> : default 0\n");
        printf("    -n <number of listeners> : default 1\n");
        printf("    -s <sleep time in microsecs> : default 0\n");
        printf("    -i <number of iterations> : default 100\n");
        exit(0);
    }
};

#endif // INPUT_PARSER_H