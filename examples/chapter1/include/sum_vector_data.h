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
#ifndef SUM_VECTOR_DATA_H
#define SUM_VECTOR_DATA_H

#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/publisher.h>
#include <numeric>

class SumVectorData {
public:
   SumVectorData(kpsr::Subscriber<std::vector<float>> * subscriber,
                  kpsr::Publisher<float> * publisher)
      : _subscriber(subscriber)
      , _publisher(publisher)
      {
         _subscriber->registerListener("sum_vector", [this](const std::vector<float> & event) {
            float sum = calculateSum(event);
            _publisher->publish(sum);
         });
      }
      
   ~SumVectorData() {
         _subscriber->removeListener("sum_vector");
      }
      
private:
   kpsr::Subscriber<std::vector<float>> * _subscriber;
   kpsr::Publisher<float> * _publisher;
   
   float calculateSum(const std::vector<float> & event) {
       return std::accumulate(event.begin(), event.end(), 0.0f);
   }
};

#endif // SUM_VECTOR_DATA
