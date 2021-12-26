// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TIER4_AUTOWARE_UTILS__MATH__SEQUENTIAL_COMPUTATION_HPP_
#define TIER4_AUTOWARE_UTILS__MATH__SEQUENTIAL_COMPUTATION_HPP_

#include <cmath>

namespace tier4_autoware_utils
{

class Statistics{
/**
 * @brief calculate infinite step statistics
 *  cnt : number of sequence to calculate
 */

private:
    size_t cnt_;
    double mean_;
    double variance_;
    //! Update should be done only once at last.
    void updateCount(){cnt_++;}
public:
    explicit Statistics(const double x)
    {
      resetStatistics(x);
    }
    void resetStatistics(const double x){
        cnt_= 0;
        mean_ = x;
        variance_=0;
    }
    double getMean(){return mean_;}
    double getVariance(){return variance_;}
    double getStddev(){return std::sqrt(variance_);}
    double calcStddev(const double x){
        const auto calcMean = [&](const double x){
          return (cnt_ * mean_ + x) / (cnt_ + 1.0);
        };
        mean_ = calcMean(x);        
        const auto calcVariance = [&](const double x){
          return (cnt_ * (variance_ + std::pow(mean_, 2)) + std::pow(x, 2)) / (cnt_ + 1.0) -
          std::pow(mean_, 2);
        };
        variance_ = calcVariance(x);
        updateCount();
        return std::sqrt(variance_);
    }
};

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__MATH__NORMALIZATION_HPP_
