// Copyright 2020 Tier IV, Inc.
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

#include "tier4_autoware_utils/math/sequential_computation.hpp"

#include <gtest/gtest.h>

TEST(sequential_computation, calcVariance)
{
  using tier4_autoware_utils::Statistics;
  Statistics stat(0.0);
  int cnt =0;
  stat.calcStddev(0.0);
  EXPECT_DOUBLE_EQ(stat.getMean(), 0.0);
  EXPECT_DOUBLE_EQ(stat.getVariance(), 0.0);
  EXPECT_DOUBLE_EQ(stat.getStddev(), 0.0);
  stat.calcStddev(1.0);
  EXPECT_DOUBLE_EQ(calcVariance(0.0,variance), 0.5);
  stat.calcStddev(2.0);
  EXPECT_DOUBLE_EQ(calcVariance(0.0,variance), 1.0);
  cnt++;
}
