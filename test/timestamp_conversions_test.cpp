// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2018, SICK AG, Waldkirch
*  Copyright (C) 2018, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file SickSafetyscannersRos.h
 *
 * \author  Lucas Chiesa <lucas.chiesa@gmail.com>
 * \date    2021-10-20
 */
//----------------------------------------------------------------------

#include "sick_safetyscanners/SickSafetyscannersRos.h"

#include <gtest/gtest.h>

namespace sick {
namespace {

TEST(TimestampConversionMethods, isRealTimeClockAvailable_WhenDateIsSmall_ReturnsFalse)
{
    const uint16_t only_two_months = 61u;
    ASSERT_FALSE(isRealTimeClockAvailable(only_two_months));
}

TEST(TimestampConversionMethods, isRealTimeClockAvailable_WhenDateIsBig_ReturnsTrue)
{
    const uint16_t twenty_years = 365u * 20u;
    ASSERT_TRUE(isRealTimeClockAvailable(twenty_years));
}

TEST(TimestampConversionMethods, timestampToSeconds_ReturnsExpectedValues)
{
    // 1day 1hour 1min and 4seconds of uptime.
    uint16_t timestamp_date = 1u;
    uint32_t timestamp_time = (4u+60u+3600u)*1000u;
    ASSERT_DOUBLE_EQ(90064.0, timestampToSeconds(timestamp_date, timestamp_time));

    // 10 years 1 millisecond.
    timestamp_date = 10u*365u;
    timestamp_time = 1000u;
    ASSERT_DOUBLE_EQ(315360001.0, timestampToSeconds(timestamp_date, timestamp_time));
}

TEST(TimestampConversionMethods, toSecondsSinceEpoch_CorrectlyAddsTwoYears)
{
    const double two_years = 2u*365u*86400.0;
    ASSERT_DOUBLE_EQ(two_years, toSecondsSinceEpoch(0.0));
    const double uptime_seconds = 315360001.0;
    const double seconds_since_epoch = toSecondsSinceEpoch(uptime_seconds);
    ASSERT_DOUBLE_EQ(two_years, seconds_since_epoch-uptime_seconds);
}

} // namespace
} // nemespace sick
