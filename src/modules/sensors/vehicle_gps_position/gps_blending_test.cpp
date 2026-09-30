/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Test code for the GPS receiver selection logic
 * Run this test only using make tests TESTFILTER=GpsBlending
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "gps_blending.hpp"

using matrix::Vector3f;

class GpsBlendingTest : public ::testing::Test
{
public:
	sensor_gnss_s getDefaultGnssData();
	void runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gnss_s &gnss_data, int instance);
	void runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gnss_s &gnss_data0, sensor_gnss_s &gnss_data1);

	uint64_t _time_now_us{1000000};
};

sensor_gnss_s GpsBlendingTest::getDefaultGnssData()
{
	sensor_gnss_s gnss_data{};
	gnss_data.timestamp = _time_now_us - 10e3;
	gnss_data.time_utc_usec = 0;
	gnss_data.latitude = 47.0;
	gnss_data.longitude = 9.0;
	gnss_data.altitude_msl = 800.0;
	gnss_data.altitude_ellipsoid = 800.0;
	gnss_data.speed_accuracy = 0.2f;
	gnss_data.course_accuracy = 0.5f;
	gnss_data.eph = 0.7f;
	gnss_data.epv = 1.2f;
	gnss_data.hdop = 1.f;
	gnss_data.vdop = 1.f;
	gnss_data.noise = 20;
	gnss_data.jamming_indicator = 40;
	gnss_data.ground_speed = 1.f;
	gnss_data.vel_north = 1.f;
	gnss_data.vel_east = 1.f;
	gnss_data.vel_down = 1.f;
	gnss_data.course = 0.f;
	gnss_data.timestamp_time_relative = 0;
	gnss_data.fix_type = 4;
	gnss_data.vel_ned_valid = true;
	gnss_data.satellites_used = 8;

	return gnss_data;
}

void GpsBlendingTest::runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gnss_s &gnss_data, int instance)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < static_cast<int>(duration_s / dt); k++) {
		gps_blending.setGnssData(gnss_data, instance);
		gps_blending.update(_time_now_us);

		_time_now_us += dt_us;
		gnss_data.timestamp += dt_us;
	}
}

void GpsBlendingTest::runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gnss_s &gnss_data0,
				 sensor_gnss_s &gnss_data1)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < static_cast<int>(duration_s / dt); k++) {
		gps_blending.setGnssData(gnss_data0, 0);
		gps_blending.setGnssData(gnss_data1, 1);

		gps_blending.update(_time_now_us);

		_time_now_us += dt_us;
		gnss_data0.timestamp += dt_us;
		gnss_data1.timestamp += dt_us;
	}
}

TEST_F(GpsBlendingTest, noData)
{
	GpsBlending gps_blending;

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());

	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, singleReceiver)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	sensor_gnss_s gnss_data = getDefaultGnssData();

	gps_blending.setGnssData(gnss_data, 1);
	gps_blending.update(_time_now_us);

	_time_now_us += 200e3;
	gnss_data.timestamp = _time_now_us - 10e3;
	gps_blending.setGnssData(gnss_data, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: a second update is called without data
	gps_blending.update(_time_now_us);

	// THEN: no new data should be available
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverNoBlending)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers with the same prioity, gps0 only has a 2D fix
	gps_blending.setPrimaryInstance(-1);
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_2D;
	gps_blending.setGnssData(gnss_data0, 0);
	gps_blending.setGnssData(gnss_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: gps1 should be selected because it is the only one with a 3D fix
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	_time_now_us += 200e3;
	gnss_data0.timestamp = _time_now_us - 10e3;
	gnss_data1.timestamp = _time_now_us - 10e3;
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_3D;
	gnss_data0.satellites_used = gnss_data1.satellites_used + 2;
	gps_blending.setGnssData(gnss_data0, 0);
	gps_blending.setGnssData(gnss_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: gps1 should still be selected, the satellite count isn't used for ranking
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverFailover)
{
	GpsBlending gps_blending;

	// GIVEN: a dual GPS setup with the first instance (0)
	// set as primary
	gps_blending.setPrimaryInstance(0);

	// WHEN: only the secondary receiver is available
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	const float duration_s = 10.f;
	runSeconds(duration_s, gps_blending, gnss_data1, 1);

	// THEN: the secondary instance as the primary one is not available
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT WHEN: the data of the primary receiver is avaialbe
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the secondary instance is kept for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the primary instance is selected and the data
	// is available
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	runSeconds(duration_s, gps_blending, gnss_data0, gnss_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT WHEN: the primary receiver isn't available anymore
	runSeconds(duration_s, gps_blending, gnss_data1, 1);

	// THEN: the data of the secondary receiver can be used
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// AND IF: the primary receiver is available again
	gnss_data0.timestamp = gnss_data1.timestamp;

	runSeconds(3.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the primary receiver should be used again after the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: the secondary receiver has a better fix type and accuracy than the primary one
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;
	gnss_data1.eph = 0.02f;
	gnss_data1.epv = 0.03f;

	runSeconds(5.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selector shouldn't switch again as the primary one is available
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: the primary receiver looses its fix
	gnss_data0.fix_type = 1;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selector should switch as the primary one is unable to provide correct data
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, preferredFailoverAfterHold)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers, gps0 preferred
	gps_blending.setPrimaryInstance(0);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: the preferred receiver doesn't meet the EKF2 eph requirement
	gnss_data0.eph = 4.f;

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: it is kept for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// AND WHEN: it meets the requirement again before the hold time elapses, then fails again
	gnss_data0.eph = 0.7f;

	runSeconds(0.5f, gps_blending, gnss_data0, gnss_data1);

	gnss_data0.eph = 4.f;

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: no switch happens as the hold time restarted
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// BUT WHEN: it doesn't meet the requirement for longer than the hold time
	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the receiver meeting the requirements is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, preferredFailoverOnlyToBetterQuality)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(0);

	// GIVEN: two receivers, gps0 preferred, neither meeting the EKF2 eph requirement
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	gnss_data0.eph = 4.f;
	gnss_data1.eph = 3.5f;

	runSeconds(5.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the preferred receiver is kept as the other one doesn't meet the requirements either
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: the preferred receiver loses its 3D fix
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_2D;

	runSeconds(0.1f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selector switches immediately to the receiver that still has a 3D fix
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, preferredReturnWithHold)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(0);

	// GIVEN: gps0 preferred but not available
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(3.f, gps_blending, gnss_data1, 1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 publishes but doesn't meet the EKF2 eph requirement
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.eph = 4.f;

	runSeconds(5.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: gps1 is kept
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 meets the requirements
	gnss_data0.eph = 0.7f;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: gps1 is kept for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// AND WHEN: gps0 fails the requirements again before the hold time elapses, then meets them again
	gnss_data0.eph = 4.f;

	runSeconds(0.5f, gps_blending, gnss_data0, gnss_data1);

	gnss_data0.eph = 0.7f;

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: gps1 is still kept as the hold time restarted
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// BUT WHEN: gps0 meets the requirements for longer than the hold time
	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection returns to the preferred receiver
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, preferredArmedOnlySwitchesOnFailure)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(0);

	// GIVEN: an armed vehicle using gps0, the preferred receiver
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(2.f, gps_blending, gnss_data0, gnss_data1);
	gps_blending.setArmed(true);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps0 stops meeting the EKF2 eph requirement
	gnss_data0.eph = 4.f;

	runSeconds(2.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection fails over after the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 meets the requirements again
	gnss_data0.eph = 0.7f;

	runSeconds(5.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: gps1 is kept while armed, as it didn't fail
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: the vehicle disarms
	gps_blending.setArmed(false);

	runSeconds(2.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection returns to the preferred receiver after the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: armed again and gps0 loses its fix
	gps_blending.setArmed(true);
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_NONE;

	runSeconds(0.1f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection fails over immediately
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
}

TEST_F(GpsBlendingTest, rankedAccuracyWithHold)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers with the same priority and the same accuracy
	gps_blending.setPrimaryInstance(-1);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(2.f, gps_blending, gnss_data0, gnss_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps1 reports a slightly better accuracy and a better fix type
	gnss_data1.eph = 0.6f;
	gnss_data1.epv = 1.0f;
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FIXED;

	runSeconds(5.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection is kept, the difference is within the switching margin
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps1 reports a clearly better accuracy
	gnss_data1.eph = 0.3f;
	gnss_data1.epv = 0.5f;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// AND WHEN: gps1 degrades again before the hold time elapses
	gnss_data1.eph = gnss_data0.eph;
	gnss_data1.epv = gnss_data0.epv;

	runSeconds(0.5f, gps_blending, gnss_data0, gnss_data1);

	// AND: is clearly better again, but not for the full hold time
	gnss_data1.eph = 0.3f;
	gnss_data1.epv = 0.5f;

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: no switch happens as the hold time restarted
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// BUT WHEN: gps1 stays clearly better for longer than the hold time
	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the more accurate receiver is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// AND WHEN: gps0 becomes slightly more accurate than gps1
	gnss_data0.eph = 0.25f;
	gnss_data0.epv = 0.45f;

	runSeconds(5.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection is kept
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
}

TEST_F(GpsBlendingTest, rankedUpdateRate)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers with the same accuracy, gps0 at 5 Hz and gps1 at 10 Hz
	gps_blending.setPrimaryInstance(-1);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	const uint64_t dt_us = 100_ms;

	auto run = [&](int steps) {
		for (int k = 0; k < steps; k++) {
			if ((k % 2) == 0) {
				gps_blending.setGnssData(gnss_data0, 0);
				gnss_data0.timestamp += 2 * dt_us;
			}

			gps_blending.setGnssData(gnss_data1, 1);
			gnss_data1.timestamp += dt_us;

			gps_blending.update(_time_now_us);
			_time_now_us += dt_us;
		}
	};

	run(10);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	run(30);

	// THEN: the receiver with the higher update rate is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: the slower receiver is clearly more accurate
	gnss_data0.eph = 0.3f;
	gnss_data0.epv = 0.5f;

	run(30);

	// THEN: the accuracy has precedence over the update rate
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
}

TEST_F(GpsBlendingTest, rankedSwitchOnFailure)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);

	// GIVEN: two equivalent receivers, gps0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(2.f, gps_blending, gnss_data0, gnss_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps0 loses its fix
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_NONE;

	runSeconds(0.1f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selector switches immediately
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 recovers and gps1 no longer meets the EKF2 requirements
	gnss_data0.fix_type = gnss_data1.fix_type;
	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	gnss_data1.eph = 4.f;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the receiver meeting the requirements is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps1 recovers and gps0 stops publishing
	gnss_data1.eph = gnss_data0.eph;
	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	runSeconds(3.f, gps_blending, gnss_data1, 1);

	// THEN: the selector switches once gps0 timed out
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, rankedMinimumRequirements)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	gps_blending.setMinimumRequirements(sensor_gnss_s::FIX_TYPE_3D, 1.f, 3.f);

	// GIVEN: gps0 has a better fix type but doesn't meet the eph requirement,
	// gps1 meets all requirements
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_RTK_FLOAT;
	gnss_data0.eph = 1.5f;
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_3D;
	gnss_data1.eph = 0.8f;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: gps0, selected by default, is kept for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the receiver meeting the requirements is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 meets the requirements and is clearly more accurate
	gnss_data0.eph = 0.3f;
	gnss_data0.epv = 0.5f;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the more accurate receiver wins after the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
}

TEST_F(GpsBlendingTest, rankedArmedOnlySwitchesOnFailure)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);

	// GIVEN: an armed vehicle with two equivalent receivers, gps0 selected
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	runSeconds(2.f, gps_blending, gnss_data0, gnss_data1);
	gps_blending.setArmed(true);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps1 reports a clearly better accuracy
	gnss_data1.eph = 0.3f;
	gnss_data1.epv = 0.5f;

	runSeconds(5.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: gps0 is kept while armed, as it didn't fail
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps0 stops meeting the EKF2 eph requirement
	gnss_data0.eph = 4.f;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the receiver meeting the requirements is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 recovers and is clearly more accurate than gps1
	gnss_data0.eph = 0.1f;
	gnss_data0.epv = 0.2f;

	runSeconds(5.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: gps1 is kept while armed
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: the vehicle disarms
	gps_blending.setArmed(false);

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	runSeconds(1.5f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the more accurate receiver is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
}

TEST_F(GpsBlendingTest, rankedExtrapolatedFixNotUsable)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);

	// GIVEN: gps0 extrapolating (dead reckoning), gps1 with a 3D fix
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_EXTRAPOLATED;
	gnss_data0.eph = 0.1f;
	gnss_data0.epv = 0.1f;
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_3D;

	runSeconds(3.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the 3D fix is selected even if the extrapolated fix type value and accuracy are better
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps1 starts extrapolating and gps0 gets a 3D fix
	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_3D;
	gnss_data1.fix_type = sensor_gnss_s::FIX_TYPE_EXTRAPOLATED;

	runSeconds(0.1f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the switch happens immediately
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
}

TEST_F(GpsBlendingTest, singleReceiverAntennaOffset)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	sensor_gnss_s gnss_data = getDefaultGnssData();

	const Vector3f offset0(0.1f, 0.0f, -0.05f);
	gps_blending.setAntennaOffset(offset0, 1);

	gps_blending.setGnssData(gnss_data, 1);
	gps_blending.update(_time_now_us);

	_time_now_us += 200e3;
	gnss_data.timestamp = _time_now_us - 10e3;
	gps_blending.setGnssData(gnss_data, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	const Vector3f &out = gps_blending.getOutputAntennaOffset();
	EXPECT_FLOAT_EQ(out(0), offset0(0));
	EXPECT_FLOAT_EQ(out(1), offset0(1));
	EXPECT_FLOAT_EQ(out(2), offset0(2));
}

TEST_F(GpsBlendingTest, failoverAntennaOffset)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(0);

	const Vector3f offset0(0.1f, 0.0f, 0.0f);
	const Vector3f offset1(-0.1f, 0.0f, 0.0f);
	gps_blending.setAntennaOffset(offset0, 0);
	gps_blending.setAntennaOffset(offset1, 1);

	// Only secondary available
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	runSeconds(10.f, gps_blending, gnss_data1, 1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_FLOAT_EQ(gps_blending.getOutputAntennaOffset()(0), offset1(0));

	// Now primary becomes available for longer than the hold time
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.timestamp = gnss_data1.timestamp;
	runSeconds(3.f, gps_blending, gnss_data0, gnss_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FLOAT_EQ(gps_blending.getOutputAntennaOffset()(0), offset0(0));
}

TEST_F(GpsBlendingTest, dualReceiverNoBlendingStaleFlag)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers
	gps_blending.setPrimaryInstance(-1);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	gnss_data0.fix_type = sensor_gnss_s::FIX_TYPE_2D; // gps1 wins selection

	// First update: both receivers provide data, gps1 is selected
	gps_blending.setGnssData(gnss_data0, 0);
	gps_blending.setGnssData(gnss_data1, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// Second update: NO new data from either receiver
	// With the bug (_gps_updated[gps_select_index] instead of [i]),
	// the non-selected instance's flag was never cleared, so this
	// would spuriously report new data available.
	gps_blending.update(_time_now_us);

	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}
