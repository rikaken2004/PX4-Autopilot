/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file gps_blending.cpp
 */

#include "gps_blending.hpp"


void GpsBlending::update(uint64_t hrt_now_us)
{
	_is_new_output_data_available = false;

	updateReceiverTimeouts(hrt_now_us);

	const bool has_preferred = (_primary_instance >= 0) && (_primary_instance < GPS_MAX_RECEIVERS_BLEND);
	const int gps_select_index = has_preferred ? selectPreferredReceiver(hrt_now_us) : selectRankedReceiver(hrt_now_us);

	_selected_gps = gps_select_index;
	_output_antenna_offset = _antenna_offset[gps_select_index];
	_is_new_output_data_available =  _gps_updated[gps_select_index];

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		// clear updated flags
		_gps_updated[i] = false;
		_time_prev_us[i] = _gnss_state[i].timestamp;
	}
}

void GpsBlending::updateReceiverTimeouts(uint64_t hrt_now_us)
{
	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		const uint64_t timestamp = _gnss_state[i].timestamp;

		if ((timestamp > _time_prev_us[i]) && (timestamp - _time_prev_us[i] < GPS_TIMEOUT_US)) {
			// the first sample of a receiver has no previous one to compute an update interval from
			if (_time_prev_us[i] > 0) {
				const float dt = 1e-6f * (timestamp - _time_prev_us[i]);
				// seed the filter with the first interval so that the update rate is usable for the receiver ranking right away
				_gnss_dt[i] = (_gnss_dt[i] > 0.f) ? (0.1f * dt + 0.9f * _gnss_dt[i]) : dt;
			}

		} else if ((timestamp > 0) && (hrt_now_us >= timestamp + GPS_TIMEOUT_US)) {
			// Timed out - kill the stored fix for this receiver
			_gnss_state[i].timestamp = 0;
			_gnss_state[i].fix_type = 0;
			_gnss_state[i].satellites_used = 0;
			_gnss_state[i].vel_ned_valid = 0;
		}
	}
}

int GpsBlending::selectPreferredReceiver(uint64_t hrt_now_us)
{
	const int preferred = _primary_instance;
	const int current = _selected_gps;

	int candidate = -1;

	if ((current != preferred) && (receiverQuality(preferred) == ReceiverQuality::Qualified)) {
		// Return to the preferred receiver
		candidate = preferred;

	} else {
		// Fail over to the best receiver with a better quality than the selected one
		const ReceiverQuality current_quality = receiverQuality(current);

		for (int i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
			if ((i != current) && (receiverQuality(i) > current_quality)
			    && ((candidate < 0) || isBetterReceiver(i, candidate))) {
				candidate = i;
			}
		}
	}

	return switchAfterHold(current, candidate, hrt_now_us);
}

uint8_t GpsBlending::effectiveFixType(uint8_t fix_type)
{
	return (fix_type == sensor_gnss_s::FIX_TYPE_EXTRAPOLATED) ? sensor_gnss_s::FIX_TYPE_NONE : fix_type;
}

GpsBlending::ReceiverQuality GpsBlending::receiverQuality(int instance) const
{
	const sensor_gnss_s &gnss = _gnss_state[instance];
	const uint8_t fix_type = effectiveFixType(gnss.fix_type);

	// A timed out receiver has its timestamp and fix type cleared
	if ((gnss.timestamp == 0) || (fix_type < sensor_gnss_s::FIX_TYPE_3D)) {
		return ReceiverQuality::Unusable;
	}

	if ((fix_type >= effectiveFixType(_req_fix_type))
	    && (gnss.eph <= _req_eph)
	    && (gnss.epv <= _req_epv)) {
		return ReceiverQuality::Qualified;
	}

	return ReceiverQuality::NotQualified;
}

float GpsBlending::positionVariance(int instance) const
{
	const sensor_gnss_s &gnss = _gnss_state[instance];

	if (!(gnss.eph > 0.f) || !(gnss.epv > 0.f)) {
		// accuracy not reported
		return FLT_MAX;
	}

	return gnss.eph * gnss.eph + gnss.epv * gnss.epv;
}

bool GpsBlending::isBetterReceiver(int a, int b) const
{
	const ReceiverQuality quality_a = receiverQuality(a);
	const ReceiverQuality quality_b = receiverQuality(b);

	if (quality_a != quality_b) {
		return quality_a > quality_b;
	}

	if (quality_a == ReceiverQuality::Unusable) {
		return false;
	}

	// Compare the position accuracy with a margin so that noise in the reported accuracy doesn't cause switching
	const float variance_ratio = GPS_SWITCH_ACCURACY_RATIO * GPS_SWITCH_ACCURACY_RATIO;
	const float variance_a = positionVariance(a);
	const float variance_b = positionVariance(b);

	if (variance_a < variance_ratio * variance_b) {
		return true;
	}

	if (variance_b < variance_ratio * variance_a) {
		return false;
	}

	// Comparable accuracy: prefer a clearly higher update rate
	return (_gnss_dt[a] > 0.f) && (_gnss_dt[b] > 0.f) && (_gnss_dt[a] < GPS_SWITCH_INTERVAL_RATIO * _gnss_dt[b]);
}

int GpsBlending::selectRankedReceiver(uint64_t hrt_now_us)
{
	const int current = _selected_gps;

	// Find the best receiver among the ones ranking better than the selected one
	int candidate = -1;

	for (int i = 0; i < GPS_MAX_RECEIVERS_BLEND; i++) {
		if ((i != current) && isBetterReceiver(i, current)
		    && ((candidate < 0) || isBetterReceiver(i, candidate))) {
			candidate = i;
		}
	}

	return switchAfterHold(current, candidate, hrt_now_us);
}

int GpsBlending::switchAfterHold(int current, int candidate, uint64_t hrt_now_us)
{
	if (candidate < 0) {
		_switch_candidate = -1;
		return current;
	}

	if (receiverQuality(current) == ReceiverQuality::Unusable) {
		// The selected receiver timed out or lost its 3D fix
		_switch_candidate = -1;
		return candidate;
	}

	if (_switch_candidate != candidate) {
		// start the hold time for this candidate
		_switch_candidate = candidate;
		_switch_candidate_since_us = hrt_now_us;
		return current;
	}

	if ((hrt_now_us - _switch_candidate_since_us) >= GPS_SWITCH_HOLD_US) {
		_switch_candidate = -1;
		return candidate;
	}

	return current;
}
