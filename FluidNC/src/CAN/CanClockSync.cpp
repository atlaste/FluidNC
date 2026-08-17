// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "CanClockSync.h"

#include "../Logging.h"

#include <cmath>

namespace CAN {
    int64_t CanClockSync::unwrap(uint32_t raw) {
        if (_haveFirstSample && raw < _lastRawTicks && (_lastRawTicks - raw) > 0x80000000u) {
            _tickHighBits += 0x100000000LL;
        }
        _lastRawTicks    = raw;
        _haveFirstSample = true;
        return _tickHighBits + int64_t(raw);
    }

    void CanClockSync::rebase(double shift_s) {
        // Move the time origin forward by shift_s and transform the accumulated sums so the
        // fit is unchanged.  Keeping x small and centred is what stops the normal equations
        // from losing precision after the machine has been running for hours.
        _timeBaseUs += int64_t(shift_s * 1e6);

        double d = shift_s;
        _sumXX   = _sumXX - 2.0 * d * _sumX + d * d * _sumW;
        _sumX    = _sumX - d * _sumW;
        _sumXY   = _sumXY - d * _sumY;
        _offset  = _offset + _freq * d;
    }

    void CanClockSync::onReply(uint32_t node_ticks, int64_t tx_time_us, int64_t rx_time_us) {
        _lastReplyUs = rx_time_us;

        if (tx_time_us == 0) {
            return;  // A reply we did not ask for; nothing to anchor it against.
        }

        int64_t rtt = rx_time_us - tx_time_us;

        if (rtt < 0 || rtt > MaxAcceptableRttUs) {
            // The bus was busy.  Using this sample would bias the midpoint estimate, so drop
            // it; the next request is only a few tens of milliseconds away.
            return;
        }

        // Best estimate of when the node latched its counter: halfway through the round trip.
        int64_t master_us = rx_time_us - rtt / 2;
        int64_t ticks     = unwrap(node_ticks);

        if (_samples == 0) {
            _timeBaseUs = master_us;
        }

        double x = double(master_us - _timeBaseUs) * 1e-6;
        if (x > RebaseIntervalS) {
            rebase(x - RebaseIntervalS / 2);
            x = double(master_us - _timeBaseUs) * 1e-6;
        }

        double y = double(ticks);

        if (_samples >= MinSamplesForSync) {
            // Measure the sample against the existing fit before folding it in, so that a
            // node which has been reset or has drifted badly shows up as a large deviation
            // instead of being quietly absorbed.
            _deviation = std::fabs(y - (_freq * x + _offset));
        }

        _sumW  = _sumW * Decay + 1.0;
        _sumX  = _sumX * Decay + x;
        _sumY  = _sumY * Decay + y;
        _sumXX = _sumXX * Decay + x * x;
        _sumXY = _sumXY * Decay + x * y;

        ++_samples;

        double denom = _sumW * _sumXX - _sumX * _sumX;
        if (std::fabs(denom) > 1e-9) {
            _freq   = (_sumW * _sumXY - _sumX * _sumY) / denom;
            _offset = (_sumY - _freq * _sumX) / _sumW;
        }
    }

    uint32_t CanClockSync::masterToNode(int64_t master_us) const {
        double x     = double(master_us - _timeBaseUs) * 1e-6;
        double ticks = _freq * x + _offset;
        return uint32_t(int64_t(ticks) & 0xFFFFFFFFLL);
    }

    int64_t CanClockSync::nodeToMaster(uint32_t node_ticks) const {
        if (_freq <= 0.0) {
            return 0;
        }
        // node_ticks is the low 32 bits of the node counter; lift it into the same unwrapped
        // space the fit was built in before inverting.
        int64_t unwrapped = _tickHighBits + int64_t(node_ticks);
        double  x         = (double(unwrapped) - _offset) / _freq;
        return _timeBaseUs + int64_t(x * 1e6);
    }

    void CanClockSync::reset() {
        _lastRawTicks    = 0;
        _tickHighBits    = 0;
        _haveFirstSample = false;
        _sumW = _sumX = _sumY = _sumXX = _sumXY = 0.0;
        _freq                                   = 1e6;
        _offset                                 = 0.0;
        _deviation                              = 0.0;
        _samples                                = 0;
    }
}
