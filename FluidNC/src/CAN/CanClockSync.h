// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "CanBus.h"
#include "CanIds.h"

#include <cstdint>

namespace CAN {
    /*
        Klipper-style clock synchronisation for one node.

        The master periodically broadcasts a clock request.  Each node answers with its own
        free-running counter.  The master records when the request left and when the reply
        arrived, and takes the midpoint as the master-time instant at which the node's counter
        held the reported value.  Samples with an unusually long round trip are discarded,
        because on a shared bus a delayed reply skews the midpoint estimate.

        Accepted samples feed a decaying least-squares fit of node_ticks against master
        microseconds, giving both the frequency ratio and the offset.  That fit is what lets
        the master convert a planned master-time instant into the node tick value to put in a
        scheduled event, which is the whole basis of coordinated motion over CAN.
    */
    class CanClockSync {
    public:
        // Round trips longer than this are treated as bus congestion and dropped.
        static constexpr int64_t MaxAcceptableRttUs = 2000;

        // How far the predicted and observed node clocks may drift apart before the node is
        // considered unhealthy.  At 1 MHz node clocks this is 200 us of uncertainty, which is
        // far larger than a healthy fit ever produces and far smaller than anything that
        // would visibly corrupt a coordinated move.
        static constexpr double MaxAcceptableDeviationTicks = 200.0;

        explicit CanClockSync(uint8_t node_id) : _nodeId(node_id) {}

        // Called from the CAN RX task when a clock reply for this node arrives.  tx_time_us
        // is the instant the request was handed to the CAN peripheral, not the instant it
        // was queued, so that queueing delay does not bias the fit.
        void onReply(uint32_t node_ticks, int64_t tx_time_us, int64_t rx_time_us);

        // Converts a master timestamp to this node's clock domain.  Only meaningful once
        // synced() is true.
        uint32_t masterToNode(int64_t master_us) const;

        // Converts a node tick value to master microseconds.
        int64_t nodeToMaster(uint32_t node_ticks) const;

        // True once enough samples have been accepted for the fit to be trustworthy.
        bool synced() const { return _samples >= MinSamplesForSync; }

        // True when the fit is both established and still tracking the node.
        bool healthy() const { return synced() && _deviation < MaxAcceptableDeviationTicks; }

        double frequency() const { return _freq; }
        double deviation() const { return _deviation; }
        int64_t lastReplyTime() const { return _lastReplyUs; }

        // Drops the fit, e.g. after a node reset.  The next samples rebuild it from scratch.
        void reset();

    private:
        static constexpr uint32_t MinSamplesForSync = 8;

        // Exponential forgetting factor for the running least-squares sums.  Older samples
        // fade out so that the fit tracks temperature-driven drift in the node oscillator.
        // At the default 30 ms request period this is an effective window of about 30 s.
        static constexpr double Decay = 0.9990;

        // The time origin is moved forward this often so that the regression variable stays
        // small; see rebase().
        static constexpr double RebaseIntervalS = 10.0;

        uint8_t _nodeId;

        int64_t _lastReplyUs = 0;

        // Node counters are 32 bit and wrap.  We keep an unwrapped 64 bit version.
        uint32_t _lastRawTicks    = 0;
        int64_t  _tickHighBits    = 0;
        bool     _haveFirstSample = false;

        // Running weighted sums for the least-squares fit of ticks = _freq * x + _offset,
        // where x is seconds since _timeBaseUs.  Seconds rather than microseconds, and a
        // periodically advanced origin, keep the normal equations well conditioned.
        int64_t _timeBaseUs = 0;
        double  _sumW = 0, _sumX = 0, _sumY = 0, _sumXX = 0, _sumXY = 0;

        double   _freq      = 1e6;  // node ticks per second
        double   _offset    = 0.0;
        double   _deviation = 0.0;
        uint32_t _samples   = 0;

        int64_t unwrap(uint32_t raw);
        void    rebase(double shift_s);
    };
}
