// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

class Channel;

// A live table of the state of every input-capable pin the configuration is not using, behind the
// $Diag/Inputs command.  It answers "which pin is this wire actually on": start the dump, move the
// switch, and read off the column that changes.
namespace InputDump {
    bool running();

    // Claims the free input pins and starts writing rows to the channel.  Restarts if already running,
    // which also moves the output to this channel.
    void start(Channel& out);

    // Releases the pins.  Safe when nothing is running.
    void stop();
}
