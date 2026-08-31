// Copyright (c) 2026 -  FluidNC contributors
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>

/*
    CAN identifier map for FluidNC.

    Everything uses 11 bit standard identifiers.  On CAN the numerically lowest identifier
    wins arbitration, so the layout below is ordered by urgency: abort first, then the clock
    sync and scheduling traffic that coordinated motion depends on, then the ODrive spindle,
    then the human-scale I/O.

    ODrive CAN Simple derives its identifiers as (node_id << 5) | cmd_id, which means node 0
    would occupy 0x000-0x01F and collide with the reserved range below.  ODrive node ids are
    therefore restricted to 1..3; see ODriveSpindle::validate().
*/

namespace CAN {
    // Broadcast: every node aborts immediately, clears its scheduled event queue and holds
    // position.  Highest priority identifier on the bus.
    static constexpr uint32_t IdAbort = 0x000;

    // Clock synchronisation.  The master broadcasts a request and each node answers on its
    // own identifier so replies do not collide.
    static constexpr uint32_t IdClockRequest   = 0x001;
    static constexpr uint32_t IdClockReplyBase = 0x002;  // + node_id, node_id in 1..13
    static constexpr uint32_t IdClockReplyLast = 0x00F;

    // Scheduled events (move_to / set_pwm / set_digital) addressed to one node.
    static constexpr uint32_t IdScheduleBase = 0x010;  // + node_id
    static constexpr uint32_t IdScheduleLast = 0x01F;

    // ODrive CAN Simple occupies 0x020-0x07F for node ids 1..3.
    static constexpr uint32_t IdODriveFirst = 0x020;
    static constexpr uint32_t IdODriveLast  = 0x07F;

    // Module identity administration.  The master addresses nodes by their burned-in UUID
    // before any node id has been assigned, so these are single broadcast identifiers rather
    // than a per-node range.  Only used at bring-up and on $CM/Scan, so priority is low
    // enough to sit above human I/O but below motion.
    static constexpr uint32_t IdAdminCommand = 0x080;  // master -> all nodes
    static constexpr uint32_t IdAdminReply   = 0x081;  // matching node -> master

    // Admin opcodes, carried in byte 0 of an IdAdminCommand frame.  The UUID is a 48-bit
    // little-endian value in bytes 1..6; byte 7 carries the node id for Assign.
    enum class AdminOp : uint8_t {
        Probe   = 0,  // "are you there?"  The node with this UUID replies with its state.
        Assign  = 1,  // adopt the node id in byte 7 and reply to confirm.
        Release = 2,  // forget the assigned node id and go back to unconfigured.
    };

    // Pin extender: node -> master input bitmap, master -> node output bitmap.
    static constexpr uint32_t IdExtenderInputBase  = 0x100;  // + node_id
    static constexpr uint32_t IdExtenderOutputBase = 0x140;  // + node_id

    // Pendant / jog wheel: node -> master encoder deltas and selector state.
    static constexpr uint32_t IdPendantBase = 0x180;  // + node_id

    // Auxiliary actuators: master -> node command, node -> master completion.
    static constexpr uint32_t IdActuatorCmdBase  = 0x200;  // + node_id
    static constexpr uint32_t IdActuatorDoneBase = 0x240;  // + node_id

    // Node status / heartbeat / faults.  Lowest priority: never delays anything above.
    static constexpr uint32_t IdNodeStatusBase = 0x700;  // + node_id

    static constexpr uint32_t MaxNodeId = 0x3F;

    // Scheduled event opcodes, carried in the first payload byte of an IdScheduleBase frame.
    enum class Op : uint8_t {
        MoveTo     = 0,  // int32 cumulative step target, uint24 end time (node clock ticks)
        SetPwm     = 1,  // uint16 duty, uint32 at-time
        SetDigital = 2,  // uint8 pin, uint8 value, uint32 at-time
        Reset      = 3,  // clear queue, adopt supplied position as current, no motion
        Enable     = 4,  // uint8 enable/disable drive output
    };
}
