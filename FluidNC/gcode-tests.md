# TODO list

- Tool table helemaal afmaken
- ATC tool changes moeten rekening houden met tool table
- Probing


# G-Code Functions:

G33 - Spindle synchronized motion (single-pass threading) - requires spindle encoder
G76 - Multi-pass threading canned cycle - requires spindle encoder
G95 - Feed per revolution mode (instead of per minute) - requires spindle encoder
G96 - Constant Surface Speed mode (CSS for lathe)
G97 - Constant RPM mode (default)
G50 - Set maximum spindle speed for CSS mode

# Test Plan:

## G97 & G50

```gcode
$H           ; Home if needed
G97          ; Set to constant RPM mode (should be default)
?            ; Check modal state - should show G97
G50 S1000    ; Set max spindle speed to 1000 RPM
?            ; Check state
M3 S500      ; Start spindle at 500 RPM
?            ; Verify spindle is running
M5           ; Stop spindle
```

## CSS and G50

```gcode
G50 S500
G96 S200
M3
G0 X200
G1 X5 F300
```

Expected behavior: 

At X=200: uncapped RPM ≈ 159 (below cap, should show ~159)
At X=63.7: RPM = 500 (exactly at cap boundary)
At X<63.7: would exceed 500, so capped at 500

During the G1 move inward, you should see the speed ramp up from ~159 toward 500, then hold at 500 for the rest of the cut.

**Pass**

## G96 - Constant Surface Speed

Note that for CSS mode the centerline of the spindle has to be X=0 in MCO (!). We take the TLO into 
account when calculating the centerline.

```gcode
$H                    ; Home
G0 X-50               ; Move to 50mm from center (machine coordinates)
G97                   ; Constant RPM mode
G50 S500              ; Set max RPM to 500
G96 S31.4             ; CSS mode: 31.4 m/min surface speed
M3                    ; Start spindle - should be ~100 RPM at radius 50mm
G0 X-25               ; Move to 25mm from center - should be ~200 RPM
G0 X-10               ; Move to 10mm from center - should be ~500 RPM (capped)
M5
G97
```

## G95 - Feed Per Revolution (requires spindle encoder)

```gcode
; Simplest test first:
M3 S100
G95
G1 Z-10 F1.0
?
G94
G0 Z0
M5

; First check if spindle encoder is configured
M3 S100      ; Start spindle
G94          ; Units per minute (default)
G1 X10 F100  ; Normal feed rate
?            ; Check state
G95          ; Switch to feed per revolution
G1 X20 F0.5  ; F is now mm/rev (0.5mm per spindle revolution)
?            ; Check state - should show G95
G94          ; Back to units per minute
M5
```

**Pass** **RETEST**

## G33 - Spindle Synchronized Threading (requires spindle encoder)

```gcode
M3 S100      ; Start spindle at 100 RPM
G0 X0 Z0     ; Move to start position
G33 Z-10 K1.5 ; Thread to Z-10 with 1.5mm pitch
?            ; Check state during move if possible
G0 Z0        ; Retract
M5
```

**Pass**

# Manual rigid tapping

```gcode
M5          ; Spindle off
G95 F1.0    ; Feed per rev, 1mm pitch
G1 Z-15     ; Tap to depth
```

The motion system doesn't dynamically reverse direction based on encoder direction. We probably 
want to implement that? Or not? I'm not sure.

There's currently also no way to 'stop' this once you've given the command.

All in all, it doesn't work at the moment and I don't have a viable path forward.

**Fail**

# Test Plan for Tool Table

Prerequisites

* A tooltable.yaml file exists (or will be created by G10 commands)
* Machine is homed
* Safe Z height available for moves

## Basic G43 H# - Load TLO from Tool Table

```gcode
; Test 1: Load TLO from tool table
; Prerequisite: tool1 exists in tooltable.yaml with known Z offset (e.g., z: -50.0)

G21 G90         ; mm, absolute
G54             ; Select WCS 1
G49             ; Ensure TLO cancelled
?               ; Query position - note WPos Z

G43 H1          ; Load tool 1 offset
?               ; Query position - WPos Z should decrease by tool1's Z offset

G49             ; Cancel TLO
?               ; Query position - WPos Z should return to original
```

**Pass**

## G43.1 - Dynamic (Temporary) TLO

```gcode
; Test 2: Dynamic TLO (not stored in tool table)

G49             ; Cancel any existing TLO
?               ; Note WPos

G43.1 Z-25.0    ; Apply temporary -25mm Z offset
?               ; WPos Z should be 25mm lower

G49             ; Cancel
?               ; WPos Z should return to original

$TT             ; Verify tool table unchanged (no new entry created)
```

**Pass**

## G10 L1 - Set Tool Offset Directly

```gcode
; Test 3: Set absolute tool offset

G10 L1 P5 Z-75.5    ; Set tool 5 Z offset to -75.5mm
$TT                  ; Verify tool 5 shows Z: -75.5

G43 H5              ; Load tool 5
?                   ; Verify WPos reflects -75.5mm Z offset

G49
```

**Pass**

## G10 L10 - Set Offset from Current Position + WCS

```gcode
; Test 4: Compute offset from probed position
; Scenario: Touch off on reference surface, store the offset

G54                 ; Select WCS
G49                 ; Cancel TLO
G0 Z10              ; Move to known position
G10 L10 P6 Z0       ; Set tool 6 offset so current pos becomes Z0 in WCS

$TT                 ; Check tool 6 offset value
G43 H6              ; Load it
?                   ; Z should now read 0 (or close)

G49
```

**Pass**

## G10 L11 - Set Offset from Machine Position

```gcode
; Test 5: Compute offset from machine coordinates

G49                 ; Cancel TLO
G53 G0 Z-100        ; Move to known machine Z position
G10 L11 P7 Z-100    ; Set tool 7 so MPos -100 = desired Z-100

$TT                 ; Check tool 7 offset (should be ~0 if ref is at -100)
G43 H7
?

G49
```

**Pass**

## Persistence - save and reload

```gcode
; Test 6: Verify persistence

G10 L1 P8 X1.5 Y-2.0 Z-88.8   ; Set tool 8 with X, Y, Z offsets
$TTS                           ; Save tool table to file
$TTL                           ; Reload from file
$TT                            ; Verify tool 8 still has correct values

G43 H8
?                              ; Verify offsets applied correctly
G49
```

**Pass**

## Non-existing tool handling

```gcode
; Test 7: Error handling for missing tool

G43 H999            ; Try to load non-existent tool
; Should produce an error, TLO should remain unchanged

$TT                 ; Verify table state
```

**Pass**

## Non-existing tool handling 2

```gcode
; Test 7: Error handling for missing tool

T1 M6              ; Load existing tool
T999 M6            ; Try to load non-existent tool
T1 M6              ; Should be a no-op!
```

**Pass**

## Turret mapping

```
; Test 8: Turret position mapping
; Requires turret section in tooltable.yaml:
;   turret:
;     position1: 5
;     position2: 8

; This is tested via ATC integration - M6 T1 should load tool 5's offset
; if position1 maps to tool 5
```

**FAIL** -> I don't think this implementation works, nor do I think the mapping is used 
internally. Also, I don't think we can set the turret positions. `setTurretMapping` is just 
never called, ever.

## Console commands

```gcode
$TT     ; Display all tools and turret mapping
$TTL    ; Reload tool table from /localfs/tooltable.yaml
$TTS    ; Save current tool table to file
```

**Pass**

# Validation Checklist

| Test	    | Expected Result
|-----------|------------------
| G43 H#	| WPos changes by tool's offset
| G43.1	    | Temporary offset applied, not saved
| G49	    | TLO cancelled, WPos restored
| G10 L1	| Absolute offset stored
| G10 L10	| Offset computed from WPos + WCS
| G10 L11	| Offset computed from MPos
| $TTS/$TTL	| Offsets persist across reload
| Invalid H# | Error returned, state unchanged

# Tool table

```
; --- Setup ---
G21 G90 G54
G92 X0 Y0 Z0

; --- G10 L1: Set tool offset directly ---
G10 L1 P1 X10 Z-5
$#
; LOOK FOR: [TLO:...] unchanged (G10 L1 doesn't activate TLO)
; Verify tool table: $T (or inspect /localfs/tooltable.yaml)

; --- G10 L10: Set offset so current position = given work position ---
; At MPos:0, this computes TLO = MPos - WCS - G92 - desiredWPos
G10 L10 P2 X0 Z0
$#
; LOOK FOR: Tool 2 offset should be written to tool table

; --- G10 L11: Set offset relative to G59.3 ---
G10 L2 P9 X5 Z5
; (set G59.3 to X5 Z5 first)
G10 L11 P3 X0 Z0
; LOOK FOR: Tool 3 offset = MPos - G59.3 - desiredWPos = 0 - 5 - 0 = -5

; --- G10 L2 / L20: Set coordinate system offset ---
G10 L2 P1 X0 Y0 Z0
; Sets G54 to X0Y0Z0
$#
; LOOK FOR: [G54:0.000,0.000,0.000,...]

G10 L20 P2 X0 Y0 Z0
; Sets G55 so that current position IS X0Y0Z0
; G55 = MPos - G92 - TLO - desiredWPos
$#
; LOOK FOR: [G55:...] computed from current position
```

**Pass**

# TLO

```
; --- Setup: create tool offsets ---
G10 L1 P1 X10 Z-5
G10 L1 P2 X0 Z-20

; --- G43 H1: Load TLO from tool table ---
G43 H1
$#
; LOOK FOR: [TLO:10.000,0.000,-5.000,...]
; LOOK FOR: ? shows WCO changed by TLO amount

; --- G43 H2: Switch to different tool ---
G43 H2
$#
; LOOK FOR: [TLO:0.000,0.000,-20.000,...]

; --- G43.1 X5 Z-3: Dynamic TLO from axis words ---
G43.1 X5 Z-3
$#
; LOOK FOR: [TLO:5.000,0.000,-3.000,...]

; --- G49: Cancel TLO ---
G49
$#
; LOOK FOR: [TLO:0.000,0.000,0.000,...]
; LOOK FOR: ? shows WCO back to just G54+G92

; --- G43 H0: Should also cancel (same as G49) ---
G43 H0
$#
; LOOK FOR: [TLO:0.000,0.000,0.000,...]
```

**Pass**

# Cutter radius compensation (G40/G41/G42)

```gcode

; --- Setup: Set tool radius in tool table ---
G10 L1 P1 X0 Z0
; Then manually set radius (or use the tool table yaml)
; For now, use D word with tool number

; --- G41 D1: Activate left compensation ---
G41 D1
; LOOK FOR: $G should show G41 - ok
; NOTE: Will warn if tool 1 has no radius defined

; --- Move with compensation active ---
G1 X50 F200
G1 Y50
G1 X0
G1 Y0
; LOOK FOR: ? during motion - MPos should be offset from
; programmed path by the tool radius amount (left side)

; --- G40: Cancel compensation ---
G40
G1 X10
; LOOK FOR: $G should show G40

; --- G42 D1: Right compensation ---
G42 D1
G1 X10 F200
G1 X15 F200
G1 Y10
; LOOK FOR: Offset to the right of the programmed path
G40
G1 X0 Y0

; --- D0 should also cancel ---
G41 D0
; LOOK FOR: $G should show G40 (D0 cancels)
```

**Pass**

# G33 (Spindle Synchronized Motion) -- NEEDS ENCODER

```gcode
; --- Requires spindle encoder connected ---
; G33 does a single-pass threading move synced to spindle rotation
; K = thread pitch (mm per revolution)

G90 G21
G0 X0 Z5
M3 S300
; Wait for spindle to reach speed

; --- Single threading pass: 1mm pitch, move Z to -20 ---
G33 Z-20 K1.0
; LOOK FOR: ? during motion should show State:Run
; Motion should be synchronized to spindle speed
; Feed rate = K * spindle_RPM (automatic, not F word)

G0 Z5
; Retract

M5
```

**Pass**

# G76 (Multi-pass Threading Cycle) -- NEEDS ENCODER

```gcode
; --- Requires spindle encoder connected ---
; G76 does multiple G33 passes at increasing depth

G90 G21
G0 X10 Z5
M3 S300

; LinuxCNC G76 parameters:
;   P = pitch (mm/rev)
;   Z = end Z position (thread length)
;   J = initial cut depth, first pass (required, > 0)
;   K = full thread depth from start X position (required, > 0)
;   I = taper at Z end (optional, 0 = no taper)
;   Q = compound infeed angle degrees (optional, 0 = straight)
;   H = spring passes at final depth (optional, 0)
;   R = depth degression (optional, 1.0 = constant area)
;
; Example: 1mm pitch, Z end at -20, 0.5mm first cut, 2mm total depth
G76 P1.0 Z-20 J0.5 K2.0

; LOOK FOR: Multiple passes cycling X deeper each time:
;   rapid to depth -> thread along Z -> retract X -> rapid back to Z start
;   In total we'll get roughly 14 passes.
; For 4 passes of 0.5 depth we do:
G76 P1.0 Z-20 J0.5 K2.0 R2.0

; Each pass cuts deeper by a decreasing amount (constant chip load)
; ? during motion shows Run state, X should vary between passes

M5
```


```gcode
M5
G90 G21
G0 X10 Z5
M3 S300
G76 P1.0 Z-20 J0.5 K2.0 R2.0
?
M5
```
