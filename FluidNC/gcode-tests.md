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

$H           ; Home if needed
G97          ; Set to constant RPM mode (should be default)
?            ; Check modal state - should show G97
G50 S1000    ; Set max spindle speed to 1000 RPM
?            ; Check state
M3 S500      ; Start spindle at 500 RPM
?            ; Verify spindle is running
M5           ; Stop spindle

## G96 - Constant Surface Speed

Note that for CSS mode the centerline of the spindle has to be X=0 in MCO (!). We take the TLO into 
account when calculating the centerline.

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

## G95 - Feed Per Revolution (requires spindle encoder)

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

## G33 - Spindle Synchronized Threading (requires spindle encoder)

M3 S100      ; Start spindle at 100 RPM
G0 X0 Z0     ; Move to start position
G33 Z-10 K1.5 ; Thread to Z-10 with 1.5mm pitch
?            ; Check state during move if possible
G0 Z0        ; Retract
M5

## G76 - Threading Cycle (requires spindle encoder)

M3 S100      ; Start spindle
G0 X5 Z0     ; Starting position
G76 X0 Z-10 K1.5 P0.5 Q30 ; Multi-pass threading
                          ; X0 = final diameter
                          ; Z-10 = length
                          ; K1.5 = pitch
                          ; P0.5 = depth per pass
                          ; Q30 = angle
?            ; Check state
G0 Z0
M5

# Manual rigid tapping

M5          ; Spindle off
G95 F1.0    ; Feed per rev, 1mm pitch
G1 Z-15     ; Tap to depth

The motion system doesn't dynamically reverse direction based on encoder direction. We probably 
want to implement that?

# Test Plan for Tool Table
Prerequisites

* A tooltable.yaml file exists (or will be created by G10 commands)
* Machine is homed
* Safe Z height available for moves


## Basic G43 H# - Load TLO from Tool Table

```
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

## G43.1 - Dynamic (Temporary) TLO

```
; Test 2: Dynamic TLO (not stored in tool table)

G49             ; Cancel any existing TLO
?               ; Note WPos

G43.1 Z-25.0    ; Apply temporary -25mm Z offset
?               ; WPos Z should be 25mm lower

G49             ; Cancel
?               ; WPos Z should return to original

$TT             ; Verify tool table unchanged (no new entry created)
```

## G10 L1 - Set Tool Offset Directly

```
; Test 3: Set absolute tool offset

G10 L1 P5 Z-75.5    ; Set tool 5 Z offset to -75.5mm
$TT                  ; Verify tool 5 shows Z: -75.5

G43 H5              ; Load tool 5
?                   ; Verify WPos reflects -75.5mm Z offset

G49
```

## G10 L10 - Set Offset from Current Position + WCS

```
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

## G10 L11 - Set Offset from Machine Position

```
; Test 5: Compute offset from machine coordinates

G49                 ; Cancel TLO
G53 G0 Z-100        ; Move to known machine Z position
G10 L11 P7 Z-100    ; Set tool 7 so MPos -100 = desired Z-100

$TT                 ; Check tool 7 offset (should be ~0 if ref is at -100)
G43 H7
?

G49
```

## Persistence - save and reload

```
; Test 6: Verify persistence

G10 L1 P8 X1.5 Y-2.0 Z-88.8   ; Set tool 8 with X, Y, Z offsets
$TTS                           ; Save tool table to file
$TTL                           ; Reload from file
$TT                            ; Verify tool 8 still has correct values

G43 H8
?                              ; Verify offsets applied correctly
G49
```

## Non-existing tool handling

```
; Test 7: Error handling for missing tool

G43 H999            ; Try to load non-existent tool
; Should produce an error, TLO should remain unchanged

$TT                 ; Verify table state
```

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

## Console commands

```
$TT     ; Display all tools and turret mapping
$TTL    ; Reload tool table from /localfs/tooltable.yaml
$TTS    ; Save current tool table to file
```

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
