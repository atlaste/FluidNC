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
