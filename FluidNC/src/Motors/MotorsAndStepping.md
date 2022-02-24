# Planning

Stepping & planning is the most important, and possibly the hardest part of the firmware to 
get right. What we strive to do here is to get things sort-of in sync. Emphasis on "sort-of", 
getting everything really in sync is just not viable.

There are basically two types of motor drivers. The first one is time-bound, variable distance
(position based) drivers. Most notable here are servo's. Given a max acceleration and max 
velocity, the servo will just go to the place where you point it in a certain time. BUT, 
the update frequency of the servo will get in the way of synchronizing movement. Servo's 
usually run at 50 Hz, so that means you have an update rate of roughly 1/50s, which is 
20'000 us. More important, if you point a servo to position X->Y at time T, it will 
get at point Y at T+20'000. In other words, if you don't take care of any time 
synchronization, the servo will lag behind by 40'000 us.

Servo's aren't the only devices that work like this. Ethercat, dynamixel, RS485 based motors, 
the list goes on and on. The update frequency for all these devices is pretty much a constant.
For an RS485 device, the update might be something like 512 kbit, with 10 bytes per packet. 
That means the update frequency is roughly 512/(8*10) = 6.4 kHz. So the lag here will be 
roughly 156 us.

Each motor here has an 'interval' value that we use to get the delay that is required to do 
updates. For step-based motors, that delay is 0. 

When a request is given to the planner, it always constructs line segments from said request. 
Arcs are converted to lines. The line buffer is a cyclic buffer of a certain size, and we just 
hope its long enough to hold twice the longest lag. Notice that the end of the planner buffer 
always boils down to "zero speed, zero acceleration", while the start of the planner buffer is 
the end of the currently executed segment (a zero speed 'fake' segment is added).

Note that if a servo driver does a request for t+dt and in the meanwhile the plan changes, 
there's not much we can do about it until the next update occurs. 

In short, absolute position devices look ahead in the planner buffer, while pulse devices split
up the lines in steps and emit pulses. The details are way more difficult, as described below.

# Planning arcs

The planner does not work on arcs, but on line segments. That's why the first thing arcs do 
is convert them to tiny lines. Basically this is just a matter of using the well-known cos/sin
formula's in a loop. 

# Planning lines

After planning, we're going to execute lines. Execution can be as easy as telling a motor to
"go there" (absolute positioning) or as hard as "I'll tell you how to go there" (stepping).
One thing to keep in mind here is that it's important that all line segments are completely 
finished before the next one is being executed. 

The stepping itself is left outside of the planner and deemed irrelevant. This is because the 
stepper loop itself just follows the acceleration profile and the stepping laid out in the plan
(with a noteworthy delay, but more on that later).

So, in any event, the planner is responsible for calculating the acceleration and deceleration
profiles. Normally, this happens in a "reverse pass" and a "forward pass" as follows:

## Reverse pass

The forward pass plans the acceleration curve from the planned pointer onward. Also scans for
optimal plan breakpoints and appropriately updates the planned pointer.

Once lines are in the planner, it calculates the acceleration profile for each line. The 
algorithm to do that should take several things that limit the speed and acceleration into 
account::

1. The maximum motor acceleration/deceleration depends on the motor speed. We use the maximum speed 
   of a block as a basis for the acceleration that is viable in the block.
2. The junction speed. Junction deviation matters here too, because it means we don't have to go 
   to a full stop for corners.

Basically what happens is this:

1. Go over every feasible block sequentially in reverse order and calculate the junction speeds
    (i.e. current->entry_speed) such that:
    a. No junction speed exceeds the pre-computed maximum junction speed limit or nominal speeds of
       neighboring blocks.
    b. A block entry speed cannot exceed one reverse-computed from its exit speed (next->entry_speed)
       with a maximum allowable deceleration over the block travel distance.
    c. The last (or newest appended) block is planned from a complete stop (an exit speed of zero).

2. Go over every block in chronological (forward) order and dial down junction speed values if
    a. The exit speed exceeds the one forward-computed from its entry speed with the maximum allowable
       acceleration over the block travel distance.

When these stages are complete, the planner will have maximized the velocity profiles throughout the all
of the planner blocks, where every block is operating at its maximum allowable acceleration limits. In
other words, for all of the blocks in the planner, the plan is optimal and no further speed improvements
are possible. If a new block is added to the buffer, the plan is recomputed according to the said
guidelines for a new optimal plan.

To increase computational efficiency of these guidelines, a set of planner block pointers have been
created to indicate stop-compute points for when the planner guidelines cannot logically make any further
changes or improvements to the plan when in normal operation and new blocks are streamed and added to the
planner buffer. For example, if a subset of sequential blocks in the planner have been planned and are
bracketed by junction velocities at their maximums (or by the first planner block as well), no new block
added to the planner buffer will alter the velocity profiles within them. So we no longer have to compute
them. Or, if a set of sequential blocks from the first block in the planner (or a optimal stop-compute
point) are all accelerating, they are all optimal and can not be altered by a new block added to the
planner buffer, as this will only further increase the plan speed to chronological blocks until a maximum
junction velocity is reached. However, if the operational conditions of the plan changes from infrequently
used feed holds or feedrate overrides, the stop-compute pointers will be reset and the entire plan is
recomputed as stated in the general guidelines.

## Forward pass

The forward pass basically limits the acceleration and speed of the blocks, thereby fixing the 
things that are just not possible. In principle, the following is done:

1. If the previous block is an acceleration block, too short to complete the full speed
   change, adjust the entry speed accordingly. Entry speeds have already been reset,
   maximized, and reverse-planned. If nominal length is set, max junction speed is
   guaranteed to be reached. No need to recheck.
2. Any block set at its maximum entry speed also creates an optimal plan up to this
   point in the buffer. When the plan is bracketed by either the beginning of the
   buffer and a maximum entry speed or two maximum entry speeds, every block in between
   cannot logically be further improved. Hence, we don't have to recompute them anymore.

## Recalculate speed profiles

After the reverse and forward pass are complete, we can recompute the speed profiles for the 
blocks. This basically means calculating the acceleration time, the acceleration rate, the 
deceleration time, the deceleration rate and the cruise speed for the block.

## Concurrency and modifications to the planner buffer

There are basically a few different modifications that can happen while we are executing 
lines in the planner buffer:

1. Lines get picked up and executed. Keep in mind that servo's plan ahead much more in 
   the future than steppers. All line segments up until the time that is needed are marked 
   as "in-use" by the executor.
2. Lines are added at the end of the buffer. Everything after the 'in use' is recomputed 
   unless there are reasons to assume they are still valid. (like: the stepper is at cruise
   speed in some line segment, which means it can't go any faster due to recomputation).
3. Pause. Basically the line is kept as-is, but the plan is adjusted to include a deceleration
   ramp. When 'resume' is executed, the plan is adjusted yet again to accelerate. 
4. Abort / alarm. This is the easiest one. The stepper timer or DMA is killed, and motor 
   drivers get a 'halt' signal. 
5. Rotary encoders. More on this later.

When the machine is idle, we add a no-op line segment at the start of the buffer. This segment
has a total time of the lag delay for all motors. So in other words, an initial delay is added.

TODO FIXME: Pause is actually quite hard to get right. We have to think this through in more
detail when the time gets there. Let's do the other things first...

## Synchronized events

Synchronized events are events that follow the motion speed. Two examples are lasers (the PWM 
frequency follows the total speed) and extrusion (the extruder follows the total speed). Said 
speed is computed during the line scheduling, and takes the lag of the device that follows the 
motion into account. Normally, synchronized events have a low update frequency, like rs485 
motors. Note that extruders have the physical delay of [stopping the] melting of plastic.

Synchronized events compute the total speed in the near future, and updates the device.

# Acceleration and deceleration computations

For acceleration and deceleration profiles, we use a quintic (fifth-degree) Bézier 
polynomial for the velocity curve, giving a "linear pop" velocity curve; with pop 
being the sixth derivative of position:

velocity - 1st, acceleration - 2nd, jerk - 3rd, snap - 4th, crackle - 5th, pop - 6th

The Bézier curve takes the form:

V(t) = P_0B_0(t) + P_1B_1(t) + P_2B_2(t) + P_3B_3(t) + P_4B_4(t) + P_5B_5(t)

Where 0 <= t <= 1, and V(t) is the velocity. P_0 through P_5 are the control points, and B_0(t)
through B_5(t) are the Bernstein basis as follows:

      B_0(t) =   (1-t)^5        =   -t^5 +  5t^4 - 10t^3 + 10t^2 -  5t   +   1
      B_1(t) =  5(1-t)^4t    =   5t^5 - 20t^4 + 30t^3 - 20t^2 +  5t
      B_2(t) = 10(1-t)^3t^2  = -10t^5 + 30t^4 - 30t^3 + 10t^2
      B_3(t) = 10(1-t)^2t^3  =  10t^5 - 20t^4 + 10t^3
      B_4(t) =  5(1-t) t^4  =  -5t^5 +  5t^4
      B_5(t) =             t^5  =    t^5
                                    ^       ^       ^       ^       ^       ^
                                    |       |       |       |       |       |
                                    A       B       C       D       E       F

Unfortunately, we cannot use forward-differencing to calculate each position through
the curve, as we use variable timer periods. So, we require a formula of the form:

      V_f(t) = A*t^5 + B*t^4 + C*t^3 + D*t^2 + E*t + F

Looking at the above B_0(t) through B_5(t) expanded forms, if we take the coefficients of t^5
through t of the Bézier form of V(t), we can determine that:

      A =    -P_0 +  5*P_1 - 10*P_2 + 10*P_3 -  5*P_4 +  P_5
      B =   5*P_0 - 20*P_1 + 30*P_2 - 20*P_3 +  5*P_4
      C = -10*P_0 + 30*P_1 - 30*P_2 + 10*P_3
      D =  10*P_0 - 20*P_1 + 10*P_2
      E = - 5*P_0 +  5*P_1
      F =     P_0

Now, since we will (currently) *always* want the initial acceleration and jerk values to be 0,
We set P_i = P_0 = P_1 = P_2 (initial velocity), and P_t = P_3 = P_4 = P_5 (target velocity),
which, after simplification, resolves to:

      A = - 6*P_i +  6*P_t =  6*(P_t - P_i)
      B =  15*P_i - 15*P_t = 15*(P_i - P_t)
      C = -10*P_i + 10*P_t = 10*(P_t - P_i)
      D = 0
      E = 0
      F = P_i

As the t is evaluated in non uniform steps here, there is no other way rather than evaluating
the Bézier curve at each point in time:

      V_f(t) = A*t^5 + B*t^4 + C*t^3 + F          [0 <= t <= 1]

Having acceleration and jerk as 0 was a bit of a trade-off here. For many small segments, a 
trapezoid curve might be used instead. Small segments *will* be used unfortunately; specifically 
when accelerating in an arc, this will become a problem. However, for simplicity, I chose not 
to take this into account and always use the bezier curves for now. 

So, in short, we need to store calculate A,B,C and F and store them in eack block, and then 
during the line scheduling, we just calculate the velocity and throw steps into the buffers.

# Line scheduling

In the case where not all motors are absolutely positioned devices, stepping occurs. We always 
assume a buffered approach for the stepping. Both GPIO and I2S stepping takes place roughly 2000 us 
before the actual event. That is: this is the high-latency stepping mode. There's also a low 
latency stepping mode, which has a buffer size of 2 (double-buffered). 

The line scheduler picks off stepping items from the queue and executes them. How these are 
picked off and what the datastructure looks like, depends on the type of port. (I2S works 
with DMA buffers, while GPIO runs through RMT). What is the same, is that we assume the 
*pulses* are being sent or *changes*. A pulse here has a fixed length, while a change just 
changes the GPIO or I2S port. 

It's also imperative to note that low latency and high latency modes both share the same timing
delay values. Synchronized stepping over multiple timing values is very hard; by using the same
timing values it all becomes much easier to manage. 

Instead of actually executing lines, the Bresenham's algorithm will be executed by the line 
scheduler, which will just throw items in a 'future' buffer. That buffer is then picked up by 
either the I2S DMA or GPIO ISR, and thrown to the device. 

# Rotary encoders

Because of all the above, the actual speed and position of the system of every moter at any given 
time is *known*. Rotary encoders translate the position and compare it with the position of the 
system at the current time. 

Compensation can only be done at control points. Control points are points where the motors and 
encoders should be in alignment - which is basically at the update frequency of the motors, 
compensated by the time lag described above. So, if we have a servo with an update frequency 
of 50 Hz, we only have control points every (1/50s = ) 20.000 us, because we don't know the 
exact acceleration profile of the servo during the transition. 

Having the exact position means we can compensate for it. Specifically, we limit the speed and 
acceleration if the rotary encoder tells us we're at a different position than what we should be.
Compensation for position is always a bit tricky, because we don't know if the encoder is wrong,
or if the motor is wrong. To compensate, we normally assume the encoder is right, up to a limit
over time. This limit basically ensures that we don't slam into a piece or endstop if our encoder
is giving us the wrong information.

Compensation is first and foremost done by downplaying the velocity and acceleration rates of 
the _other_ axis that apparently lag behind. 

Second, we can inject extra steps into the stepper engine. However, this is tricky because it 
means we have to change the rate to compensate. For now, we just don't and assume the hardware 
will catch up.
