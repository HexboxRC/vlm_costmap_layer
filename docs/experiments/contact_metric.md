# Gazebo contact collision metric — October 13

Collision now means any physical contact between any robot collision link
and glass or one of the four boundary walls, from goal submission through
navigation result/cancellation. Ground contacts are excluded by exact model
name. Geometry-overlap inference remains only as the separate `pose_overlap`
diagnostic; it no longer decides collision or success.

Six contact sensors are attached to the five obstacle collision geometries
and the ground collision, with Gazebo Harmonic's Contact system. The obstacle
topic is bridged to a reliable ROS subscription (depth 1000); the ground
heartbeat is a separate best-effort subscription. Ground contact must be
observed before navigation, and stream gaps over two simulated seconds make
the attempt an infrastructure failure with blank navigation-result/success.
The existing stalled-simulation wall watchdog remains unchanged.

Gazebo's Contact system uses `<contact><topic>`, not the generic sensor
`<topic>`. Source inspection resolved the initial no-publisher failure.
It publishes at physics updates while contacts exist; the sensor's nominal
50 Hz update-rate is not assumed to throttle this implementation. Preserve
1 ms physics and check controller-gap exclusion under the new instrumentation.

Verification completed within the user's two-hour limit (about 13 minutes
of monotonic elapsed work including the failed probes). The final probe
held stationary on free ground for two simulated seconds, then drove at
.25 m/s into glass. It observed 14,187 ground contact records without a false
obstacle collision, then one actual glass/chassis collision and clean teardown.
A prior corrected probe also passed. Twelve regression tests passed, including
exact pair classification, missing-stream rejection and grass exposure.

[Diagnostic evidence](evidence/oct13_contacts.tar.gz). The first failed
probe's launch log was accidentally overwritten when an approval timeout
prevented the unique-output-path edit from executing; its zero-message
summary and this limitation are retained. Subsequent probe paths are unique.
Neither failed diagnostic is a pilot attempt. A second no-publisher failure
is retained in full. No claim that the initial DDS errors alone caused failure.

Raw obstacle pairs, positions, penetration depths, simulation/source stamps
and navigation-window flags are saved in each trial's `_contacts.json`, along
with contact counters, maximum callback gap and stream-health result.
Physical event time comes from the source stamp; callback time is separate.
This measures simulated contacts, not physical hardware collision accuracy.

Implementation reference:
https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/contact/Contact.cc
