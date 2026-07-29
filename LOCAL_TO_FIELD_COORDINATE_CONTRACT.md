# Local-to-FIELD Coordinate Contract

This repository follows the shared Phase-01 coordinate contract in the sibling
drone and ground-station repositories. It deliberately adds no runtime
behavior.

The car reports only its startup-local pose: rear-axle centre at startup is
`(0, 0)`, `+X` is the initial vehicle heading, `+Y` is vehicle-left, and
heading is top-down counter-clockwise positive. All positions and velocities
are centimetres and centimetres per second. Its FleetBus state source is
`navigation.pose`, never radar-native `Pose2D.yaw_cw_deg`.

The ground station alone applies the fixed SE(2) local-to-FIELD transform. The
car must not pre-apply a FIELD origin/heading, rotate its map, or rebase
Navigation due to `SET_COORDINATE_FRAME`. FleetBus mode must therefore not
call `_on_coordinate_frame_command()` or an equivalent map-rebase route. A
process has one HC-14 business-protocol owner.

The car hardware and localization protections remain outside this work:
`radar_driver.py` ICP/wall fusion/mount semantics, `navigation.py` rear-axle
origin/planning/control/safe stops, and rear-motor/steering/Ackermann protocols
and calibration are unchanged.

The current branch contains FleetBus modules and a coordinate-frame callback.
Their existence is not conformance with this contract. The later car phases
must adapt the current implementation without changing its protected radar or
vehicle-control behavior.

The full cross-repository transform, FIELD definition, inverse command rule,
and display-offset separation are recorded in the matching
`LOCAL_TO_FIELD_COORDINATE_CONTRACT.md` documents in the drone and ground
station repositories. No site-specific origin, H point, or initial heading is
guessed here.
