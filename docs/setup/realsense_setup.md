# RealSense D435 setup

Install the target Pi dependencies, connect D435 to USB 3, confirm with
`rs-enumerate-devices`, and verify the serial in the edge profile. Check optical
frames, calibration, timestamps, resolution, rate, USB bandwidth, and temperature.

Start camera/VO in edge safe-idle with control STOP. D435 obstacle-cloud and
voxel integration remain disabled by default. First validate filtering against
floor, chassis, people, and fixed obstacles; only an explicit signed-off profile
may later enable the voxel layer.
