# road_reconstruction

Module that performs the road reconstruction and filters the resulting point cloud.

Uses the laser scan point cloud of 4 planes, provided by the 'laser_assembler' service, assembles them into a single cloud and processes it to remove the points refering to the road.
