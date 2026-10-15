"""Conversion of Water Linked raw IMU batches to ROS messages."""

from sensor_msgs.msg import Imu


def imu_batch_to_messages(batch, frame_id):
    """Convert one sonar FRD ImuBatch to timestamped ROS FLU Imu messages."""
    samples = int(batch.samples)
    if (
        len(batch.timestamp) != samples
        or len(batch.specific_force) != samples * 3
        or len(batch.rate_of_turn) != samples * 3
    ):
        raise ValueError(
            'Malformed ImuBatch: expected '
            f'{samples} timestamps and {samples * 3} vector values, got '
            f'{len(batch.timestamp)}, {len(batch.specific_force)}, '
            f'{len(batch.rate_of_turn)}'
        )

    messages = []
    for sample_index in range(samples):
        vector_index = sample_index * 3
        timestamp = batch.timestamp[sample_index]

        msg = Imu()
        msg.header.stamp.sec = timestamp.seconds
        msg.header.stamp.nanosec = timestamp.nanos
        msg.header.frame_id = frame_id

        # ImuBatch contains raw specific force and rate of turn, but no
        # orientation estimate. ROS uses -1 in the first covariance element
        # to explicitly mark orientation as unavailable.
        msg.orientation_covariance[0] = -1.0

        # The sonar reports vectors in FRD. Convert them to ROS FLU by
        # preserving x and negating y and z.
        msg.linear_acceleration.x = batch.specific_force[vector_index]
        msg.linear_acceleration.y = -batch.specific_force[vector_index + 1]
        msg.linear_acceleration.z = -batch.specific_force[vector_index + 2]
        msg.angular_velocity.x = batch.rate_of_turn[vector_index]
        msg.angular_velocity.y = -batch.rate_of_turn[vector_index + 1]
        msg.angular_velocity.z = -batch.rate_of_turn[vector_index + 2]
        messages.append(msg)

    return messages
