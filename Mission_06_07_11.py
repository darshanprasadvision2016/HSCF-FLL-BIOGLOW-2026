
from hub import port, motion_sensor
import runloop
import motor_pair
import motor

PAIR = motor_pair.PAIR_1
PAIR1 = motor_pair.PAIR_2

# Drive motors
motor_pair.pair(PAIR, port.A, port.C)

# Attachment motors
motor_pair.pair(PAIR1, port.D, port.E)


# --------------------------------------------------
# FAST GYRO STRAIGHT - FOR LONG FORWARD MOVEMENTS
# --------------------------------------------------

async def gyro_straight(degrees, max_velocity=600):

    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.C, 0)

    await runloop.until(motion_sensor.stable)
    motion_sensor.reset_yaw(0)

    last_yaw = 0

    while True:

        left = abs(motor.relative_position(port.A))
        right = abs(motor.relative_position(port.C))

        travelled = (left + right) / 2
        remaining = degrees - travelled

        if remaining <= 0:
            break

        # Smooth acceleration
        if travelled < 100:
            velocity = 350

        # Smooth braking
        elif remaining < 120:
            velocity = 300

        # Full speed
        else:
            velocity = max_velocity

        # Read gyro
        yaw = motion_sensor.tilt_angles()[0] / 10

        # PD gyro correction
        proportional = yaw * 2.0
        derivative = (yaw - last_yaw) * 0.8

        correction = int(proportional + derivative)

        last_yaw = yaw

        # Prevent excessive steering
        if correction > 25:
            correction = 25

        elif correction < -25:
            correction = -25

        motor_pair.move(
            PAIR,
            correction,
            velocity=velocity
        )

        await runloop.sleep_ms(10)

    motor_pair.stop(PAIR)


# --------------------------------------------------
# GYRO MOVE - WORKS FOR FORWARD OR BACKWARD
# Example:
#gyro_move_degrees(100, 200)= forward
#gyro_move_degrees(-100, 200) = backward
# --------------------------------------------------

async def gyro_move_degrees(degrees, velocity):

    motor.reset_relative_position(port.A, 0)
    motor.reset_relative_position(port.C, 0)

    await runloop.until(motion_sensor.stable)
    motion_sensor.reset_yaw(0)

    target = abs(degrees)

    if degrees >= 0:
        direction = 1
    else:
        direction = -1

    last_yaw = 0

    while True:

        left = abs(motor.relative_position(port.A))
        right = abs(motor.relative_position(port.C))

        travelled = (left + right) / 2

        if travelled >= target:
            break

        yaw = motion_sensor.tilt_angles()[0] / 10

        proportional = yaw * 2.0
        derivative = (yaw - last_yaw) * 0.8

        correction = int(proportional + derivative)

        last_yaw = yaw

        if correction > 25:
            correction = 25

        elif correction < -25:
            correction = -25

        # Reverse steering correction when driving backward
        steering = correction * direction

        motor_pair.move(
            PAIR,
            steering,
            velocity=velocity * direction
        )

        await runloop.sleep_ms(10)

    motor_pair.stop(PAIR)


# --------------------------------------------------
# MAIN MISSION
# --------------------------------------------------

async def main():

    # Long straight forward
    await gyro_straight(1563.12, 600)

    # Attachment up
    await motor_pair.move_for_degrees(
        PAIR1, -360, 0,
        velocity=600
    )

    # Attachment down
    await motor_pair.move_for_degrees(
        PAIR1, 360, 0,
        velocity=600
    )

    # Backward 58 degrees using gyro
    await gyro_move_degrees(-59, 200)

    # Turn
    await motor_pair.move_tank_for_degrees(
        PAIR, 214,
        -200, 200
    )

    # Forward with gyro
    await gyro_straight(765, 600)

    # Turn
    await motor_pair.move_tank_for_degrees(
        PAIR, 325,
        200, -200
    )

    # Attachment
    await motor_pair.move_for_degrees(
        PAIR1, -255, 0,
        velocity=400
    )

    # Forward
    await motor_pair.move_for_degrees(
        PAIR, 360, 0,
        velocity=180
    )

    # Backward
    await motor_pair.move_for_degrees(
        PAIR, -110, 0,
        velocity=400
    )

    # Turn
    await motor_pair.move_tank_for_degrees(
        PAIR, 207,
        200, -200
    )

    # Long straight
    await gyro_straight(1510, 600)

    # Turn
    await motor_pair.move_tank_for_degrees(
        PAIR, 107,
        200, -200
    )

    # Forward
    await motor_pair.move_for_degrees(
        PAIR, 225, 0,
        velocity=400
    )

    # Attachment
    await motor_pair.move_for_degrees(
        PAIR1, 235, 0,
        velocity=400
    )

    # Final turn
    await motor_pair.move_tank_for_degrees(
        PAIR, 428,
        200, -200
    )


runloop.run(main())