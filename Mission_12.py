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

async def gyro_backward(degrees, velocity):

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

    await motor_pair.move_for_degrees(PAIR1,-10,0)
   
    await gyro_straight(731,600)

    await motor_pair.move_tank_for_degrees(PAIR,111,400,-400)

    await motor_pair.move_for_degrees(PAIR1,-62,0)

    await gyro_straight(432,600)

    await motor_pair.move_for_degrees(PAIR1,-117,0)

    await motor_pair.move_tank_for_degrees(PAIR,154,400,-400)

    await motor_pair.move_for_degrees(PAIR1,117,0)

    await gyro_backward(-317,600)

    await motor_pair.move_for_degrees(PAIR1,-350,0)

    await motor_pair.move_tank_for_degrees(PAIR,213,-400,400)

    await gyro_straight(270,600)

    await motor_pair.move_tank_for_degrees(PAIR,194,350,-350)

    await gyro_straight(265,250)

    await gyro_backward(-153,600)

    await motor_pair.move_tank_for_degrees(PAIR,76,-150,150)

    await motor_pair.move_for_degrees(PAIR1,395,0)

    await gyro_straight(105,600)

    await motor_pair.move_for_degrees(PAIR1,-100,0,velocity=600)

    await motor_pair.move_tank_for_degrees(PAIR,182,1000,-1000)


runloop.run(main())
