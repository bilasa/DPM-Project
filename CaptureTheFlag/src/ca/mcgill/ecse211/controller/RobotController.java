package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.main.CaptureTheFlag;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;

/**
 * This class controls the low-level operations of the robot's motors.
 * These operations include turning the robot, moving it forward,
 * stopping it, and traveling to a waypoint. The RobotController is used
 * by any class that needs to do any of the operations described above.
 * This class is used by all middle layer (navigation) classes, since
 * they all involve the movement of the robot.
 * 
 * @author Bijan Sadeghi
 *
 */
public class RobotController {

	// Motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	// Constants
	public final double WHEEL_RAD;
	public final double TRACK;
	public final int FORWARD_SPEED; // made public due to frequent use
	public final int ROTATE_SPEED; // made public due to frequent use
	public final int ACCELERATION; // made public due to frequent use
	public final double TILE_SIZE;
	public final double SENSOR_DIST;

	// OdometryCorrection
	private OdometryCorrection odoCorrection;

	// Odometer
	private Odometer odo;

	/**
	 * @param leftMotor the left motor
	 * @param rightMotor the right motor
	 * @param WHEEL_RAD the radius of the wheel
	 * @param TRACK the track of the robot
	 * @param FORWARD_SPEED the speed to travel forward at
	 * @param ROTATE_SPEED the speed to rotate at
	 * @param ACCELERATION the acceleration to use
	 * @param TILE_SIZE the size of a tile
	 * @param SENSOR_DIST the vertical offset of the rear sensors from the robot's center
	 */
	public RobotController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double WHEEL_RAD,
			double TRACK, int FORWARD_SPEED, int ROTATE_SPEED, int ACCELERATION, double TILE_SIZE, double SENSOR_DIST) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.WHEEL_RAD = WHEEL_RAD;
		this.TRACK = TRACK;
		this.FORWARD_SPEED = FORWARD_SPEED;
		this.ROTATE_SPEED = ROTATE_SPEED;
		this.ACCELERATION = ACCELERATION;
		this.TILE_SIZE = TILE_SIZE;
		this.SENSOR_DIST = SENSOR_DIST;
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.setAcceleration(this.ACCELERATION);
	}

	/**
	 * @param radius the radius of the wheel
	 * @param distance the desired distance to move the robot
	 * @return the angle the robot needs to rotate its wheels by in order to travel forward by
	 *         distance
	 */
	public int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * @param radius the radius of the wheel
	 * @param width the track of the robot
	 * @param angle the angle desired to turn the robot by
	 * @return the angle the robot needs to turn each wheel to rotate by angle.
	 */
	public int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * Travels to (x, y) with speed "speed" and an optional lock for the motors.
	 * 
	 * @param x the x destination of the robot
	 * @param y the y destination of the robot
	 * @param speed the speed to move the robot at
	 * @param lock an optional lock on the motors
	 */
	public void directTravelTo(double x, double y, int speed, boolean lock) {

		double lastX = odo.getXYT()[0];
		double lastY = odo.getXYT()[1];
		double theta; // Angle to next point

		// Distance to travel in x and y
		double xDist = TILE_SIZE * x - lastX;
		double yDist = TILE_SIZE * y - lastY;

		// Absolute angle to next point
		theta = Math.toDegrees(Math.atan2(xDist, yDist));

		// Distance the robot has to travel
		double distance = Math.sqrt(xDist * xDist + yDist * yDist);

		// Rotate robot towards next point
		turnTo(theta);

		// Advance towards next point
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);
		leftMotor.rotate(convertDistance(WHEEL_RAD, distance), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, distance), !lock);
	}

	/**
	 * Travels to (x, y) strictly on the x-axis and y-axis using odometry correction. 
	 * If one of x or y is the same as the robot's current x or y, then the robot 
	 * will travel directly either horizontally or vertically. If both x and y differ 
	 * from the robot's, then the robot will travel in an L-shape, starting on the 
	 * x-axis first.
	 * 
	 * @param x the x destination of the robot
	 * @param y the y destination of the robot
	 * @param speed max speed to move the robot at
	 * @param lock an optional lock on the motors
	 */
	public void travelTo(int x, int y, int speed, boolean lock) {
		// Unpause the OdometryCorrection, set the target destination
		// odoCorrection.setTargetXY(x, y);
		// odoCorrection.setPaused(false);

		// Compute the nearest waypoint from the odometer reading
		int lastX = (int) Math.round(odo.getXYT()[0] / TILE_SIZE);
		int lastY = (int) Math.round(odo.getXYT()[1] / TILE_SIZE);

		boolean negX = false;
		boolean negY = false;

		// Check if the target point is below the current X and current Y
		if (lastX > x) {
			negX = true;
		}
		if (lastY > y) {
			negY = true;
		}
		// Angle to turn to to go to the next point. In OdometryCorrection, use this
		// angle to correct the Odometer's theta.
		double corrTheta = odo.getXYT()[2];

		setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		// Find the proper angle to rotate to (if lastX == x, not needed)
		if (negX) {
			corrTheta = 270;
		} else {
			corrTheta = 90;
		}

		// Rotate to the proper angle
		if (lastX != x)
			turnTo(corrTheta);

		// Calculate the number of tiles the robot needs to move in the X direction
		int tilesX = x - lastX;

		setSpeeds(speed, speed);

/*
		// Advance towards next point's x coordinate
		for (int i = 1; i <= Math.abs(tilesX); i++) {

			// Immediate correction for the first tile moved
			if (i == 1) {
				odoCorrection.correct(corrTheta, odo.getXYT());
			}

			double[] initialOdo = odo.getXYT();

			// travelToDirect() to the next closest point
			directTravelTo(lastX + (tilesX / Math.abs(tilesX)) * i, lastY, FORWARD_SPEED, lock);
			// leftMotor.rotate(convertDistance(WHEEL_RAD, 2.0 / 3.0 * TILE_SIZE), true);
			// rightMotor.rotate(convertDistance(WHEEL_RAD, 2.0 / 3.0 * TILE_SIZE), false);

			// Correct the robot in the X-direction with correct theta corrTheta
			odoCorrection.correct(corrTheta, initialOdo);

			// Move back by sensor offset at the last tile
			if (i == Math.abs(tilesX)) {
				this.travelDist(-SENSOR_DIST, true);
			}

		}
*/
		// Advance towards next point's x coordinate
		moveForward();
		while(Math.abs(odo.getXYT()[0] - x * TILE_SIZE) > 2) {
			moveForward();
			double currX = odo.getXYT()[0];
			// Proportionality constant between 0 and 0.5
			double propCnst = Math.abs((currX / TILE_SIZE) - Math.round(currX / TILE_SIZE));
			// Correct if close enough to a line
			if(propCnst <= 0.008) {
				odoCorrection.correct(corrTheta, odo.getXYT());
			}
			int propSpeed = (int) (150 + propCnst * 900);	// Speed between 150 and 600
			setSpeeds(propSpeed, propSpeed);
			Delay.msDelay(50);
			
		}
		setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		travelDist(-SENSOR_DIST, lock);

		setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		// Find the proper angle to rotate to (if lastX == x, not needed)
		if (negY) {
			corrTheta = 180;
		} else {
			corrTheta = 0;
		}

		// Rotate to the proper angle
		if (lastY != y)
			turnTo(corrTheta);

		// Calculate the number of tiles the robot needs to move in the Y direction
		int tilesY = y - lastY;

		setSpeeds(speed, speed);

/*
		// Advance towards next point's y coordinate
		for (int i = 1; i <= Math.abs(tilesY); i++) {

			// Immediate correction for the first tile moved
			if (i == 1) {
				odoCorrection.correct(corrTheta, odo.getXYT());
			}

			double[] initialOdo = odo.getXYT();

			// travelToDirect() to the next closest point
			directTravelTo(lastX, lastY + (tilesY / Math.abs(tilesY)) * i, FORWARD_SPEED, lock);

			// Correct the robot in the Y-direction with correct theta corrTheta
			odoCorrection.correct(corrTheta, initialOdo);

			// Move back by sensor offset at the last tile
			if (i == Math.abs(tilesY)) {
				this.travelDist(-SENSOR_DIST, true);
			}
		}
*/
		// Advance towards next point's y coordinate
		travelDist(tilesY * TILE_SIZE, false);
		while(Math.abs(odo.getXYT()[1] - y * TILE_SIZE) > SENSOR_DIST) {
			// Proportionality constant between 0 and 0.5
			double currY = odo.getXYT()[1];
			double propCnst = Math.abs((currY / TILE_SIZE) - Math.round(currY / TILE_SIZE));
			if(propCnst <= 0.008) {
				odoCorrection.correct(corrTheta, odo.getXYT());
			}
			int propSpeed = (int) (150 + propCnst * 900);	// Speed between 150 and 600
			setSpeeds(propSpeed, propSpeed);
			moveForward();
			Delay.msDelay(50);
			
		}
		setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		travelDist(-SENSOR_DIST, lock);
		
		
		// Pause the OdometryCorrection
		// odoCorrection.setPaused(true);
	}

	/**
	 * Turns the robot to the specified absolute angle.
	 * 
	 * @param absTheta the absolute angle to turn the robot to
	 */
	public void turnTo(double absTheta) {

		// Get current theta
		double currTheta = odo.getXYT()[2];

		// Set angle displacement
		double dTheta = absTheta - currTheta;

		// Set speed to turn speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Calculate smallest angle
		if (dTheta > 180) {
			dTheta -= 360;
		} else if (dTheta < -180) {
			dTheta += 360;
		}
		if (dTheta == 180 || dTheta == -180) {
			dTheta = 180;
		}

		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), false);
	}

	/**
	 * Turns the robot by the specified angle with an optional lock.
	 * 
	 * @param dTheta the angle to turn the robot by
	 */
	public void turnBy(double dTheta, boolean lock) {
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), !lock);
	}

	/**
	 * Sets the speeds of the motors.
	 * 
	 * @param leftSpeed the speed to set the left motor to
	 * @param rightSpeed the speed to set the right motor to
	 */
	public void setSpeeds(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}

	/**
	 * Set the acceleration of the motors.
	 * 
	 * @param acceleration the acceleration to set the left and right motor to
	 */
	public void setAcceleration(int acceleration) {
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
	}

	/**
	 * Moves the robot forward by the specified distance with an optional lock
	 * 
	 * @param dist the desired distance in cm's to move the robot by
	 */
	public void travelDist(double dist, boolean lock) {
		leftMotor.rotate(convertDistance(WHEEL_RAD, dist), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, dist), !lock);
	}

	/**
	 * Starts moving the robot forward with motor synchronization.
	 */
	public void moveForward() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}

	/**
	 * Starts moving the robot backward with motor synchronization.
	 */
	public void moveBackward() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.endSynchronization();
	}

	/**
	 * Starts rotating the robot clockwise or counterclockwise.
	 * 
	 * @param rotateClockwise whether the robot should rotate clockwise
	 * @param speed the speed to set the motors to during the rotation
	 */
	public void rotate(boolean rotateClockwise, int speed) {
		setSpeeds(speed, speed);
		if (rotateClockwise) {
			leftMotor.forward();
			rightMotor.backward();
		} else {
			leftMotor.backward();
			rightMotor.forward();
		}
	}

	/**
	 * Specifies whether the robot is current moving.
	 * 
	 * @return true if the left or right motor is moving
	 */
	public boolean isMoving() {
		if (leftMotor.isMoving() || rightMotor.isMoving())
			return true;
		return false;
	}

	/**
	 * Stops the robot with motor synchronization.
	 */
	public void stopMoving() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.endSynchronization();
	}

	/**
	 * Stops the specified motors of the robot.
	 * 
	 * @param stopLeft whether to stop the left motor
	 * @param stopRight whether to stop the right motor
	 */
	public void stopMoving(boolean stopLeft, boolean stopRight) {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		if (stopLeft)
			leftMotor.stop();
		if (stopRight)
			rightMotor.stop();
		leftMotor.endSynchronization();
	}

	/**
	 * Sets the OdometryCorrection object to be used by the robot controller.
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorrection = odoCorrection;
	}

}