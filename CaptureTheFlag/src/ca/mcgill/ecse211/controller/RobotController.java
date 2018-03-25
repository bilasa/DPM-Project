package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.main.CaptureTheFlag;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;

/**
 * Controls the low-level operations of the robot's motors
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
	private final double SENSOR_DIST;

	// OdometryCorrection
	private OdometryCorrection odoCorrection;

	// Odometer
	private Odometer odo;

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
	 * @param radius
	 * @param distance
	 * @return the angle the robot needs to rotate its wheels to travel forward by
	 *         distance
	 */
	public int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * @param radius
	 * @param width
	 * @param angle
	 * @return the angle the robot needs to turn each wheel to rotate by angle
	 */
	public int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * Travels to (x, y) with speed "speed" and an optional lock for the motors.
	 * 
	 * @param x
	 * @param y
	 * @param speed
	 * @param lock
	 */
	public void travelTo(int x, int y, int speed, boolean lock) {
		// Unpause the OdometryCorrection, set the target destination
		//odoCorrection.setTargetXY(x, y);
		//odoCorrection.setPaused(false);

		// Compute the nearest waypoint from the odometer reading
		int lastX = (int)Math.round(odo.getXYT()[0] / TILE_SIZE);
		int lastY = (int)Math.round(odo.getXYT()[1] / TILE_SIZE);

		// Angle to turn to to go to the next point. In OdometryCorrection, use this angle to correct the Odometer's theta.
		double corrTheta = odo.getXYT()[2]; 

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Find the proper angle to rotate to (if lastX == x, not needed)
		if (lastX < x) {
			corrTheta = 90;
		} else if (lastX > x) {
			corrTheta = 270;
		}

		// Rotate to the proper angle
		turnTo(corrTheta);

		// Calculate the number of tiles the robot needs to move in the X direction
		int tilesX = Math.abs(x - lastX);

		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		// Advance towards next point's x coordinate
		for (int i = 0; i < tilesX; i++) {

			// Immediate correction for the first tile moved
			if (i == 0) {
				odoCorrection.correct(true, corrTheta);
			}

			// Move forward by 2/3 of a tile so that the sensors are behind the perpendicular line we will correct at
			leftMotor.rotate(convertDistance(WHEEL_RAD, 2.0 / 3.0 * TILE_SIZE), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, 2.0 / 3.0 * TILE_SIZE), false);

			// *********TODO: 	CALL TO ODOMETRY CORRECTION TO MOVE FORWARD UNTIL YOU FIND A BLACK LINE,
			// *********		FIX YOURSELF, THEN UPDATE ODOMETER READING BASED ON APPROXIMATING 
			// ********* 		CURRENT READINGS ON ODOMETER AND THEN EXIT

			// Correct the robot in the X-direction with correct theta corrTheta
			odoCorrection.correct(true, corrTheta);

			// Move back by sensor offset at the last tile
			if (i == (tilesX - 1)) {
				this.travelDist(-SENSOR_DIST, true);
			}

		}



		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Find the proper angle to rotate to (if lastX == x, not needed)
		if (lastX < x) {
			corrTheta = 0;
		} else if (lastX > x) {
			corrTheta = 180;
		}

		// Rotate to the proper angle
		turnTo(corrTheta);

		// Calculate the number of tiles the robot needs to move in the Y direction
		int tilesY = Math.abs(y - lastY);

		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		// Advance towards next point's y coordinate
		for (int i = 0; i < tilesY; i++) {
			
			// Immediate correction for the first tile moved
			if (i == 0) {
				odoCorrection.correct(false, corrTheta);
			}

			leftMotor.rotate(convertDistance(WHEEL_RAD, 2.0 / 3.0 * TILE_SIZE), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, 2.0 / 3.0 * TILE_SIZE), false);
			// *********TODO: 	CALL TO ODOMETRY CORRECTION TO MOVE FORWARD UNTIL YOU FIND A BLACK LINE,
			// *********		FIX YOURSELF, THEN UPDATE ODOMETER READING BASED ON APPROXIMATING 
			// ********* 		CURRENT READINGS ON ODOMETER AND THEN EXIT

			// Correct the robot in the Y-direction with correct theta corrTheta
			odoCorrection.correct(false, corrTheta);

			// Move back by sensor offset at the last tile
			if (i == (tilesY - 1)) {
				this.travelDist(-SENSOR_DIST, true);
			}
		}





		// Pause the OdometryCorrection
		//odoCorrection.setPaused(true);
	}

	/**
	 * Turns the robot to the absolute angle "absTheta"
	 * 
	 * @param absTheta
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
	 * Turns the robot by an angle "dTheta" with an optional lock
	 * 
	 * @param dTheta
	 */
	public void turnBy(double dTheta, boolean lock) {
		leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, dTheta), !lock);
	}

	/**
	 * Sets the speeds of the motors
	 * 
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setSpeeds(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
	}

	/**
	 * Set the acceleration of the motors
	 * 
	 * @param acceleration
	 */
	public void setAcceleration(int acceleration) {
		leftMotor.setAcceleration(acceleration);
		rightMotor.setAcceleration(acceleration);
	}

	/**
	 * Moves the robot forward by distance "dist" with an optional lock
	 * 
	 * @param dist
	 */
	public void travelDist(double dist, boolean lock) {
		leftMotor.rotate(convertDistance(WHEEL_RAD, dist), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, dist), !lock);
	}

	/**
	 * Starts moving the robot forward with motor synchronization
	 */
	public void moveForward() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}

	/**
	 * Starts moving the robot backward with motor synchronization
	 */
	public void moveBackward() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.backward();
		rightMotor.backward();
		leftMotor.endSynchronization();
	}

	/**
	 * Starts rotating the robot clockwise or counterclockwise
	 * 
	 * @param rotateClockwise
	 * @param speed
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
	 * @return true if either of the robot's motors are moving
	 */
	public boolean isMoving() {
		if (leftMotor.isMoving() || rightMotor.isMoving())
			return true;
		return false;
	}

	/**
	 * Stops the robot with motor synchronization
	 */
	public void stopMoving() {
		leftMotor.synchronizeWith(new RegulatedMotor[] { rightMotor });
		leftMotor.startSynchronization();
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.endSynchronization();
	}

	/**
	 * Stop the robot, choosing which motors to stop
	 * 
	 * @param stopLeft
	 * @param stopRight
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
	 * Set the OdometryCorrection to be used by the robot controller
	 * 
	 * @param odoCorrection
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorrection = odoCorrection;
	}

}