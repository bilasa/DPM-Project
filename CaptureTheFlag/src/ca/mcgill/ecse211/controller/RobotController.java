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

	// OdometryCorrection
	private OdometryCorrection odoCorrection;

	// Odometer
	private Odometer odo;

	public RobotController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double WHEEL_RAD,
			double TRACK, int FORWARD_SPEED, int ROTATE_SPEED, int ACCELERATION, double TILE_SIZE) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.WHEEL_RAD = WHEEL_RAD;
		this.TRACK = TRACK;
		this.FORWARD_SPEED = FORWARD_SPEED;
		this.ROTATE_SPEED = ROTATE_SPEED;
		this.ACCELERATION = ACCELERATION;
		this.TILE_SIZE = TILE_SIZE;
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
		odoCorrection.setTargetXY(x, y);
		odoCorrection.setPaused(false);

		double lastX = odo.getXYT()[0];
		double lastY = odo.getXYT()[1];
		double theta; // Angle to next point

		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Rotate to proper angle
		if (lastX < x) {
			turnTo(90);
		} else {
			turnTo(270);
		}


		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		// Advance towards next point's x coordinate
		for (int i = 0; i < x; i++) {
			leftMotor.rotate(convertDistance(WHEEL_RAD, TILE_SIZE), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, TILE_SIZE), false);
			// *********TODO: 	CALL TO ODOMETRY CORRECTION TO MOVE FORWARD UNTIL YOU FIND A BLACK LINE,
			// *********		FIX YOURSELF, THEN UPDATE ODOMETER READING BASED ON APPROXIMATING 
			// ********* 		CURRENT READINGS ON ODOMETER AND THEN EXIT
		}
		
		
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Rotate to proper angle
		if (lastY < y) {
			turnTo(0);
		} else {
			turnTo(180);
		}

		
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		// Advance towards next point's y coordinate
		for (int i = 0; i < y; i++) {
			leftMotor.rotate(convertDistance(WHEEL_RAD, TILE_SIZE), true);
			rightMotor.rotate(convertDistance(WHEEL_RAD, TILE_SIZE), false);
			// *********TODO: 	CALL TO ODOMETRY CORRECTION TO MOVE FORWARD UNTIL YOU FIND A BLACK LINE,
			// *********		FIX YOURSELF, THEN UPDATE ODOMETER READING BASED ON APPROXIMATING 
			// ********* 		CURRENT READINGS ON ODOMETER AND THEN EXIT
		}
		
		
		
	

		// Pause the OdometryCorrection
		odoCorrection.setPaused(true);
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