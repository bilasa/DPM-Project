package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;

/**
 * This class controls the low-level operations of the robot's motors.
 * These operations include turning the robot, moving it forward,
 * stopping it, and traveling to a waypoint. The RobotController is used
 * by any class that needs to do any of the operations described above.
 * This class is used by all middle layer (navigation) classes, since
 * they all involve the movement of the robot.
 * 
 * @author Bijan Sadeghi
 * @author Guillaume Richard
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
	public final int ROTATE_SPEED; 	// made public due to frequent use
	public final int ACCELERATION; 	// made public due to frequent use
	public final double TILE_SIZE;
	public final double REAR_SENSOR_DIST;

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
	 * @param REAR_SENSOR_DIST the vertical offset of the rear sensors from the robot's center
	 */
	public RobotController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double WHEEL_RAD,
			double TRACK, int FORWARD_SPEED, int ROTATE_SPEED, int ACCELERATION, double TILE_SIZE, double REAR_SENSOR_DIST) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.WHEEL_RAD = WHEEL_RAD;
		this.TRACK = TRACK;
		this.FORWARD_SPEED = FORWARD_SPEED;
		this.ROTATE_SPEED = ROTATE_SPEED;
		this.ACCELERATION = ACCELERATION;
		this.TILE_SIZE = TILE_SIZE;
		this.REAR_SENSOR_DIST = REAR_SENSOR_DIST;
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
	 * Travels to (x, y) with speed "speed" and an optional lock for the motors. Correction
	 * is not enabled
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
	 * Travels to (x, y) stricly on the x-axis and y-axis using odometry correction. 
	 * If one of x or y is the same as the robot's current x or y, then the robot 
	 * will travel directly either horizontally or vertically. If both x and y differ 
	 * from the robot's, then the robot will travel in an L-shape, starting on the 
	 * x-axis first.
	 * 
	 * @param x the x destination of the robot
	 * @param y the y destination of the robot
	 * @param speed the speed to move the robot at
	 */
	public void travelTo(int x, int y, int speed) {

		// Compute the nearest waypoint from the odometer reading
		int lastX = (int) Math.round(odo.getXYT()[0] / TILE_SIZE);
		int lastY = (int) Math.round(odo.getXYT()[1] / TILE_SIZE);

		// Angle to turn to to go to the next point. In OdometryCorrection, use this
		// angle to correct the Odometer's theta.
		double corrTheta = odo.getXYT()[2];

		// Check if the target point is below the current X and current Y
		if (lastX > x) {
			corrTheta = 270;
		} else if (lastX < x) {
			corrTheta = 90;
		}

		setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		// If we need to travel in the X direction
		if (lastX != x) {
			turnTo(corrTheta);

			setSpeeds(speed, speed);

			// Advance towards next point's x coordinate
			while(Math.abs(odo.getXYT()[0] - x * TILE_SIZE) > 10) {
				// Move forward
				moveForward();

				// Correct if not searching
				double currX = odo.getXYT()[0];
				
				// Proportionality constant between 0 and 0.5
				double propCnst = Math.abs((currX / TILE_SIZE) - Math.round(currX / TILE_SIZE));
				
				// Correct if close enough to a line
				if(propCnst <= 0.1) {
					odoCorrection.correct();
				}
				
				setSpeeds(speed, speed);
				moveForward();
			}
			odoCorrection.correct();
			setSpeeds(speed, speed);
			travelDist(-REAR_SENSOR_DIST, true);
		}

		// Find the proper angle to rotate to (if lastY == y, not needed)
		if (lastY > y) {
			corrTheta = 180;
		} else if (lastY < y) {
			corrTheta = 0;
		}

		setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

		// If we need to travel in the X direction
		if (lastY != y) {
			turnTo(corrTheta);

			setSpeeds(speed, speed);

			// Advance towards next point's y coordinate
			while(Math.abs(odo.getXYT()[1] - y * TILE_SIZE) > 10) {
				// Move forward
				moveForward();

				// Correct if not searching
				double currY = odo.getXYT()[1];
				
				// Proportionality constant between 0 and 0.5
				double propCnst = Math.abs((currY / TILE_SIZE) - Math.round(currY / TILE_SIZE));
				
				// Correct if close enough to a line
				if(propCnst <= 0.1) {
					odoCorrection.correct();
				}
				
				setSpeeds(speed, speed);
				moveForward();
			}
			odoCorrection.correct();
			setSpeeds(speed, speed);
			travelDist(-REAR_SENSOR_DIST, true);
		}

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
		// Set speed to turn speed
		setSpeeds(ROTATE_SPEED, ROTATE_SPEED);

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
	 * Specifies whether the robot is currently moving.
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
	 * Calculates the Euclidean Distance between two points given by (x1, y1) and 
	 * (x2,y2).
	 * 
	 * @param x1 X coordinate of first point
	 * @param y1 Y coordinate of first point
	 * @param x2 X coordinate of second point
	 * @param y2 Y coordinate of second point
	 * @return the distance between the two points
	 */
	public double euclideanDistance(double x1, double y1, double x2, double y2) {
		return Math.sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
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