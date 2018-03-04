package ca.mcgill.ecse211.controller;

import ca.mcgill.ecse211.main.CaptureTheFlag;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;

/**
 * Controls the low-level operations of the robot's motors
 * 
 * @author bijansadeghi
 *
 */
public class RobotController {
	
	// Motors
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	// Constants
	private final double WHEEL_RAD;
	private final double TRACK;
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final double TILE_SIZE;
	
	//Odometer
	private Odometer odo;
	
	public RobotController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double WHEEL_RAD, double TRACK, int FORWARD_SPEED, int ROTATE_SPEED, double TILE_SIZE) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.WHEEL_RAD = WHEEL_RAD;
		this.TRACK = TRACK;
		this.FORWARD_SPEED = FORWARD_SPEED;
		this.ROTATE_SPEED = ROTATE_SPEED;
		this.TILE_SIZE = TILE_SIZE;
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

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
	public void travelTo(double x, double y, int speed, boolean lock) {

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
	public void turnBy(double dTheta, boolean lock){
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
	public void moveForward(){
		leftMotor.synchronizeWith(new RegulatedMotor[]{rightMotor});
		leftMotor.startSynchronization();
		leftMotor.forward();
		rightMotor.forward();
		leftMotor.endSynchronization();
	}
	
	
	/**
	 * Starts rotating the robot clockwise or counterclockwise
	 * 
	 * @param rotateClockwise
	 * @param speed
	 */
	public void rotate(boolean rotateClockwise, int speed){
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
	public boolean isMoving(){
		if(leftMotor.isMoving() || rightMotor.isMoving())
			return true;
		return false;
	}
	
	/**
	 * Stops the robot with motor synchronization
	 */
	public void stopMoving(){
		leftMotor.synchronizeWith(new RegulatedMotor[]{rightMotor});
		leftMotor.startSynchronization();
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.endSynchronization();
	}

}