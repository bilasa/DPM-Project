/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

/**
 * This class allows the robot to correct itself by using
 * the two rear sensors. When either sensor detects a perpendicular
 * line on the path of the robot, that side's motor stops until
 * the other sensor also detects the line, effectively straightening
 * the robot.
 * 
 * @author Bijan Sadeghi
 * @author Esa Khan
 *
 */
public class OdometryCorrection {

	// Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final double TILE_SIZE;
	private final double SENSOR_DIST;

	// Robot controller
	private RobotController rc;

	// Left and right rear sensor controllers
	private LightSensorController leftLsCont;
	private LightSensorController rightLsCont;

	// Odometer
	private Odometer odo;

	// Paused variable to tell pause/unpause the OdometryCorrection
	private boolean paused;
	
	// Boolean to check if we are currently identifying a block
	private boolean identifyingBlock = false;

	// Enumeration to denote the state of the OdometryCorrection
	private enum CorrState {
		EXPECTING_LINE, CORRECTING
	}

	private CorrState corrState;

	/**
	 * @param TILE_SIZE the size of a tile
	 * @param SENSOR_DIST the vertical offset of the rear sensors from the robot's center
	 * @param rc the robot controller to use
	 * @param leftLsCont the light sensor controller to use to get data from the left rear sensor
	 * @param rightLsCont the light sensor controller to use to get data from the right rear sensor
	 */
	public OdometryCorrection(double TILE_SIZE, double SENSOR_DIST, RobotController rc,
			LightSensorController leftLsCont, LightSensorController rightLsCont) {
		this.FORWARD_SPEED = rc.FORWARD_SPEED;
		this.ROTATE_SPEED = rc.ROTATE_SPEED;
		this.TILE_SIZE = TILE_SIZE;
		this.SENSOR_DIST = SENSOR_DIST;
		this.rc = rc;
		this.leftLsCont = leftLsCont;
		this.rightLsCont = rightLsCont;
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.paused = true;
		this.corrState = CorrState.EXPECTING_LINE;
	}

	/**
	 * Moves the robot forward until a perpendicular line is detected by one of the
	 * sensors. Re-aligns the robot so that both sensors are on the line. Corrects
	 * the Odometer's theta and either the x or the y, based on which axis the robot
	 * is traveling on.
	 *
	 * @param corrTheta the angle the Odometer must be set to after correction
	 * @param initialOdo the Odometer values from the end of the last correction
	 */
	public void correct(double corrTheta, double[] initialOdo) {
		// Intermediate odometer reading: odometer's reading after detecting the first line
		//double[] intermediateOdo = new double[3];

		// Sleep for 250 ms
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Booleans for whether the line has been detected by the right sensor or left
		// sensor
		boolean rightLineDetected = false;
		boolean leftLineDetected = false;

		// Start moving the robot forward
		rc.setSpeeds(150, 150);
		rc.moveForward();

		// Move the robot until one of the sensors detects a line
		while (!leftLineDetected && !rightLineDetected && !identifyingBlock) {
			if (rightLsCont.getColorSample()[0] == 13.0) {
				rightLineDetected = true;
				// Stop the right motor
				rc.stopMoving(false, true);
				//intermediateOdo = odo.getXYT();

			} else if (leftLsCont.getColorSample()[0] == 13.0) {
				leftLineDetected = true;

				// Stop the left motor
				rc.stopMoving(true, false);
				//intermediateOdo = odo.getXYT();
			}
		}

		// Get the odometer's reading 

		// Keep moving the left/right motor until both lines have been detected
		while ((!leftLineDetected || !rightLineDetected) && !identifyingBlock) {
			// If the other line detected, stop the motors
			if (rightLineDetected && leftLsCont.getColorSample()[0] == 13.0) {
				leftLineDetected = true;
				rc.stopMoving();
			} else if (leftLineDetected && rightLsCont.getColorSample()[0] == 13.0) {
				rightLineDetected = true;
				rc.stopMoving();
			}
		}

		// The robot is now aligned. Correct the odometer.

		// Get the final odometer reading: reading after both lines are detected
		/*double[] finalOdo = odo.getXYT();

		// Compute the angle the robot rotated by to correct itself
		double dTheta = finalOdo[2] - intermediateOdo[2];

		// Compute the distance traveled from initial to intermediate
		double dist = Math.hypot(intermediateOdo[0] - initialOdo[0], intermediateOdo[1] - initialOdo[1]);

		// Compute the offset of the sensor's from the line
		double c = rc.TRACK / 2 * Math.tan(Math.toRadians(Math.abs(dTheta)));

		// Compute the robot's offset from its intended traveling axis
		double offset = (dist - SENSOR_DIST + c) * Math.sin(Math.abs(dTheta));*/

		// Variables to store the final correct odometer values
		double corrX = 0;
		double corrY = 0;

		// Check if we are correcting along X or Y axis
		if (corrTheta == 90 || corrTheta == 270) {

			// Compute the robot's X-coordinate in cm's by adding/subtracting sensor offset
			// to the sensor's X-coordinate (in cm's)
			if (corrTheta == 90) {
				// Compute the sensors' X position in cm's
				double sensorX = odo.getXYT()[0] - SENSOR_DIST;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrSensorX = (int) Math.round(sensorX / TILE_SIZE);

				// Get the correct X
				corrX = TILE_SIZE * corrSensorX + SENSOR_DIST;

				// Get the correct Y
				//corrY = intermediateOdo[1] + (dTheta / Math.abs(dTheta) * offset);


			} else {
				// Compute the sensors' X position in cm's
				double sensorX = odo.getXYT()[0] + SENSOR_DIST;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrSensorX = (int) Math.round(sensorX / TILE_SIZE);

				// Get the correct X
				corrX = TILE_SIZE * corrSensorX - SENSOR_DIST;

				// Get the correct Y
				//corrY = intermediateOdo[1] - (dTheta / Math.abs(dTheta) * offset);
			}
			
			odo.setX(corrX);

		} else {

			// Compute the robot's X-coordinate in cm's by adding/subtracting sensor offset
			// to the sensor's X-coordinate (in cm's)
			if (corrTheta == 0) {
				// Compute the sensors' Y position in cm's
				double sensorY = odo.getXYT()[1] - SENSOR_DIST;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrSensorY = (int) Math.round(sensorY / TILE_SIZE);

				// Get the correct Y
				corrY = TILE_SIZE * corrSensorY + SENSOR_DIST;

				// Get the correct X
				//corrX = intermediateOdo[0] - (dTheta / Math.abs(dTheta) * offset);

			} else {
				// Compute the sensors' Y position in cm's
				double sensorY = odo.getXYT()[1] + SENSOR_DIST;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrSensorY = (int) Math.round(sensorY / TILE_SIZE);

				// Get the correct Y
				corrY = TILE_SIZE * corrSensorY - SENSOR_DIST;

				// Get the correct X
				//corrX = intermediateOdo[0] + (dTheta / Math.abs(dTheta) * offset);
			}
			odo.setY(corrY);
		}

		// Correct the odometer
		//odo.setXYT(corrX, corrY, corrTheta);
		odo.setTheta(corrTheta);

		rc.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

		// Sleep for 250 ms
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * Set the identifyingBlock boolean
	 * 
	 * @param identifyingBlock
	 */
	public void setIdentifyingBlock(boolean identifyingBlock) {
		this.identifyingBlock = identifyingBlock;
	}
}
