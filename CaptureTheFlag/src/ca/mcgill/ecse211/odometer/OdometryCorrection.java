/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

/**
 * Corrects the odometer and ensures the robot travels forward in a straight
 * path
 * 
 * @author Bijan Sadeghi
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

	// Target x, y point from the travelTo call
	private int targetX;
	private int targetY;

	// Left and right rear sensor controllers
	private LightSensorController leftLsCont;
	private LightSensorController rightLsCont;

	// Odometer
	private Odometer odo;

	// Paused variable to tell pause/unpause the OdometryCorrection
	private boolean paused;

	// Enumeration to denote the state of the OdometryCorrection
	private enum CorrState {
		EXPECTING_LINE, CORRECTING
	}

	private CorrState corrState;

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
	 * Pause the OdometryCorrection
	 * 
	 * @param paused
	 */
	public void setPaused(boolean paused) {
		this.paused = paused;
	}

	public void setTargetXY(int x, int y) {
		this.targetX = x;
		this.targetY = y;
	}

	/**
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	/*
	 * public void run() {
	 * 
	 * while(true) { // Apply the correction if not paused if(!paused) { boolean
	 * rightLineDetected = false; boolean leftLineDetected = false;
	 * 
	 * // Check the state of OdometryCorrection switch(corrState) { case CORRECTING:
	 * if (rightLsCont.getColorSample()[0] != 13.0 && leftLsCont.getColorSample()[0]
	 * != 13.0) { corrState = CorrState.EXPECTING_LINE; } break; case
	 * EXPECTING_LINE: // If line detected with right sensor
	 * if(rightLsCont.getColorSample()[0] == 13.0) { rightLineDetected = true; //
	 * Stop the right motor //rc.stopMoving(false, true); rc.setSpeeds(100, 0); }
	 * else if (leftLsCont.getColorSample()[0] == 13.0) { leftLineDetected = true;
	 * // Stop the left motor //rc.stopMoving(true, false); rc.setSpeeds(0, 100); }
	 * 
	 * while (rightLineDetected || leftLineDetected) { //Sound.beep();
	 * LCD.drawString("Right: " + rightLineDetected, 0, 5); LCD.drawString("Left: "
	 * + leftLineDetected, 0, 6);
	 * 
	 * // Set the state to correcting corrState = CorrState.CORRECTING;
	 * 
	 * // If other line detected if (rightLineDetected &&
	 * leftLsCont.getColorSample()[0] == 13.0) { leftLineDetected = true; } else if
	 * (leftLineDetected && rightLsCont.getColorSample()[0] == 13.0) {
	 * rightLineDetected = true; }
	 * 
	 * // If both lines have been detected, the robot is straight if
	 * (rightLineDetected && leftLineDetected) { //rc.stopMoving();
	 * 
	 * // Travel to the original target destination //rc.travelTo(targetX, targetY,
	 * rc.FORWARD_SPEED, false); rc.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
	 * 
	 * // Set both line detections back to false rightLineDetected = false;
	 * leftLineDetected = false; }
	 * 
	 * 
	 * } break; } } } }
	 */

	/**
	 * Moves the robot forward until a perpendicular line is detected by one of the
	 * sensors. Re-aligns the robot so that both sensors are on the line. Corrects
	 * the Odometer based on its new position.
	 *
	 * @param correctingX
	 *            is the robot being corrected on the X-axis
	 * @param corrTheta
	 *            the angle we must correct the odometer to be
	 */
	public void correct(double corrTheta, double[] initialOdo) {
		// Intermediate odometer reading: odometer's reading after detecting the first line
		double[] intermediateOdo = new double[3];

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
		while (!leftLineDetected && !rightLineDetected) {
			if (rightLsCont.getColorSample()[0] == 13.0) {
				rightLineDetected = true;
				// Stop the right motor
				rc.stopMoving(false, true);
				intermediateOdo = odo.getXYT();

			} else if (leftLsCont.getColorSample()[0] == 13.0) {
				leftLineDetected = true;

				// Stop the left motor
				rc.stopMoving(true, false);
				intermediateOdo = odo.getXYT();
			}
		}

		// Get the odometer's reading 

		// Keep moving the left/right motor until both lines have been detected
		while (!leftLineDetected || !rightLineDetected) {
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
		double[] finalOdo = odo.getXYT();

		// Compute the angle the robot rotated by to correct itself
		double dTheta = finalOdo[2] - intermediateOdo[2];

		// Compute the distance traveled from initial to intermediate
		double dist = Math.hypot(intermediateOdo[0] - initialOdo[0], intermediateOdo[1] - initialOdo[1]);

		// Compute the offset of the sensor's from the line
		double c = rc.TRACK / 2 * Math.tan(Math.toRadians(Math.abs(dTheta)));

		// Compute the robot's offset from its intended traveling axis
		double offset = (dist - SENSOR_DIST + c) * Math.sin(Math.abs(dTheta));

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
				corrY = intermediateOdo[1] + (dTheta / Math.abs(dTheta) * offset);


			} else {
				// Compute the sensors' X position in cm's
				double sensorX = odo.getXYT()[0] + SENSOR_DIST;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrSensorX = (int) Math.round(sensorX / TILE_SIZE);

				// Get the correct X
				corrX = TILE_SIZE * corrSensorX - SENSOR_DIST;

				// Get the correct Y
				corrY = intermediateOdo[1] - (dTheta / Math.abs(dTheta) * offset);
			}

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
				corrX = intermediateOdo[0] - (dTheta / Math.abs(dTheta) * offset);

			} else {
				// Compute the sensors' Y position in cm's
				double sensorY = odo.getXYT()[1] + SENSOR_DIST;

				// Find the X-coordinate of the nearest waypoint to sensorX.
				int corrSensorY = (int) Math.round(sensorY / TILE_SIZE);

				// Get the correct Y
				corrY = TILE_SIZE * corrSensorY - SENSOR_DIST;

				// Get the correct X
				corrX = intermediateOdo[0] + (dTheta / Math.abs(dTheta) * offset);
			}

		}

		// Correct the odometer
		odo.setXYT(corrX, corrY, corrTheta);

		rc.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

		// Sleep for 250 ms
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
