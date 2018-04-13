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
	private final int CORRECTION_SPEED;
	private final double TILE_SIZE;
	private final double SENSOR_DIST;

	// Robot controller
	private RobotController rc;

	// Left and right rear sensor controllers
	private LightSensorController leftLsCont;
	private LightSensorController rightLsCont;

	// Odometer
	private Odometer odo;

	/**
	 * @param TILE_SIZE the size of a tile
	 * @param SENSOR_DIST the vertical offset of the rear sensors from the robot's center
	 * @param rc the robot controller to use
	 * @param leftLsCont the light sensor controller to use to get data from the left rear sensor
	 * @param rightLsCont the light sensor controller to use to get data from the right rear sensor
	 */
	public OdometryCorrection(double TILE_SIZE, double SENSOR_DIST, int CORRECTION_SPEED,
			RobotController rc, LightSensorController leftLsCont,
			LightSensorController rightLsCont) {
		this.FORWARD_SPEED = rc.FORWARD_SPEED;
		this.CORRECTION_SPEED = CORRECTION_SPEED;
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
	}

	/**
	 * Moves the robot forward until a perpendicular line is detected by one of the
	 * sensors. Re-aligns the robot so that both sensors are on the line. Corrects
	 * the Odometer's theta and either the x or the y, based on which axis the robot
	 * is traveling on.
	 */
	public void correct() {
		// Intermediate odometer reading: odometer's reading after detecting the first line
		long initialTime = 0;

		double corrTheta = odo.getXYT()[2];
		
		// Sleep for 250 ms
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Booleans for whether the line has been detected by the right sensor or left
		// sensor or if the correction went for too long
		boolean rightLineDetected = false;
		boolean leftLineDetected = false;

		// Start moving the robot forward
		rc.setSpeeds(CORRECTION_SPEED, CORRECTION_SPEED);
		rc.moveForward();

		// Move the robot until one of the sensors detects a line
		while (!leftLineDetected && !rightLineDetected) {
			if (rightLsCont.getColorSample()[0] == 13.0) {
				rightLineDetected = true;
				// Stop the right motor
				rc.stopMoving(false, true);
				initialTime = System.currentTimeMillis();

			} else if (leftLsCont.getColorSample()[0] == 13.0) {
				leftLineDetected = true;

				// Stop the left motor
				rc.stopMoving(true, false);
				initialTime = System.currentTimeMillis();
			}
		}

		// Get the odometer's reading 

		// Keep moving the left/right motor until both lines have been detected
		while (!leftLineDetected || !rightLineDetected) {
			// If the correction went for too long (1.5 s), abort
			if(System.currentTimeMillis() - initialTime > 1500) {
				return;
			}
			// If the other line detected, stop the motors
			if (rightLineDetected && leftLsCont.getColorSample()[0] == 13.0) {
				leftLineDetected = true;
				rc.stopMoving();
			} else if (leftLineDetected && rightLsCont.getColorSample()[0] == 13.0) {
				rightLineDetected = true;
				rc.stopMoving();
			}
		}
		
		// Get an approximation of the correct theta
		if(corrTheta >= 350 && corrTheta <= 10) {
			corrTheta = 0;
		}
		else if(corrTheta >= 80 && corrTheta <= 100) {
			corrTheta = 90;
		}
		else if(corrTheta >= 170 && corrTheta <= 190) {
			corrTheta = 180;
		}
		else if(corrTheta >= 260 && corrTheta <= 280) {
			corrTheta = 270;
		}

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
}
