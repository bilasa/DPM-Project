/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

/**
 * Corrects the odometer and ensures the robot
 * travels forward in a straight path
 * 
 * @author Bijan Sadeghi
 *
 */
public class OdometryCorrection implements Runnable {

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

	public OdometryCorrection(double TILE_SIZE, double SENSOR_DIST, RobotController rc, LightSensorController leftLsCont, LightSensorController rightLsCont) {
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
	public void run() {
		
		while(true) {
			// Apply the correction if not paused
			if(!paused) {
				boolean rightLineDetected = false;
				boolean leftLineDetected = false;

				// Check the state of OdometryCorrection
				switch(corrState) {
				case CORRECTING:
					if (rightLsCont.getColorSample()[0] != 13.0 && leftLsCont.getColorSample()[0] != 13.0) {
						corrState = CorrState.EXPECTING_LINE;
					}
					break;
				case EXPECTING_LINE:
					// If line detected with right sensor
					if(rightLsCont.getColorSample()[0] == 13.0) {
						rightLineDetected = true;
						// Stop the right motor
						//rc.stopMoving(false, true);
						rc.setSpeeds(100, 0);
					} else if (leftLsCont.getColorSample()[0] == 13.0) {
						leftLineDetected = true;
						// Stop the left motor
						//rc.stopMoving(true, false);
						rc.setSpeeds(0, 100);
					}

					while (rightLineDetected || leftLineDetected) {
						//Sound.beep();
						LCD.drawString("Right: " + rightLineDetected, 0, 5);
						LCD.drawString("Left: " + leftLineDetected, 0, 6);
						
						// Set the state to correcting
						corrState = CorrState.CORRECTING;

						// If other line detected
						if (rightLineDetected && leftLsCont.getColorSample()[0] == 13.0) {
							leftLineDetected = true;
						} else if (leftLineDetected && rightLsCont.getColorSample()[0] == 13.0) {
							rightLineDetected = true;
						}

						// If both lines have been detected, the robot is straight
						if (rightLineDetected && leftLineDetected) {
							//rc.stopMoving();

							// Travel to the original target destination
							//rc.travelTo(targetX, targetY, rc.FORWARD_SPEED, false);
							rc.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

							// Set both line detections back to false
							rightLineDetected = false;
							leftLineDetected = false;
						}


					}
					break;
				}
			}
		}
	}
}
