package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * Localizes the robot at a waypoint using the left rear light sensor
 * 
 * @author Bijan Sadeghi & Esa Khan
 */
public class LightLocalizer {

	// Constants
	private final int FORWARD_SPEED;
	private final int ROTATE_SPEED;
	private final double TILE_SIZE;
	private final double SENSOR_DIST;

	// Robot controller
	private RobotController rc;

	// Light sensor controller
	private LightSensorController lsCont;

	// Odometer
	private Odometer odo;

	public LightLocalizer(double TILE_SIZE, double SENSOR_DIST, RobotController rc, LightSensorController lsCont) {
		this.FORWARD_SPEED = rc.FORWARD_SPEED;
		this.ROTATE_SPEED = rc.ROTATE_SPEED;
		this.TILE_SIZE = TILE_SIZE;
		this.SENSOR_DIST = SENSOR_DIST;
		this.rc = rc;
		this.lsCont = lsCont;
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Light localize the robot in its initial corner
	 * 
	 * @param startingCorner
	 * @param playZoneCorners
	 */
	public void initialLightLocalize(int startingCorner, int[][] playZoneCorners) {

		rc.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		rc.turnTo(45);

		rc.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		rc.moveForward();

		while (rc.isMoving()) {
			// Reach a line, stop
			if (lsCont.getColorSample()[0] == 13.0) {
				rc.stopMoving();
				Sound.beep();
			}
		}
		// Back up a little to be sure the robot is in the third quadrant
		rc.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		rc.travelDist(-15, true);

		// Do a circle and check the lines
		rc.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		rc.turnBy(360, false);

		double[] angles = new double[4];
		int lineCount = 0;
		while (rc.isMoving()) {
			if (lsCont.getColorSample()[0] == 13.0) {
				angles[lineCount] = Math.toRadians(odo.getXYT()[2]);
				Sound.beep();
				lineCount++;
			}
		}

		// Compute the correction
		odo.setX(-SENSOR_DIST * (Math.cos((angles[3] - angles[1]) / 2)));
		odo.setY(-SENSOR_DIST * (Math.cos((angles[2] - angles[0]) / 2)));

		// Move to the origin
		rc.directTravelTo(0, 0, FORWARD_SPEED, true);
		rc.rotate(false, ROTATE_SPEED); // rotate the robot counterclockwise

		// Set the angle to perfect 0
		while (rc.isMoving()) {
			// The robot is in the third quadrant, if the robot turns counter clockwise
			// the first line it will cross will be at angle -24 based on experimental results
			if (lsCont.getColorSample()[0] == 13.0) {
				rc.stopMoving();
				rc.turnBy(29, true);
				break;
			}

			// Change the robot's direction if it "missed the line"
			if (odo.getXYT()[2] > 280 && odo.getXYT()[2] < 300) {
				rc.rotate(true, ROTATE_SPEED); // rotate the robot clockwise
			}
		}
		// Set the position and angle depending on starting corner
		switch (startingCorner) {
		case 0:	// Lower left
			odo.setXYT(playZoneCorners[0][0] * TILE_SIZE, playZoneCorners[0][1] * TILE_SIZE, 0);
			break;
		case 1:	// Lower right
			odo.setXYT(playZoneCorners[1][0] * TILE_SIZE, playZoneCorners[1][1] * TILE_SIZE, 270);
			break;
		case 2:	// Upper right
			odo.setXYT(playZoneCorners[2][0] * TILE_SIZE, playZoneCorners[2][1] * TILE_SIZE, 180);
			break;
		case 3:	// Upper left
			odo.setXYT(playZoneCorners[3][0] * TILE_SIZE, playZoneCorners[3][1] * TILE_SIZE, 90);
			break;
		}
		
		rc.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
		
		// Sleep for 1 second after localization
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
	}

	/**
	 * Light localize the robot at a general waypoint (not in the initial corner)
	 */
	public void generalLightLocalize() {
		// Compute the nearest waypoint from the odometer reading
		int corrX = (int)Math.round(odo.getXYT()[0] / TILE_SIZE);
		int corrY = (int)Math.round(odo.getXYT()[1] / TILE_SIZE);

		// Turn the robot to 45 degrees
		rc.turnTo(45);

		// Start moving forward
		rc.moveForward();
		
		// Store the initial odometer reading
		double[] position = odo.getXYT();
		double distMoved = 0.0;
		
		// Keep moving until black line is detected or robot has moved 15 cm
		while(rc.isMoving()) {
			if (lsCont.getColorSample()[0] == 13.0) {
				rc.stopMoving();
				break;
			}
			distMoved = Math.hypot(position[0] - odo.getXYT()[0], position[1] - odo.getXYT()[1]);
			
			if (distMoved >= 15.0) {
				rc.moveBackward();
			}
		}
		
		// Go back and left to enter the "bottom-left quadrant"
		rc.travelDist(-17, true);
		
		// Do a circle and check the lines
		rc.setSpeeds(ROTATE_SPEED, ROTATE_SPEED);
		rc.turnBy(360, false);

		double[] angles = new double[4];
		int lineCount = 0;
		while (rc.isMoving()) {
			if (lsCont.getColorSample()[0] == 13.0) {
				angles[lineCount] = Math.toRadians(odo.getXYT()[2]);
				Sound.beep();
				lineCount++;
			}
		}

		// Set the odometer to the original (actual) x, y
		double origX = (TILE_SIZE * corrX) - SENSOR_DIST * (Math.cos((angles[3] - angles[1]) / 2));
		double origY = (TILE_SIZE * corrY) - SENSOR_DIST * (Math.cos((angles[2] - angles[0]) / 2));
		odo.setX(origX);
		odo.setY(origY);

		// Move to the waypoint
		rc.directTravelTo(corrX, corrY, FORWARD_SPEED, true);
		rc.rotate(false, ROTATE_SPEED); // rotate the robot counterclockwise

		/*// Compute the correction
		double dx = SENSOR_DIST * (Math.cos((angles[3] - angles[1]) / 2));
		double dy = SENSOR_DIST * (Math.cos((angles[2] - angles[0]) / 2));

		// Move to the origin
		double absTheta = Math.toDegrees(Math.atan(dx / dy));
		double dist = Math.hypot(dx, dy);
		rc.turnTo(absTheta);
		rc.travelDist(dist, true);*/
		//rc.rotate(false, ROTATE_SPEED); // rotate the robot counterclockwise


		// Set the angle to perfect 0
		while (rc.isMoving()) {
			// The robot is in the third quadrant, if the robot turns counter clockwise
			// the first line it will cross will be at angle 0
			if (lsCont.getColorSample()[0] == 13.0) {
				rc.stopMoving();
				odo.setTheta(0);
				break;
			}

			// Change the robot's direction if it "missed the line"
			if (odo.getXYT()[2] > 300 && odo.getXYT()[2] < 320) {
				rc.rotate(true, ROTATE_SPEED); // rotate the robot clockwise
			}
		}


		rc.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

		// Sleep for 1 second after localization
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}