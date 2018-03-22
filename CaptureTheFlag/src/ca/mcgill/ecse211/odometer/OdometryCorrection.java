/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;

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

	// Light sensor controller
	private LightSensorController lsCont;

	// Odometer
	private Odometer odo;

	public OdometryCorrection(double TILE_SIZE, double SENSOR_DIST, RobotController rc, LightSensorController lsCont) {
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
	 * Here is where the odometer correction code should be run.
	 * 
	 * @throws OdometerExceptions
	 */
	// run method (required for Thread)
	public void run() {

	}
}
