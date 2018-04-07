package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.Flag;
import ca.mcgill.ecse211.enumeration.SearchState;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.WiFi;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class includes all flag searching tasks of the robot. The class allows
 * the execution of the search algorithm. The FlagSearcher makes the robot
 * search for its target block by going on the perimeter of the search zone.
 * When a block is found, the robot moves towards it to identify it. If the
 * target block is found or if too much time was spent, the robot ends its
 * search.
 * 
 * @author Bijan Sadeghi
 * @author Esa Khan
 * @author Guillaume Richard
 */
public class FlagSearcher {

	// WiFi class
	private WiFi wifi;

	// Robot controller
	private RobotController rc;

	// Ultrasonic sensor controller
	private UltrasonicSensorController usCont;

	// Front light sensor
	private static final EV3ColorSensor frontColorSensor = new EV3ColorSensor(SensorPort.S4);
	private static SensorMode frontRGBColor = frontColorSensor.getRGBMode();
	private static float[] frontRGBColorSample = new float[frontRGBColor.sampleSize()];

	// Front light sensor controller
	private static LightSensorController lsCont = new LightSensorController(frontColorSensor, frontRGBColor,
			frontRGBColorSample);

	// Odometer
	private Odometer odo;

	// Odmometry Correction
	private OdometryCorrection odoCorrection;

	// Enumeration to represent the state of the search
	private SearchState searchState;

	// Constants
	private final int DETECT_THRESHOLD = 60; // Minimum distance at which block is detected
	private final int IDENTIFY_THRESH = 5; // Minimum distance at which a block is identified
	private final int SEARCH_SPEED;
	private final double FRONT_SENSOR_DIST;
	private final long START_TIME;
	private final int[][] SEARCH_ZONE;

	/**
	 * @param wifi
	 *            the wifi object to get the challenge data from
	 * @param rc
	 *            the robot controller to use
	 */
	public FlagSearcher(WiFi wifi, RobotController rc, UltrasonicSensorController usCont, double FRONT_SENSOR_DIST,
			long START_TIME, int SEARCH_SPEED) {
		this.wifi = wifi;
		this.rc = rc;
		this.usCont = usCont;
		this.searchState = SearchState.IN_PROGRESS;
		this.FRONT_SENSOR_DIST = FRONT_SENSOR_DIST;
		this.START_TIME = START_TIME;
		this.SEARCH_SPEED = SEARCH_SPEED;
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.SEARCH_ZONE = getSearchZone();
	}

	/**
	 * Set the OdometryCorrection object to be used by the robot controller
	 * 
	 * @param odoCorrection
	 *            the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorrection = odoCorrection;
	}

	public void searchFlag(Flag color) {

		// Rotate the sensor to the left
		usCont.rotateSensorTo(90);

		// Travel to the next corner of the search zone
		int[] startingSearchCorner = getClosestSearchCorner();
		int[] currentCorner = startingSearchCorner;
		int[] nextCorner = nextSearchCorner(currentCorner);
		rc.directTravelTo(nextCorner[0], nextCorner[1], rc.ROTATE_SPEED, false);

		try {
			Thread.sleep(750);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Keep traveling to the next corner as long as the search is in progress
		// The robot will stop traveling to the next corner if the search state is
		// either:
		// 1. TIMED_OUT
		// 2. FLAG_FOUND
		while (searchState == SearchState.IN_PROGRESS) {
			// Time elapsed
			long timeElapsed = System.currentTimeMillis() - START_TIME;

			// If 4 minutes have elapsed since the beginning, time out the search
			if (timeElapsed > 240000) {
				searchState = SearchState.TIMED_OUT;
				// Play sound when timed out
				Sound.playTone(440, 1000);
				Sound.playTone(500, 1000);
				
			}

			// Check for blocks with the ultrasonic sensor
			int usDist = usCont.getAvgUSDistance();
			if (usDist < DETECT_THRESHOLD && withinArea(usDist, SEARCH_ZONE)) {
				// Block was found; stop moving and notify user
				rc.stopMoving();
				Sound.beepSequence();

				// Go and check the block
				identifyBlock();

				try {
					Thread.sleep(500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

				// When the block is identified, keep going towards the current destination
				// corner
				rc.directTravelTo(nextCorner[0], nextCorner[1], rc.ROTATE_SPEED, false);

				// Sleep so that the same block is not detected again
				try {
					Thread.sleep(2500);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

			// If the robot is stopped, it means it reached the destination corner,
			// so the next corner has to be set as the destination
			if (!rc.isMoving()) {
				// if(rc.euclideanDistance([0], [1], nextCorner[0], nextCorner[1])
				// < FRONT_SENSOR_DIST) {
				// Finish the travel with correction
				odoCorrection.correct();
				rc.travelDist(-rc.REAR_SENSOR_DIST, true);
				// Rotate towards the next corner and correct first
				// rc.turnBy(-90, true);

				// odoCorrection.correct();

				// Set new destination
				currentCorner = nextCorner;
				nextCorner = nextSearchCorner(currentCorner);

				try {
					Thread.sleep(250);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}

				// Start travelling
				rc.directTravelTo(nextCorner[0], nextCorner[1], rc.ROTATE_SPEED, false);
			}

		}

		// Rotate the sensor back to 0 degrees
		usCont.rotateSensorTo(0);

		// Flag has been found or search was timed-out, so finish the
		// current travelling and go back to the starting corner of the search
		rc.travelTo(nextCorner[0], nextCorner[1], rc.FORWARD_SPEED);
		rc.travelTo(startingSearchCorner[0], startingSearchCorner[1], rc.FORWARD_SPEED);

	}

	/**
	 * Approaches the block that has been detected and checks if its color matches
	 * the target
	 * 
	 */
	private void identifyBlock() {

		// Take another samples to make sure a block is really detected
		int distanceDetected = usCont.getAvgUSDistance();

		// Check the validity of the new sample
		if (distanceDetected > DETECT_THRESHOLD) {
			Sound.twoBeeps();
			// Do nothing and go back to travelling
			return;
		}

		// Position before identification
		double[] initialPos = { 0, 0, 0 };
		double[] finalPos = { 0, 0, 0 };

		// Move forward by the front sensor offset
		rc.setSpeeds(SEARCH_SPEED, SEARCH_SPEED);
		rc.travelDist(FRONT_SENSOR_DIST + 2, true);

		// If the block was detected close enough already, sample directly
		if (distanceDetected < 5) {
			// Get the block's color
			if (LightSensorController.getBlockColor(lsCont.getColorSample()) == wifi.getFlagColor()) {
				Sound.beep();
				searchState = SearchState.FLAG_FOUND;
			} else {
				Sound.twoBeeps();
			}
		}
		// Approach the block if it is too far
		else {
			// Turn to the left by 90 degrees
			rc.turnBy(-90, true);

			// Turn the sensor to 0 degrees, facing forward
			usCont.rotateSensorTo(0);

			// Record the position before approaching the block
			initialPos = odo.getXYT();

			// Keep moving forward until the block is close enough or until the robot
			// traveled too far in case of a false positive
			rc.moveForward();
			while (usCont.getAvgUSDistance() > IDENTIFY_THRESH) {
				// Intermediate position used to know if the robot traveled too far without
				// noticing a block
				double[] interPos = odo.getXYT();
				if (Math.hypot(interPos[0] - initialPos[0], interPos[1] - initialPos[1]) > distanceDetected) {
					break;
				}
			}

			rc.stopMoving();

			// Get the block's color
			if (LightSensorController.getBlockColor(lsCont.getColorSample()) == wifi.getFlagColor()) {
				Sound.beep();
				searchState = SearchState.FLAG_FOUND;
			}

			usCont.rotateSensorTo(45);
			if (searchState != SearchState.FLAG_FOUND
					&& LightSensorController.getBlockColor(lsCont.getColorSample()) == wifi.getFlagColor()) {
				Sound.beep();
				searchState = SearchState.FLAG_FOUND;
			}

			usCont.rotateSensorTo(-45);
			if (searchState != SearchState.FLAG_FOUND
					&& LightSensorController.getBlockColor(lsCont.getColorSample()) == wifi.getFlagColor()) {
				Sound.beep();
				searchState = SearchState.FLAG_FOUND;
			}

			if (searchState != SearchState.FLAG_FOUND) {
				Sound.twoBeeps();
			}

			finalPos = odo.getXYT();
			double dist = Math.hypot(finalPos[0] - initialPos[0], finalPos[1] - initialPos[1]);

			// Go back by the distance traveled to reach the block
			rc.travelDist(-dist, true);

			// Turn back on the initial path
			rc.turnBy(90, true);

			// rc.travelDist(-(FRONT_SENSOR_DIST + 2), true);

			try {
				Thread.sleep(250);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			// Rotate the sensor back to the left
			usCont.rotateSensorTo(90);
		}
	}

	private boolean withinArea(float distance, int[][] searchZone) {
		float usAngle = 90;
		boolean result = false;
		double[] data;
		double absAngle = 0, xPos = 0, yPos = 0;

		data = odo.getXYT();

		double URy = searchZone[2][1] * rc.TILE_SIZE;
		double URx = searchZone[2][0] * rc.TILE_SIZE;
		double LLx = searchZone[0][0] * rc.TILE_SIZE;
		double LLy = searchZone[0][1] * rc.TILE_SIZE;

		absAngle = data[2] - usAngle;

		if (absAngle < 0)
			absAngle += 360;

		if (absAngle >= 0 && absAngle < 90) {
			xPos = distance * Math.sin(Math.toRadians(absAngle));
			yPos = distance * Math.cos(Math.toRadians(absAngle));

			xPos += data[0];
			yPos += data[1] + FRONT_SENSOR_DIST;

		} else if (absAngle >= 90 && absAngle < 180) {
			xPos = distance * Math.cos(Math.toRadians(absAngle - 90));
			yPos = distance * Math.sin(Math.toRadians(absAngle - 90));

			xPos += data[0] + FRONT_SENSOR_DIST;
			yPos -= data[1];

		} else if (absAngle >= 180 && absAngle < 270) {
			xPos = distance * Math.sin(Math.toRadians(absAngle - 180));
			yPos = distance * Math.cos(Math.toRadians(absAngle - 180));

			xPos -= data[0];
			yPos -= data[1] + FRONT_SENSOR_DIST;

		} else {
			xPos = distance * Math.cos(Math.toRadians(absAngle - 270));
			yPos = distance * Math.sin(Math.toRadians(absAngle - 270));

			xPos -= data[0] + FRONT_SENSOR_DIST;
			yPos += data[1];

		}

		xPos = Math.abs(xPos);
		yPos = Math.abs(yPos);

		xPos = (int) xPos;
		yPos = (int) yPos;

		if (xPos > LLx && xPos < URx && yPos > LLy && yPos < URy) {
			result = true;
		}

		return result;
	}

	/**
	 * Gets the corner of the search zone closest to the robot after crossing the
	 * tunnel/bridge into the opponent's zone.
	 * 
	 * @return the corner of the search zone closest to the robot after it has
	 *         crossed
	 */
	public int[] getClosestSearchCorner() {

		// Look for the closest corner of the search zone to the robot
		double shortestDist = Double.MAX_VALUE;
		int[] closestCorner = SEARCH_ZONE[0];
		for (int[] corner : SEARCH_ZONE) {
			double cornerDist = Math.hypot(odo.getXYT()[0] - (corner[0] * rc.TILE_SIZE),
					odo.getXYT()[1] - (corner[1] * rc.TILE_SIZE));

			if (cornerDist < shortestDist) {
				shortestDist = cornerDist;
				closestCorner = corner;
			}
		}

		// Return the closest corner found
		return closestCorner;
	}

	/**
	 * Gets the search zone of the opponent team (which is the search zone the robot
	 * will search in)
	 * 
	 * @return a two-dimensional int array containing four (x, y) pairs for each
	 *         corner of the search zone
	 */
	private int[][] getSearchZone() {
		// Get the opponent team
		Team opponentTeam = wifi.getTeam();
		if (wifi.getTeam() == Team.GREEN) {
			opponentTeam = Team.RED;
		} else if (wifi.getTeam() == Team.RED) {
			opponentTeam = Team.GREEN;
		}

		// Get the search zone of the opponent team
		return wifi.getSearchZone(opponentTeam);
	}

	/**
	 * Gets the next corner of the search zone the robot should travel to based on
	 * where it is now
	 * 
	 * @param currentCorner
	 * @return an int array holding the (x, y) of the next search corner
	 */
	private int[] nextSearchCorner(int[] currentCorner) {

		// Get the index of the current corner
		int currentCornerIndex = 0;
		for (int i = 0; i < SEARCH_ZONE.length; i++) {
			if (SEARCH_ZONE[i][0] == currentCorner[0] && SEARCH_ZONE[i][1] == currentCorner[1]) {
				currentCornerIndex = i;
				break;
			}
		}

		// Return the next corner in the search zone array
		if (currentCornerIndex < 3)
			return SEARCH_ZONE[currentCornerIndex + 1];
		else
			return SEARCH_ZONE[0];
	}

}
