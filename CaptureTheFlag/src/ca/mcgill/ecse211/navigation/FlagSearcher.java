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
import lejos.hardware.Sound;
import lejos.hardware.Wifi;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

/**
 * This class includes all flag searching tasks of the robot.
 * The class allows the navigation of the robot to the search
 * zone, as well as the execution of the search algorithm.
 * The FlagSearcher makes the robot search for its target block
 * by going on the perimeter of the search zone. If the target
 * block is found, the robot ends its search.
 * 
 * @author Bijan Sadeghi
 * @author Esa Khan
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
	private static LightSensorController lsCont = new LightSensorController(frontColorSensor, frontRGBColor, frontRGBColorSample);

	// Odometer
	private Odometer odo;

	// Enumeration to represent the state of the search
	private SearchState searchState;

	// Object to store the main thrad
	private Thread mainThread;

	// Constants
	private int DETECT_THRESH = 30; // Minimum distance at which block is detected
	private int IDENTIFY_THRESH = 5; // Minimum distance at which a block is identified
	private double FRONT_SENSOR_DIST;
	private long START_TIME;
	private int[][] searchZone;

	/**
	 * @param wifi the wifi object to get the challenge data from
	 * @param rc the robot controller to use
	 */
	public FlagSearcher(WiFi wifi, RobotController rc, UltrasonicSensorController usCont, double FRONT_SENSOR_DIST, long START_TIME) {
		this.wifi = wifi;
		this.rc = rc;
		this.usCont = usCont;
		this.searchState = SearchState.IN_PROGRESS;
		this.FRONT_SENSOR_DIST = FRONT_SENSOR_DIST;
		this.START_TIME = START_TIME;
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		this.searchZone = getSearchZone();
	}

	/*	@Override
	public void run() {
		// Rotate the sensor to the left
		usCont.rotateSensorTo(90);

		// Get the current distance read by the ultrasonic sensor
		int usDist = usCont.getAvgUSDistance();

		while(searchState == SearchState.IN_PROGRESS) {

			// Time elapsed
			long timeElapsed = System.currentTimeMillis() - START_TIME;

			// If 3.5 minutes have elapsed since the beginning, time out the search
			if(timeElapsed > 150000) {
				searchState = SearchState.TIMED_OUT;
			}

			// If a block is detected, identify it
			if (blockDetected(usDist)) {
				Sound.beepSequence();

				// Make the main thread wait
				mainThread.suspend();

				rc.stopMoving();
				identifyBlock(usDist);

				// Notify the main thread to start it again
				mainThread.resume();
			}

			// Update the current distance read by the ultrasonic sensor
			usDist = usCont.getAvgUSDistance();
		}

		// Rotate the sensor back to 0 degrees
		usCont.rotateSensorTo(0);
	}
	 */

	public void searchFlag(Flag color) {

		// Rotate the sensor to the left
		usCont.rotateSensorTo(90);

		// Travel to the next corner of the search zone
		int[] startingSearchCorner = getClosestSearchCorner();
		int[] currentCorner = startingSearchCorner;
		int[] nextCorner = nextSearchCorner(currentCorner);
		rc.directTravelTo(nextCorner[0], nextCorner[1], rc.ROTATE_SPEED, false);

		// Keep traveling to the next corner as long as the search is in progress
		// The robot will stop traveling to the next corner if the search state is either:
		//	 1. TIMED_OUT
		//   2. FLAG_FOUND
		while(searchState == SearchState.IN_PROGRESS) {
			
			// Keep going until the next corner is reached
			while(!rc.hasReached()) {
				rc.directTravelTo(nextCorner[0], nextCorner[1], rc.ROTATE_SPEED, false);

				// Time elapsed
				long timeElapsed = System.currentTimeMillis() - START_TIME;

				// If 3.5 minutes have elapsed since the beginning, time out the search
				if(timeElapsed > 150000) {
					searchState = SearchState.TIMED_OUT;
				}

				// CHeck for blocks
				int usDist = usCont.getAvgUSDistance();
				if(usDist < DETECT_THRESH) {
					rc.stopMoving();
					Sound.beepSequence();
					identifyBlock(usDist);
				}
			}
			// Next corner has been reached; set new destination
			currentCorner = nextCorner;
			nextCorner = nextSearchCorner(currentCorner);
			rc.directTravelTo(nextCorner[0], nextCorner[1], rc.ROTATE_SPEED, false);
		}
		
		// Flag has been found or search was timed-out
	}

	/**
	 * Approaches the block that has been detected and
	 * checks if its color matches the target
	 */
	private void identifyBlock(double distanceDetected) {	
		// Move forward by the front sensor offset
		rc.travelDist(FRONT_SENSOR_DIST, true);

		if(distanceDetected < 5) {
			// Get the block's color
			if(LightSensorController.getBlockColor(lsCont.getColorSample()) == wifi.getFlagColor()) {
				Sound.beep();
				searchState = SearchState.FLAG_FOUND;
			}
			else {
				Sound.twoBeeps();
			}
		}
		else {
			// Turn to the left by 90 degrees
			rc.turnBy(-90, true);

			//Turn the sensor to 0 degrees
			usCont.rotateSensorTo(0);

			// Travel by the distance the sensor detected the object at
			rc.travelDist(distanceDetected - 17, true);

			// Get the block's color
			if(LightSensorController.getBlockColor(lsCont.getColorSample()) == wifi.getFlagColor()) {
				Sound.beep();
				searchState = SearchState.FLAG_FOUND;
			}
			else {
				Sound.twoBeeps();
			}

			// Go back by the same distance
			rc.travelDist(-(distanceDetected - 17), true);

			// Turn back
			rc.turnBy(90, true);

			// Rotate the sensor back to the left
			usCont.rotateSensorTo(90);
		}
	}


	/**
	 * Gets the corner of the search zone closest to the robot after crossing
	 * the tunnel/bridge into the opponent's zone.
	 * 
	 * @return the corner of the search zone closest to the robot after it has crossed
	 */
	private int[] getClosestSearchCorner() {

		// Look for the closest corner of the search zone to the robot
		double shortestDist = Double.MAX_VALUE;
		int[] closestCorner = searchZone[0];
		for(int[] corner : searchZone) {
			double cornerDist = Math.hypot(odo.getXYT()[0] - (corner[0] * rc.TILE_SIZE), odo.getXYT()[0] - (corner[0] * rc.TILE_SIZE));

			if (cornerDist < shortestDist) {
				shortestDist = cornerDist;
				closestCorner = corner;
			}
		}

		// Return the closest corner found
		return closestCorner;
	}

	/**
	 * Gets the search zone of the opponent team (which is the search zone the robot will search in)
	 * 
	 * @return a two-dimensional int array containing four (x, y) pairs for each corner of the search zone
	 */
	private int[][] getSearchZone() {
		// Get the opponent team
		Team opponentTeam = wifi.getTeam();
		if (wifi.getTeam() == Team.GREEN) {
			opponentTeam = Team.RED;
		}else if(wifi.getTeam() == Team.RED){
			opponentTeam = Team.GREEN;
		}

		// Get the search zone of the opponent team
		return wifi.getSearchZone(opponentTeam);
	}

	/**
	 * Gets the next corner of the search zone the robot should travel to based on where it is now
	 * 
	 * @param currentCorner
	 * @return an int array holding the (x, y) of the next search corner
	 */
	private int[] nextSearchCorner(int[] currentCorner) {

		// Get the index of the current corner
		int currentCornerIndex = 0;
		for(int i=0; i<searchZone.length; i++) {
			if (searchZone[i][0] == currentCorner[0] && searchZone[i][1] == currentCorner[1]) {
				currentCornerIndex = i;
				break;
			}
		}

		// Return the next corner in the search zone array
		if (currentCornerIndex < 3)
			return searchZone[currentCornerIndex + 1];
		else
			return searchZone[0];
	}

}
