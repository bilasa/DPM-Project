package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.SearchState;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.WiFi;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.Wifi;
import lejos.hardware.lcd.LCD;

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
public class FlagSearcher implements Runnable {

	// WiFi class
	private WiFi wifi;

	// Robot controller
	private RobotController rc;

	// Ultrasonic sensor controller
	private UltrasonicSensorController usCont;

	// Front light sensor controller
	private LightSensorController lsCont;

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

	/**
	 * @param wifi the wifi object to get the challenge data from
	 * @param rc the robot controller to use
	 */
	public FlagSearcher(WiFi wifi, RobotController rc, UltrasonicSensorController usCont, LightSensorController frontLsCont, double FRONT_SENSOR_DIST, long START_TIME) {
		this.wifi = wifi;
		this.rc = rc;
		this.usCont = usCont;
		this.lsCont = frontLsCont;
		this.searchState = SearchState.IN_PROGRESS;
		this.FRONT_SENSOR_DIST = FRONT_SENSOR_DIST;
		this.START_TIME = START_TIME;
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	@Override
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

	public SearchState getSearchState() {
		return searchState;
	}

	/**
	 * Returns true if a block has been detected, which occurs
	 * when the distance detected by the ultrasonic sensor is less
	 * than 
	 * 
	 * @return whether a block has been detected by the ultrasonic sensor
	 */
	private boolean blockDetected(int previousDist) {
		int usDist = usCont.getAvgUSDistance();

		if (usDist < previousDist && usDist <= DETECT_THRESH)
			return true;

		return false;
	}

	/**
	 * Approaches the block that has been detected and
	 * checks if its color matches the target
	 */
	private void identifyBlock(double distanceDetected) {
		// Move forward by the front sensor offset
		rc.travelDist(FRONT_SENSOR_DIST, true);

		// Turn to the left by 90 degrees
		rc.turnBy(-90, true);
		
		//Turn the sensor to 0 degrees
		usCont.rotateSensorTo(0);

		// Get the initial odometer reading
		//double[] initialPosition = odo.getXYT();

		// Keep moving forward while the ultrasonic sensor distance is less than the threshold
		/*rc.moveForward();
		while(usCont.getAvgUSDistance() > IDENTIFY_THRESH) {
			// Keep moving
			LCD.drawString("US DIST: " + usCont.getAvgUSDistance(), 0, 5);
		}

		// Stop the robot, the block is close enough
		rc.stopMoving();*/
		
		// Travel by the distance the sensor detected the object at
		rc.travelDist(distanceDetected - 17, true);

		// Get the block's color
		if(lsCont.getBlockColor(lsCont.getColorSample()) == wifi.getFlagColor()) {
			Sound.beep();
			searchState = SearchState.FLAG_FOUND;
		}

		// Get the final odometer reading
		//double[] finalPosition = odo.getXYT();
		
		// Compute the distance we traveled to reach the block
		//double distTraveled = Math.hypot(initialPosition[0] - finalPosition[0], initialPosition[1] - finalPosition[1]);

		// Go back by the same distance
		rc.travelDist(-(distanceDetected - 17), true);
		
		// Turn back
		rc.turnBy(90, true);
		
		// Rotate the sensor back to the left
		usCont.rotateSensorTo(90);
	}
	
	/**
	 * Set mainThread to the main thread in order to be able to pause it
	 * 
	 * @param mainThread
	 */
	public void setMainThread(Thread mainThread) {
		this.mainThread = mainThread;
	}

	/**
	 * Travels to the corner of the search zone closest to the robot
	 * after it has crossed the bridge/tunnel into the opponent
	 * team's zone.
	 */
	/*public void travelToSearchZone() {
		rc.travelTo(startingSearchCorner[0], startingSearchCorner[1], rc.FORWARD_SPEED, true);
	}

	/**
	 * Searches for the flag in the search zone. Navigates on the rectangular 
	 * perimeter of the search zone with the ultrasonic sensor facing the
	 * interior of the search zone. Continuously checks for falling edge signals,
	 * which would indicate the presence of a block. When a block is detected,
	 * the robot turns towards the interior of the search zone, approaches the
	 * block to a given threshold distance, and identifies the color of the block.
	 * If the block is the target, it beeps twice and ends its search. Otherwise,
	 * it backs up to the perimeter and continues its search.
	 * 
	 */
	/*public void searchFlag() {
		// Initialize search thread
		// _______ 

		int[] currentCorner = startingSearchCorner;
		int[] nextCorner = nextSearchCorner(currentCorner);		
		rc.travelTo(nextCorner[0], nextCorner[1], rc.FORWARD_SPEED, true);

		// Keep traveling to the next corner as long as the search is in progress
		// The robot will stop traveling to the next corner if the search state is either:
		//	 1. TIMED_OUT
		//   2. FLAG_FOUND
		while (searchState == SearchState.IN_PROGRESS) {
			currentCorner = nextCorner;
			nextCorner = nextSearchCorner(currentCorner);
			rc.travelTo(nextCorner[0], nextCorner[1], rc.FORWARD_SPEED, true);
		}

	}

	/**
	 * Gets the corner of the search zone closest to the robot after crossing
	 * the tunnel/bridge into the opponent's zone.
	 * 
	 * @return the corner of the search zone closest to the robot after it has crossed
	 */
	/*private int[] getClosestSearchCorner() {
		/*Team opponentTeam = wifi.getTeam();
		if (wifi.getTeam() == Team.GREEN) {
			opponentTeam = Team.RED;
		}else if(wifi.getTeam() == Team.RED){
			opponentTeam = Team.GREEN;
		}

		switch(wifi.getStartingCorner(opponentTeam)) {
		case 0:
			return wifi.getSearchZone(opponentTeam)[2];
		case 1:
			return wifi.getSearchZone(opponentTeam)[3];
		case 2:
			return wifi.getSearchZone(opponentTeam)[0];
		case 3:
			return wifi.getSearchZone(opponentTeam)[1];
		}*/

	// Look for the closest corner of the search zone to the robot
	/*double shortestDist = Double.MAX_VALUE;
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
	/*private int[][] getSearchZone() {
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
	/*private int[] nextSearchCorner(int[] currentCorner) {

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
	}*/

}
