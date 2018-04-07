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
import lejos.hardware.Wifi;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

/**
 * This class includes all flag searching tasks of the robot.
 * The class allows the execution of the search algorithm.
 * The FlagSearcher makes the robot search for its target block
 * by going on the perimeter of the search zone. When a block is
 * found, the robot moves towards it to identify it. If the target
 * block is found or if too much time was spent, the robot ends its search.
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
	private static LightSensorController lsCont = new LightSensorController(frontColorSensor, frontRGBColor, frontRGBColorSample);

	// Odometer
	private Odometer odo;
	
	// Odmometry Correction
	private OdometryCorrection odoCorrection;

	// Enumeration to represent the state of the search
	private SearchState searchState;

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
	
	/**
	 * Set the OdometryCorrection object to be used by the robot controller
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorrection = odoCorrection;
	}

	public void searchFlag(Flag color) {

		// Position after identifying block
		double[] currPos = {0,0,0};

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
			// Time elapsed
			long timeElapsed = System.currentTimeMillis() - START_TIME;

			// If 3.5 minutes have elapsed since the beginning, time out the search
			if(timeElapsed > 150000) {
				searchState = SearchState.TIMED_OUT;
			}

			// Check for blocks with the ultrasonic sensor
			int usDist = usCont.getAvgUSDistance();
			if(usDist < DETECT_THRESH) {
				// Block was found; stop moving and notify user
				rc.stopMoving();
				Sound.beepSequence();

				// Go and check the block
				identifyBlock(usDist);
				
				// When the block is identified, keep going towards the current destination corner
				rc.directTravelTo(nextCorner[0], nextCorner[1], rc.ROTATE_SPEED, false);
				
				// Sleep so that the same block is not detected again
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

			// If the robot is stopped, it means it reached the destination corner, 
			// so the next corner has to be set as the destination
			currPos = odo.getXYT();
			if(!rc.isMoving()) {
			//if(rc.euclideanDistance(currPos[0], currPos[1], nextCorner[0], nextCorner[1]) < FRONT_SENSOR_DIST) {
				// Finish the travel with correction
				odoCorrection.correct(currPos[2], currPos);
				rc.travelDist(-rc.REAR_SENSOR_DIST, true);
				
				// Set new destination
				currentCorner = nextCorner;
				nextCorner = nextSearchCorner(currentCorner);
				
				// Start travelling
				rc.directTravelTo(nextCorner[0], nextCorner[1], rc.ROTATE_SPEED, false);
			}

		}
		// Flag has been found or search was timed-out, so finish the
		// current travelling and go back to the starting corner of the search
		rc.travelTo(nextCorner[0], nextCorner[1], rc.FORWARD_SPEED);
		rc.travelTo(startingSearchCorner[0], startingSearchCorner[1], rc.FORWARD_SPEED);
	}

	/**
	 * Approaches the block that has been detected and
	 * checks if its color matches the target
	 * 
	 * @param distanceDetected the distance at which the ultrasonic captured a block
	 */
	private void identifyBlock(double distanceDetected) {	
		
		// Position before identification
		double[] initialPos = {0,0,0};
		double[] finalPos = {0,0,0};
		
		// Move forward by the front sensor offset
		rc.setSpeeds(150, 150);
		rc.travelDist(FRONT_SENSOR_DIST + 4, true);

		// If the block was detected close enough already, sample directly
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
		// Approach the block if it is too far
		else {
			// Turn to the left by 90 degrees
			rc.turnBy(-90, true);
			
			//Turn the sensor to 0 degrees, facing forward
			usCont.rotateSensorTo(0);

			// Record the position before approaching the block
			initialPos = odo.getXYT();
			
			// Keep moving forward until the block is close enough or until the robot traveled too far in case of a false positive
			rc.moveForward();
			while(usCont.getAvgUSDistance() > IDENTIFY_THRESH) {
				// Intermediate position used to know if the robot traveled too far without noticing a block
				double[] interPos = odo.getXYT();
				if(Math.hypot(interPos[0] - initialPos[0], interPos[1] - initialPos[1]) > distanceDetected) {
					break;
				}
			}
			
			rc.stopMoving();

			// Get the block's color
			if(LightSensorController.getBlockColor(lsCont.getColorSample()) == wifi.getFlagColor()) {
				Sound.beep();
				searchState = SearchState.FLAG_FOUND;
			}
			else {
				Sound.twoBeeps();
			}
			
			finalPos = odo.getXYT();
			double dist = Math.hypot(finalPos[0] - initialPos[0], finalPos[1] - initialPos[1]);
			
			// Go back by the distance traveled to reach the block
			rc.travelDist(-dist, true);

			// Turn back on the initial path
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
