package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
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

/**
 * This class is a runnable class from which a thread is
 * created in the navigation class. The thread is created
 * when the flag search is ready to begin. The run method 
 * continuously checks if a block has been detected within 
 * a certain distance threshold. If a block is detected, 
 * the robot approaches and identifies the block. The
 * FlagSearcher keeps track of the state of the search in
 * the form of an enumeration, and the search will end if
 * the state is either TIMED_OUT or FLAG_FOUND.
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

	// Front light sensor
	private static final EV3ColorSensor frontColorSensor = new EV3ColorSensor(SensorPort.S4);
	private static SensorMode frontRGBColor = frontColorSensor.getRGBMode();
	private static float[] frontRGBColorSample = new float[frontRGBColor.sampleSize()];

	// Front light sensor controller
	private static LightSensorController lsCont = new LightSensorController(frontColorSensor, frontRGBColor, frontRGBColorSample);

	// OdometryCorrection object
	private OdometryCorrection odoCorrection;
	
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
				
				// Set identifyingBlock boolean in odometry correction to cancel any running odometry correction
				odoCorrection.setIdentifyingBlock(true);

				// Make the main thread wait
				mainThread.suspend();

				rc.stopMoving();
				identifyBlock(usDist);

				// Notify the main thread to start it again
				mainThread.resume();
				
				// Set identifyingBlock boolean in odometry correction to allow odometry correction
				odoCorrection.setIdentifyingBlock(false);
				
				// Sleep to avoid detecting the same block
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

			// Update the current distance read by the ultrasonic sensor
			usDist = usCont.getAvgUSDistance();
		}

		// Rotate the sensor back to 0 degrees
		usCont.rotateSensorTo(0);
	}

	/**
	 * Returns the state of the search in the form of a SearchState enumeration
	 * 
	 * @return an enumeration representing the state of the search
	 */
	public SearchState getSearchState() {
		return searchState;
	}

	/**
	 * Returns true if a block has been detected, which occurs
	 * when the distance detected by the ultrasonic sensor is less
	 * than the DETECT_THRESH
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
	 * checks if its color matches the target. If it matches,
	 * the robot beeps and the search state is changed to 
	 * FLAG_FOUND.
	 */
	private void identifyBlock(double distanceDetected) {
		// Get the initial odometer reading
		double[] initialOdo = odo.getXYT();
		
		// Move forward by the front sensor offset
		rc.travelDist(FRONT_SENSOR_DIST + 5, true);

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
		rc.travelDist(distanceDetected - 15, true);

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
		
		// Move forward by the front sensor offset
		rc.travelDist(-(FRONT_SENSOR_DIST + 5), true);
		
		// Set the odometer to the initial values
		odo.setXYT(initialOdo[0], initialOdo[1], initialOdo[2]);
	}

	/**
	 * Set mainThread to the main thread in order to be able to pause it
	 * 
	 * @param mainThread the thread to set the mainThread to
	 */
	public void setMainThread(Thread mainThread) {
		this.mainThread = mainThread;
	}
	
	/**
	 * Set the OdometryCorrection object to be used by the robot controller
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorrection = odoCorrection;
	}

}
