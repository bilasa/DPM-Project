package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.navigation.FlagSearcher;
import ca.mcgill.ecse211.navigation.LightLocalizer;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.navigation.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Display;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * This class is the main class of the project. This class is 
 * at the top layer of the layered hierarchy and calls methods
 * from classes in the middle layer (navigation layer). The main 
 * method of the class sequentially calls methods in the navigation
 * layer in order to execute the different high level tasks of the 
 * challenge.
 * 
 * @author Bijan Sadeghi
 * @author Esa Khan
 */
public class CaptureTheFlag {

	// Motors
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

	// Ultrasonic sensor
	private static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
	private static SampleProvider usDistance = usSensor.getMode("Distance");
	private static SampleProvider average = new MeanFilter(usDistance, 8);
	private static float[] usSample = new float[average.sampleSize()];

	// Left rear light sensor
	private static final EV3ColorSensor leftRearColorSensor = new EV3ColorSensor(SensorPort.S2);
	private static SensorMode leftRearColorID = leftRearColorSensor.getColorIDMode();
	private static float[] leftRearColorIDSample = new float[leftRearColorID.sampleSize()];

	// Right rear light sensor
	private static final EV3ColorSensor rightRearColorSensor = new EV3ColorSensor(SensorPort.S3);
	private static SensorMode rightRearColorID = rightRearColorSensor.getColorIDMode();
	private static float[] rightRearColorIDSample = new float[rightRearColorID.sampleSize()];

	// LCD
	private static final TextLCD LCD = LocalEV3.get().getTextLCD();

	// Constants
	private static final double WHEEL_RAD = 1.66;
	private static final double TRACK = 18.65;
	private static final int ROTATE_SPEED = 250;
	private static final int FORWARD_SPEED = 600;
	private static final int CORRECTION_SPEED = 150;
	private static final int SEARCH_SPEED = 150;
	private static final int ACCELERATION = 2000;
	private static final double TILE_SIZE = 30.48;
	private static final double REAR_SENSOR_DIST = 14;
	private static final double FRONT_SENSOR_DIST = 10.0;

	// Playzone constants
	private static final int LL_PZx = 1;
	private static final int LL_PZy = 1;
	private static final int UR_PZx = 11;
	private static final int UR_PZy = 11;
	private static final int[][] PLAY_ZONE = new int[][] { { LL_PZx, LL_PZy }, // Lower left
		{ UR_PZx, LL_PZy }, // Lower right
		{ UR_PZx, UR_PZy }, // Upper right
		{ LL_PZx, UR_PZy } // Upper left
	};

	// Odometer
	private static final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

	// WiFi class
	private static WiFi wifi = new WiFi();
	
	// Start time
	private static final long START_TIME = System.currentTimeMillis();

	// Controllers
	private static RobotController rc = new RobotController(leftMotor, rightMotor, WHEEL_RAD, TRACK, FORWARD_SPEED, ROTATE_SPEED, ACCELERATION, TILE_SIZE, REAR_SENSOR_DIST);
	private static UltrasonicSensorController usCont = new UltrasonicSensorController(usSensor, sensorMotor, usDistance, average, usSample);
	//private static LightSensorController frontLsCont = new LightSensorController(frontColorSensor, frontRGBColor, frontRGBColorSample);
	private static LightSensorController leftRearLsCont = new LightSensorController(leftRearColorSensor, leftRearColorID, leftRearColorIDSample);
	private static LightSensorController rightRearLsCont = new LightSensorController(rightRearColorSensor, rightRearColorID, rightRearColorIDSample);

	// Navigation classes
	private static UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(rc, usCont);
	private static LightLocalizer lightLocalizer = new LightLocalizer(TILE_SIZE, REAR_SENSOR_DIST, rc, leftRearLsCont);
	private static FlagSearcher flagSearcher = new FlagSearcher(wifi, rc, usCont, FRONT_SENSOR_DIST, START_TIME, SEARCH_SPEED);
	private static Navigator navigator = new Navigator(rc, wifi, flagSearcher);

	// Threads
	private static ExitProgram exit = new ExitProgram();

	// Odometry correction
	private static OdometryCorrection odoCorrection = new OdometryCorrection(TILE_SIZE, REAR_SENSOR_DIST, CORRECTION_SPEED, rc, leftRearLsCont, rightRearLsCont);

	/**
	 * First localizes the robot at its starting corner.
	 * Then navigates the robot through the tunnel/bridge.
	 * Then searches for the flag in the opponent's search zone.
	 * Then navigates the robot through the bridge/tunnel.
	 * Finally Returns the robot to its starting corner.
	 * 
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {
		
		// Display
		Display odometryDisplay = new Display(LCD);

		// Odometer thread
		Thread odoThread = new Thread(odometer);
		odoThread.start();

		// If escape button is pressed, exit program
		Thread exitThread = new Thread(exit);
		exit.start();

		// Display thread
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();

		// Timer thread
		//Thread timerThread = new Thread(timer);
		//timer.start();

		// Add odoCorrection to the robot controller and navigator
		rc.setOdoCorrection(odoCorrection);
		navigator.setOdoCorrection(odoCorrection);
		flagSearcher.setOdoCorrection(odoCorrection);


		// ====== Get the robot's team ======  //
		Team team = wifi.getTeam();

		
		//Sound.playSample(new File("E:\\McGill\\ECSE 211\\Labs\\Project\\CaptureTheFlag\\battle.wav"));
		//Sound.playSample(new File("battle.wav"), 100	);
		 

		// ====== Do ultrasonic localization in corner ======  //
		usLocalizer.usLocalize();

		// ====== Do initial light localization in corner ======  //
		lightLocalizer.initialLightLocalize(wifi.getStartingCorner(wifi.getTeam()), PLAY_ZONE);

		
	
		//odometer.setXYT(7 * TILE_SIZE, 1 * TILE_SIZE, 0);

		//rc.travelTo(1, 1, FORWARD_SPEED, false);
		
		//Sound.beepSequence();

		if (team == Team.GREEN) {
			// ====== Travel to the tunnel ====== //
			navigator.travelToTunnel();
		} else if (team == Team.RED){
			// ====== Travel to the bridge ====== //
			navigator.travelToBridge();
		}

		if (team == Team.GREEN) {
			// ====== Travel through the tunnel ====== //
			navigator.travelThroughTunnel();
		} else if (team == Team.RED){
			// ====== Travel through the bridge ====== //
			navigator.travelThroughBridge();
		}
	

		//odometer.setXYT(7 * TILE_SIZE, 1 * TILE_SIZE, 0);

		
		// ====== Travel to the search zone ====== //
		navigator.travelToSearchZone();

		// ====== Search for the flag ====== //
		flagSearcher.searchFlag(wifi.getFlagColor());
		Sound.beepSequenceUp();

		if (team == Team.GREEN) {
			// ====== Travel to the bridge ====== //
			navigator.travelToBridge();
		} else if (team == Team.RED){
			// ====== Travel to the tunnel ====== //
			navigator.travelToTunnel();
		}


		if (team == Team.GREEN) {
			// ====== Travel through the bridge ====== //
			navigator.travelThroughBridge();
		} else if (team == Team.RED){
			// ====== Travel through the tunnel ====== //
			navigator.travelThroughTunnel();
		}

		// ====== Returning to starting corner ====== //
		navigator.returnToStart();
		
		
		Sound.twoBeeps();
		System.exit(0);
	}
}
