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
 * Main class to be run
 * 
 * @author Bijan Sadeghi & Esa Khan
 */
public class CaptureTheFlag {

	// Motors
	private final static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private final static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private final static EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

	// Ultrasonic sensor
	private final static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S1);
	private static SampleProvider usDistance = usSensor.getMode("Distance");
	private static SampleProvider average = new MeanFilter(usDistance, 8);
	private static float[] usSample = new float[average.sampleSize()];

	// Front light sensor
	private final EV3ColorSensor frontColorSensor = new EV3ColorSensor(SensorPort.S4);
	private SensorMode frontRGBColor = frontColorSensor.getRGBMode();
	private float[] frontRGBColorSample = new float[frontRGBColor.sampleSize()];

	// Left rear light sensor
	private final static EV3ColorSensor leftRearColorSensor = new EV3ColorSensor(SensorPort.S2);
	private static SensorMode leftRearColorID = leftRearColorSensor.getColorIDMode();
	private static float[] leftRearColorIDSample = new float[leftRearColorID.sampleSize()];

	// Right rear light sensor
	private final static EV3ColorSensor rightRearColorSensor = new EV3ColorSensor(SensorPort.S3);
	private static SensorMode rightRearColorID = rightRearColorSensor.getColorIDMode();
	private static float[] rightRearColorIDSample = new float[rightRearColorID.sampleSize()];

	// LCD
	private final static TextLCD LCD = LocalEV3.get().getTextLCD();

	// Constants
	private final static double WHEEL_RAD = 1.66;
	private final static double TRACK = 17.95; // original 17.7
	private final static int ROTATE_SPEED = 250;
	private final static int FORWARD_SPEED = 250;
	private final static int ACCELERATION = 2000;
	private final static double TILE_SIZE = 30.48;
	private final static double SENSOR_DIST = 12.5;

	// Playzone constants
	private static final int LL_PZx = 1;
	private static final int LL_PZy = 1;
	private static final int UR_PZx = 7;
	private static final int UR_PZy = 7;
	private static final int[][] PLAY_ZONE = new int[][] { { LL_PZx, LL_PZy }, // Lower left
		{ UR_PZx, LL_PZy }, // Lower right
		{ UR_PZx, UR_PZy }, // Upper right
		{ LL_PZx, UR_PZy } // Upper left
	};

	// Odometer
	private final static Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

	// WiFi class
	private static WiFi wifi = new WiFi();

	// Controllers
	private static RobotController rc = new RobotController(leftMotor, rightMotor, WHEEL_RAD, TRACK, FORWARD_SPEED, ROTATE_SPEED, ACCELERATION, TILE_SIZE, SENSOR_DIST);
	private static UltrasonicSensorController usCont = new UltrasonicSensorController(usSensor, usDistance, average, usSample);
	private LightSensorController frontLsCont = new LightSensorController(frontColorSensor, frontRGBColor, frontRGBColorSample);
	private static LightSensorController leftRearLsCont = new LightSensorController(leftRearColorSensor, leftRearColorID, leftRearColorIDSample);
	private static LightSensorController rightRearLsCont = new LightSensorController(rightRearColorSensor, rightRearColorID, rightRearColorIDSample);

	// Navigation classes
	private static UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(rc, usCont);
	private static LightLocalizer lightLocalizer = new LightLocalizer(TILE_SIZE, SENSOR_DIST, rc, leftRearLsCont);
	private static Navigator navigator = new Navigator(rc, wifi);
	private static FlagSearcher flagSearcher = new FlagSearcher(wifi, rc);

	// Threads
	private static ExitProgram exit = new ExitProgram();
	private static Timer timer = new Timer();

	// Odometry correction
	private static OdometryCorrection odoCorrection = new OdometryCorrection(TILE_SIZE, SENSOR_DIST, rc, leftRearLsCont, rightRearLsCont);

	/**
	 * Localizes the robot at its corner.
	 * Navigates the robot through the tunnel/bridge.
	 * Searches for the flag in the opponent's search zone.
	 * Navigates the robot through the bridge/tunnel.
	 * Returns the robot to its starting corner.
	 * 
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {
		//rc.setSpeeds(200, 200);
		//rc.turnBy(720, true);

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

		// Set odometry correction for the RobotController
		//Thread odoCorrectionThread = new Thread(odoCorrection);
		//odoCorrectionThread.start();
		rc.setOdoCorrection(odoCorrection);
		navigator.setOdoCorrection(odoCorrection);

		/*rc.travelTo(1, 2, FORWARD_SPEED, true);
		lightLocalizer.generalLightLocalize();
		rc.turnTo(90);
		rc.travelDist(TILE_SIZE / 2, true);
		rc.turnTo(0);
		rc.travelDist(4 * TILE_SIZE, true);
		rc.turnTo(90);
		rc.travelDist(TILE_SIZE / 2, true);
		lightLocalizer.generalLightLocalize();
		rc.turnTo(180);
		rc.travelDist(TILE_SIZE / 2, true);
		rc.turnTo(90);
		rc.travelDist(4 * TILE_SIZE, true);
		rc.travelTo(7, 7, FORWARD_SPEED, true);
		lightLocalizer.generalLightLocalize();*/


		// ====== Get the robot's team ======  //
		Team team = wifi.getTeam();

		// ====== Do ultrasonic localization in corner ======  //
		usLocalizer.usLocalize();
//
//		// ====== Do initial light localization in corner ======  //
		lightLocalizer.initialLightLocalize(wifi.getStartingCorner(wifi.getTeam()), PLAY_ZONE);

		//odometer.setXYT(7 * TILE_SIZE, 1 * TILE_SIZE, 270);

//		rc.travelTo(3, 1, FORWARD_SPEED, true);
//		Sound.beepSequence();
//		rc.travelTo(1, 1, FORWARD_SPEED, true);
		
		
		if (team == Team.GREEN) {
			// ====== Travel to the tunnel ====== //
			navigator.travelToTunnel();
		} else if (team == Team.RED){
			// ====== Travel to the bridge ====== //
			navigator.travelToBridge();
		}

		// ====== Localize at the tunnel/bridge entrance ====== //
		//lightLocalizer.generalLightLocalize();

		if (team == Team.GREEN) {
			// ====== Travel through the tunnel ====== //
			navigator.travelThroughTunnel();
		} else if (team == Team.RED){
			// ====== Travel through the bridge ====== //
			navigator.travelThroughBridge();
		}

		// ====== Localize at the tunnel/bridge end ====== //
		//lightLocalizer.generalLightLocalize();

		// ====== Travel to the search zone ====== //
		//flagSearcher.travelToSearchZone();

		// ====== Localize before starting search ====== //
		//lightLocalizer.generalLightLocalize();

		// ====== Search for the flag ====== //
		//flagSearcher.searchFlag();

		// ====== Localize after search ====== //
		//lightLocalizer.generalLightLocalize();

		if (team == Team.GREEN) {
			// ====== Travel to the bridge ====== //
			navigator.travelToBridge();
		} else if (team == Team.RED){
			// ====== Travel to the tunnel ====== //
			navigator.travelToTunnel();
		}

		// ====== Localize before crossing the bridge/tunnel ====== //
		//lightLocalizer.generalLightLocalize();

		if (team == Team.GREEN) {
			// ====== Travel through the bridge ====== //
			navigator.travelThroughBridge();
		} else if (team == Team.RED){
			// ====== Travel through the tunnel ====== //
			navigator.travelThroughTunnel();
		}

		// ====== Localize after crossing the bridge/tunnel ====== //
		//lightLocalizer.generalLightLocalize();

		// ====== Returning to starting corner ====== //
		navigator.returnToStart();
	}
}
