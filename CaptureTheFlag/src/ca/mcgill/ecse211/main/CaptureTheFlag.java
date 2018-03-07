package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.enumeration.Flag;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.navigation.FlagSearcher;
import ca.mcgill.ecse211.navigation.LightLocalizer;
import ca.mcgill.ecse211.navigation.Navigator;
import ca.mcgill.ecse211.navigation.UltrasonicLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Button;
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

public class CaptureTheFlag {

	// Motors
	private final static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private final static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private final static EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

	// Ultrasonic sensor
	private final static EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
	private static SampleProvider usDistance = usSensor.getMode("Distance");
	private static SampleProvider average = new MeanFilter(usDistance, 8);
	private static float[] usSample = new float[average.sampleSize()];

	// Front light sensor
	private final EV3ColorSensor frontColorSensor = new EV3ColorSensor(SensorPort.S4);
	private SensorMode frontRGBColor = frontColorSensor.getRGBMode();
	private float[] frontRGBColorSample = new float[frontRGBColor.sampleSize()];

	// Rear light sensor
	private final static EV3ColorSensor rearColorSensor = new EV3ColorSensor(SensorPort.S1);
	private static SensorMode rearColorID = rearColorSensor.getColorIDMode();
	private static float[] rearColorIDSample = new float[rearColorID.sampleSize()];

	// LCD
	private final TextLCD LCD = LocalEV3.get().getTextLCD();

	// Constants
	private final static double WHEEL_RAD = 1.67;
	private final static double TRACK = 20.9;
	private final static int ROTATE_SPEED = 150;
	private final static int FORWARD_SPEED = 250;
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
	private static RobotController rc = new RobotController(leftMotor, rightMotor, WHEEL_RAD, TRACK, FORWARD_SPEED, ROTATE_SPEED, TILE_SIZE);
	private static UltrasonicSensorController usCont = new UltrasonicSensorController(usSensor, usDistance, average, usSample);
	private LightSensorController frontLsCont = new LightSensorController(frontColorSensor, frontRGBColor, frontRGBColorSample);
	private static LightSensorController rearLsCont = new LightSensorController(rearColorSensor, rearColorID, rearColorIDSample);

	// Navigation classes
	private static UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(rc, usCont);
	private static LightLocalizer lightLocalizer = new LightLocalizer(TILE_SIZE, SENSOR_DIST, rc, rearLsCont);
	private static Navigator navigator = new Navigator(rc, wifi);
	private static FlagSearcher flagSearcher = new FlagSearcher(wifi, rc);

	public static void main(String[] args) {
		// ====== Get the robot's team ======  //
		Team team = wifi.getTeam();

		// ====== Do initial light localization in corner ======  //
		lightLocalizer.initialLightLocalize(wifi.getStartingCorner(wifi.getTeam()), PLAY_ZONE);

		if (team == Team.GREEN) {
			// ====== Travel to the tunnel ====== //
			navigator.travelToTunnel();
		} else if (team == Team.RED){
			// ====== Travel to the bridge ====== //
			navigator.travelToBridge();
		}

		// ====== Localize at the tunnel/bridge entrance ====== //
		lightLocalizer.generalLightLocalize();

		if (team == Team.GREEN) {
			// ====== Travel through the tunnel ====== //
			navigator.travelThroughTunnel();
		} else if (team == Team.RED){
			// ====== Travel through the bridge ====== //
			navigator.travelThroughBridge();
		}

		// ====== Localize at the tunnel/bridge end ====== //
		lightLocalizer.generalLightLocalize();

		// ====== Travel to the search zone ====== //
		flagSearcher.travelToSearchZone();

		// ====== Localize before starting search ====== //
		lightLocalizer.generalLightLocalize();

		// ====== Search for the flag ====== //
		flagSearcher.searchFlag();

		// ====== Localize after search ====== //
		lightLocalizer.generalLightLocalize();

		if (team == Team.GREEN) {
			// ====== Travel to the bridge ====== //
			navigator.travelToBridge();
		} else if (team == Team.RED){
			// ====== Travel to the tunnel ====== //
			navigator.travelToTunnel();
		}

		// ====== Localize before crossing the bridge/tunnel ====== //
		lightLocalizer.generalLightLocalize();

		if (team == Team.GREEN) {
			// ====== Travel through the bridge ====== //
			navigator.travelThroughBridge();
		} else if (team == Team.RED){
			// ====== Travel through the tunnel ====== //
			navigator.travelThroughTunnel();
		}

		// ====== Localize after crossing the bridge/tunnel ====== //
		lightLocalizer.generalLightLocalize();

		// ====== Returning to starting corner ====== //
		navigator.returnToStart();

	}
}
