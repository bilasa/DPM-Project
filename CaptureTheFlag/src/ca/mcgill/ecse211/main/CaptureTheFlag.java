package ca.mcgill.ecse211.main;

import ca.mcgill.ecse211.controller.LightSensorController;
import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
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

	// Controllers
	private static RobotController rc = new RobotController(leftMotor, rightMotor, WHEEL_RAD, TRACK, FORWARD_SPEED, ROTATE_SPEED, TILE_SIZE);
	private static UltrasonicSensorController usCont = new UltrasonicSensorController(usSensor, usDistance, average, usSample);
	private LightSensorController frontLsCont = new LightSensorController(frontColorSensor, frontRGBColor, frontRGBColorSample);
	private static LightSensorController rearLsCont = new LightSensorController(rearColorSensor, rearColorID, rearColorIDSample);

	// Navigation classes
	private static UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(ROTATE_SPEED, rc, usCont);
	private static LightLocalizer lightLocalizer = new LightLocalizer(FORWARD_SPEED, ROTATE_SPEED, TILE_SIZE, SENSOR_DIST, rc, rearLsCont);
	private static Navigator navigator = new Navigator(FORWARD_SPEED, rc);
	private static FlagSearcher flagSearcher = new FlagSearcher();

	private enum Team {
		RED, GREEN
	}

	public static void main(String[] args) {
		int redTeam = 8; // Team starting out from red zone
		int greenTeam = 10; // Team starting out from green zone
		int redCorner = 3; // Starting corner for red team
		int greenCorner = 1; // Starting corner for green team
		int og = 2; // color of green opponent flag
		int or = 3; // color of red opponent flag
		int red_ll_x = 0; // lower left hand corner of Red Zone
		int red_ll_y = 7; 
		int red_ur_x = 8; // upper right hand corner of Red Zone
		int red_ur_y = 12;
		int green_ll_x = 4; // lower left hand corner of Green Zone
		int green_ll_y = 0;
		int green_ur_x = 12; // upper right hand corner of Green Zone
		int green_ur_y = 5;
		int tn_ll_x = 3; // lower left hand corner of the tunnel footprint
		int tn_ll_y = 5;
		int tn_ur_x = 4; // upper right hand corner of the tunnel footprint
		int tn_ur_y = 7;
		int br_ll_x = 7; // lower left hand corner of the bridge footprint
		int br_ll_y = 5;
		int br_ur_x = 8; // upper right hand corner of the bridge footprint
		int br_ur_y = 7;
		int sr_ll_x = 1; // lower left hand corner of search region in red player zone
		int sr_ll_y = 9;
		int sr_ur_x = 2; // upper right hand corner of search region in red player zone
		int sr_ur_y = 11;
		int sg_ll_x = 9; // lower left hand corner of search region in green player zone
		int sg_ll_y = 1;
		int sg_ur_x = 11; // upper right hand corner of search region in green player zone
		int sg_ur_y = 2;

		// Hardcoded WiFi variables
		int startingCorner = 1;
		Team team = Team.GREEN;

		// ====== Do initial light localization in corner ======  //
		lightLocalizer.initialLightLocalize(startingCorner, PLAY_ZONE);

		// ====== Select the team ====== //
		switch (team) {
		case GREEN:
			// ====== Travel to the tunnel ====== //
			navigator.travelToTunnel();

			// ====== Localize at the tunnel's entrance ====== //
			lightLocalizer.generalLightLocalize();

			break;
		case RED:
			// ====== Travel to the bridge ====== //
			navigator.travelToBridge();
			break;
		}
	}
}
