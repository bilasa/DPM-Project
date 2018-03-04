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
	private final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

	// Ultrasonic sensor
	private final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
	private SampleProvider usDistance = usSensor.getMode("Distance");
	private SampleProvider average = new MeanFilter(usDistance, 8);
	private float[] usSample = new float[average.sampleSize()];

	// Front light sensor
	private final EV3ColorSensor frontColorSensor = new EV3ColorSensor(SensorPort.S4);
	private SensorMode frontColorID = frontColorSensor.getColorIDMode();
	private float[] frontColorIDSample = new float[frontColorSensor.sampleSize()];

	// Rear light sensor
	private final EV3ColorSensor rearColorSensor = new EV3ColorSensor(SensorPort.S1);
	private SensorMode rearColorID = rearColorSensor.getColorIDMode();
	private float[] rearColorIDSample = new float[rearColorSensor.sampleSize()];

	// LCD
	private final TextLCD LCD = LocalEV3.get().getTextLCD();

	// Constants
	private final double WHEEL_RAD = 2.16;
	private final double TRACK = 12.00;
	private final int ROTATE_SPEED = 150;
	private final int FORWARD_SPEED = 250;
	private final double TILE_SIZE = 30.48;
	private double SENSOR_DIST = 12.5;
	
	// Odometer
	private final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);

	// Controllers
	private RobotController rc = new RobotController(leftMotor, rightMotor, WHEEL_RAD, TRACK, FORWARD_SPEED, ROTATE_SPEED, TILE_SIZE);
	private UltrasonicSensorController usCont = new UltrasonicSensorController(usSensor, usDistance, average, usSample);
	private LightSensorController frontLsCont = new LightSensorController(frontColorSensor, frontColorID, frontColorIDSample);
	private LightSensorController rearLsCont = new LightSensorController(rearColorSensor, rearColorID, rearColorIDSample);
	
	// Navigation classes
	private UltrasonicLocalizer usLocalizer = new UltrasonicLocalizer(ROTATE_SPEED, rc, usCont);
	private LightLocalizer lightLocalizer = new LightLocalizer(FORWARD_SPEED, ROTATE_SPEED, TILE_SIZE, SENSOR_DIST, rc, rearLsCont);
	private Navigator navigator = new Navigator();
	private FlagSearcher flagSearcher = new FlagSearcher();

	public static void main(String[] args) {

	}
}
