package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.controller.UltrasonicSensorController;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * Localizes the robot using the ultrasonic sensor,
 * setting the robot's absolute heading to 0 degrees.
 * 
 * @author Bijan Sadeghi & Esa Khan
 *
 */
public class UltrasonicLocalizer {

	// Constants
	private final int ROTATE_SPEED;
	private final int DISTANCE_TO_WALL = 20;

	// Robot controller
	private RobotController rc;

	// Ultrasonic sensor controller
	private UltrasonicSensorController usCont;

	// Odometer
	private Odometer odo;

	// Variables for localization
	static double deltaT;
	static int counter = 0;
	static double firstTheta = 0;
	static double secondTheta = 0;

	public UltrasonicLocalizer(RobotController rc, UltrasonicSensorController usCont) {
		this.ROTATE_SPEED = rc.ROTATE_SPEED;
		this.rc = rc;
		this.usCont = usCont;
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Localizes the robot with the ultrasonic sensor and sets
	 * the robot's absolute heading to 0 degrees.
	 */
	public void usLocalize() {

		//Choose when not facing a wall at first 

		//Turn clockwise
		rc.rotate(true, ROTATE_SPEED);
		int prevAvgDistance = usCont.getAvgUSDistance();
		int deltaDistance;

		// If starting in front of wall
		while(usCont.getAvgUSDistance() < DISTANCE_TO_WALL) {
			// Keep moving
		}

		// Check for wall 1
		while(rc.isMoving()) {
			int currAvgDistance = usCont.getAvgUSDistance();

			deltaDistance = currAvgDistance - prevAvgDistance;
			if(deltaDistance < 0 && currAvgDistance < DISTANCE_TO_WALL) {
				rc.setSpeeds(0,0);
				firstTheta = odo.getXYT()[2];
			}
		}

		Sound.playTone(440, 250);
		Sound.playTone(520, 250);


		// Turn the other way (counterclockwise)
		rc.rotate(false, ROTATE_SPEED);

		while(usCont.getAvgUSDistance() < DISTANCE_TO_WALL) {
			// Keep moving
		}

		// Check for wall 2
		while(rc.isMoving()) {
			int currAvgDistance = usCont.getAvgUSDistance();

			deltaDistance = currAvgDistance - prevAvgDistance;
			if(deltaDistance < 0 && currAvgDistance < DISTANCE_TO_WALL) {
				rc.setSpeeds(0,0);
				secondTheta = odo.getXYT()[2];

			}
		}

		Sound.playTone(440, 250);
		Sound.playTone(520, 250);

		//Compute angle
		deltaT = secondTheta < firstTheta ? 45 - ((secondTheta + firstTheta)/2) : 
			225 - ((secondTheta + firstTheta)/2);

		//Set theta to the real angle
		odo.setTheta(odo.getXYT()[2] + deltaT);

		rc.turnTo(0);

		rc.setSpeeds(0, 0);
	}

}