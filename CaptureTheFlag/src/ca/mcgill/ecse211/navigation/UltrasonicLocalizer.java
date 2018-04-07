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
 * This class allows the robot to be localized using the ultrasonic sensor. The
 * class localizes the robot's angle by rotating and detecting both walls, and
 * averaging the angle at which both walls were detected.
 * 
 * @author Bijan Sadeghi
 * @author Esa Khan
 * @author Guillaume Richard
 * @author Olivier Therrien
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

	/**
	 * @param rc
	 *            the robot controller to use
	 * @param usCont
	 *            the ultrasonic sensor controller to use
	 */
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
	 * Localizes the robot with the ultrasonic sensor using falling edge wall
	 * detection. Rotates the robot until the difference in distance is negative
	 * (representing a falling edge) and the distance equals a threshold, and stops,
	 * recording the angle. Then rotates the robot in the opposite direction until
	 * the falling edge is detected on the other side, recording that angle. Uses
	 * the two angles to fix the odometer's theta and set the robot's theta to 0
	 * degrees.
	 */
	public void usLocalize() {

		// Choose when not facing a wall at first

		// If starting in front of wall
		while (usCont.getAvgUSDistance() < DISTANCE_TO_WALL) {
			// Rotate 90 degrees
			rc.turnBy(90, true);
		}

		// Turn clockwise
		rc.rotate(true, ROTATE_SPEED + 150);
		int prevAvgDistance = usCont.getAvgUSDistance();
		int deltaDistance;

		// Check for wall 1
		while (rc.isMoving()) {
			int currAvgDistance = usCont.getAvgUSDistance();

			deltaDistance = currAvgDistance - prevAvgDistance;
			if (deltaDistance < 0 && currAvgDistance < DISTANCE_TO_WALL) {
				rc.setSpeeds(0, 0);
				firstTheta = odo.getXYT()[2];
			}
		}

		Sound.playTone(440, 250);
		Sound.playTone(520, 250);

		// Turn the other way (counterclockwise)
		rc.rotate(false, ROTATE_SPEED + 150);

		// Sleep for 2 seconds
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		while (usCont.getAvgUSDistance() < DISTANCE_TO_WALL) {
			// Keep moving
		}

		// Check for wall 2
		while (rc.isMoving()) {
			int currAvgDistance = usCont.getAvgUSDistance();

			deltaDistance = currAvgDistance - prevAvgDistance;
			if (deltaDistance < 0 && currAvgDistance < DISTANCE_TO_WALL) {
				rc.setSpeeds(0, 0);
				secondTheta = odo.getXYT()[2];

			}
		}

		Sound.playTone(440, 250);
		Sound.playTone(520, 250);

		// Compute angle
		deltaT = secondTheta < firstTheta ? 45 - ((secondTheta + firstTheta) / 2)
				: 225 - ((secondTheta + firstTheta) / 2);

		// Set theta to the real angle
		odo.setTheta(odo.getXYT()[2] + deltaT);

		rc.turnTo(0);

		rc.setSpeeds(0, 0);
	}

}