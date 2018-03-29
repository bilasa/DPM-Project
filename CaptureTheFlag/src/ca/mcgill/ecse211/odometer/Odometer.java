package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class holds and updates the robot's x, y, and theta
 * relative to the values it is initially set to (initially
 * set during initial localization). The values are updated
 * using the left and right motor tachometer count, which
 * tracks how many degrees each motor has rotated by.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 * @author Guillaume Richard
 */
public class Odometer extends OdometerData implements Runnable {


	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int lastLeftMotorTachoCount;
	private int lastRightMotorTachoCount;
	private int leftMotorTachoCount;
	private int rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RAD;


	private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

	/**
	 * This is the default constructor of this class. It initiates all motors and variables once.It
	 * cannot be accessed externally.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @throws OdometerExceptions
	 */
	private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {


		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		//Reset tacho counts
		this.leftMotor.resetTachoCount();
		this.rightMotor.resetTachoCount();

		// Reset the values of x, y and t to 0
		setXYT(0, 0, 0);

		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
	 * @throws OdometerExceptions
	 */
	public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD) {
		if (odo != null) { // Return existing object
			return odo;
		} else { // create object and return it
			try {
				odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
			} catch (OdometerExceptions e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			return odo;
		}
	}

	/**
	 * This class is meant to return the existing Odometer Object. It is meant to be used only if an
	 * odometer object has been created
	 * 
	 * @return error if no previous odometer exists
	 */
	public synchronized static Odometer getOdometer() throws OdometerExceptions {

		if (odo == null) {
			throw new OdometerExceptions("No previous Odometer exits.");

		}
		return odo;
	}

	/**
	 * Continuously updates the odometer's (x, y, Theta) based on the
	 * tachometer count of the left and right motor.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
		double leftDist, rightDist, deltaDist, deltaTheta, dX, dY;

		//Initialize the variables lastLeftMotorTachoCount and lastRightMotorTachoCount
		lastLeftMotorTachoCount = leftMotor.getTachoCount();
		lastRightMotorTachoCount = rightMotor.getTachoCount();

		while (true) {
			updateStart = System.currentTimeMillis();

			leftMotorTachoCount = leftMotor.getTachoCount();			//get tacho counts
			rightMotorTachoCount = rightMotor.getTachoCount();

			// TODO Calculate new robot position based on tachometer counts

			leftDist = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastLeftMotorTachoCount)/180;
			rightDist = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastRightMotorTachoCount)/180;

			lastLeftMotorTachoCount = leftMotorTachoCount;
			lastRightMotorTachoCount = rightMotorTachoCount;

			deltaDist = 0.5 * (leftDist + rightDist);
			deltaTheta = (leftDist - rightDist) / TRACK;

			//Put the deltaTheta in degrees
			deltaTheta *= (180/Math.PI);

			//Get the value of theta
			double[] values = getXYT();
			double theta = values [2];

			//The sin & cos use radians value
			dX = deltaDist * Math.sin(Math.toRadians(theta + deltaTheta));
			dY = deltaDist * Math.cos(Math.toRadians(theta + deltaTheta));

			// TODO Update odometer values with new calculated values
			odo.update(dX, dY, deltaTheta);

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done
				}
			}
		}
	}

}