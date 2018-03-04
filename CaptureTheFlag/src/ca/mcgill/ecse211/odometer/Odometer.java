/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Rodrigo Silva
 * @author Dirk Dubois
 * @author Derek Yu
 * @author Karim El-Baba
 * @author Michael Smith
 */

package ca.mcgill.ecse211.odometer;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

	private OdometerData odoData;
	private static Odometer odo = null; // Returned as singleton

	// Motors and related variables
	private int leftTachoCount;
	private int rightTachoCount;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	private final double TRACK;
	private final double WHEEL_RAD;

	private double[] position;

	private boolean isPaused;

	// Synchronization lock
	private Object lock;


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
		odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
		// manipulation methods
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		// Reset the values of x, y and z to 0
		odoData.setXYT(0, 0, 0);

		this.leftTachoCount = 0;
		this.rightTachoCount = 0;

		this.TRACK = TRACK;
		this.WHEEL_RAD = WHEEL_RAD;

		this.isPaused = false;

		// Initialize the lock
		lock = new Object();

	}

	/**
	 * This method is meant to ensure only one instance of the odometer is used throughout the code.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @return new or existing Odometer Object
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
	 * This method is where the logic for the odometer will run. Use the methods provided from the
	 * OdometerData class to implement the odometer.
	 */
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;


		// Variables to store the change in tacho count between iterations
		int deltaLeftTacho;
		int deltaRightTacho;

		// Variables to store the arc lengths of the wheels
		double leftArcLength;
		double rightArcLength;

		// Variables to store the tacho counts of the previous iteration
		int oldLeftTachoCount = 0;
		int oldRightTachoCount = 0;

		// Change in theta
		double deltaTheta;

		// The arc length corresponding to the center of the robot
		double centerArcLength;

		while (true) {
			updateStart = System.currentTimeMillis();

			leftTachoCount = leftMotor.getTachoCount();
			rightTachoCount = rightMotor.getTachoCount();

			// Calculate the change in the tacho count of each motor (degrees)
			deltaLeftTacho = leftTachoCount - oldLeftTachoCount;
			deltaRightTacho = rightTachoCount - oldRightTachoCount;

			// Update the old tacho count values
			oldLeftTachoCount = leftTachoCount;
			oldRightTachoCount = rightTachoCount;

			// Calculate the arc lengths of the wheels (first convert degrees of delta tacho to radians)
			leftArcLength = (Math.PI / 180.0) * deltaLeftTacho * WHEEL_RAD;
			rightArcLength = (Math.PI / 180.0) * deltaRightTacho * WHEEL_RAD;

			// Calculate the change in theta (convert back to degrees)
			deltaTheta = (180.0 / Math.PI) * (rightArcLength - leftArcLength) / TRACK;

			// Calculate the arc length for the center of the robot (average of left and right arc lengths)
			centerArcLength = (leftArcLength + rightArcLength) / 2.0;

			// Put in lock so that x, y, and theta are not modified in other threads
			synchronized (lock) {

				// Calculate deltaX and deltaY
				double theta = odo.getXYT()[2];

				// Compute deltaX and deltaY (convert to radians and shift by 90 degrees)
				double deltaX = centerArcLength*Math.cos((Math.PI / 180.0) * (theta + deltaTheta / 2.0) + (Math.PI / 2.0));
				double deltaY = centerArcLength*Math.sin((Math.PI / 180.0) * (theta + deltaTheta / 2.0) + (Math.PI / 2.0));

				//update x, y, and theta if the odometer is not paused
				if (!isPaused) {
					odo.update(deltaX, deltaY, deltaTheta);
				}

			}

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

	public void setPaused(boolean paused){
		isPaused = paused;
	}

}
