package ca.mcgill.ecse211.controller;

import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * Controls all aspects of a given EV3UltrasonicSensor.
 * This class is used by any class that needs access to
 * data read by an ultrasonic sensor. In particular, it is used
 * by UltrasonicLocalizer and FlagSearcher. The class can fetch the 
 * average distance read by the sensor.
 * 
 * @author Bijan Sadeghi
 * @author Guillaume Richard
 */
public class UltrasonicSensorController {

	// Ultrasonic sensor
	private EV3UltrasonicSensor usSensor;
	private SampleProvider usDistance;
	private SampleProvider average;
	private float[] usSample;
	
	// Motor to rotate sensor
	private EV3MediumRegulatedMotor sensorMotor;
	
	/**
	 * @param usSensor the ultrasonic sensor to use
	 * @param usDistance the sample provider to use for the distance
	 * @param average the sample provider to use for the average distance
	 * @param usSample the array into which samples are fetched
	 */
	public UltrasonicSensorController(EV3UltrasonicSensor usSensor, EV3MediumRegulatedMotor sensorMotor, SampleProvider usDistance, SampleProvider average, float[] usSample) {
		this.usSensor = usSensor;
		this.sensorMotor = sensorMotor;
		this.usDistance = usDistance;
		this.average = average;
		this.usSample = usSample;
	}
	
	/**
	 * Provides the distance read by the ultrasonic sensor using a mean filter.
	 * 
	 * @return the average distance fetched by the ultrasonic sensor
	 */
	public int getAvgUSDistance() {
		average.fetchSample(usSample, 0);
		return (int) (usSample[0] * 100.0);
	}
	
	public void rotateSensorTo(int theta) {
		sensorMotor.rotateTo(theta);
	}
}
