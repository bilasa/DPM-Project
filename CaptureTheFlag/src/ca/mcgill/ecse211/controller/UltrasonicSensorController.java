package ca.mcgill.ecse211.controller;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MeanFilter;

/**
 * Controls all aspects of an ultrasonic sensor
 * 
 * @author Bijan Sadeghi
 */
public class UltrasonicSensorController {

	// Ultrasonic sensor
	private EV3UltrasonicSensor usSensor;
	private SampleProvider usDistance;
	private SampleProvider average;
	private float[] usSample;
	
	public UltrasonicSensorController(EV3UltrasonicSensor usSensor, SampleProvider usDistance, SampleProvider average, float[] usSample) {
		this.usSensor = usSensor;
		this.usDistance = usDistance;
		this.average = average;
		this.usSample = usSample;
	}
	
	/**
	 * Returns the ultrasonic sensor distance using a mean filter
	 * 
	 * @return
	 */
	public int getAvgUSDistance() {
		average.fetchSample(usSample, 0);
		return (int) (usSample[0] * 100.0);
	}
}
