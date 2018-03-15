package ca.mcgill.ecse211.controller;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * Controls all aspects of a light sensor
 * 
 * @author Bijan Sadeghi
 */
public class LightSensorController {

	// Light sensor
	private EV3ColorSensor colorSensor;
	private SensorMode color;
	private float[] colorSample;

	// Block colors enumeration
	private enum BlockColors {
		RED, BLUE, YELLOW, WHITE, UNKNOWN
	}

	public LightSensorController(EV3ColorSensor colorSensor, SensorMode color, float[] colorSample) {
		this.colorSensor = colorSensor;
		this.color = color;
		this.colorSample = colorSample;
	}

	/**
	 * @return the RGB of the color read by the light sensor
	 */
	public float[] getColorSample() {
		color.fetchSample(colorSample, 0);
		return colorSample;
	}

	/**
	 * Calculates the color of the block based on red proportion
	 * 
	 * @param RGBSample: Sample of the block in front of the color sensor
	 * @return The color of the detected block
	 */
	static BlockColors getBlockColor(float[] RGBSample) {
		double redProportion = RGBSample[0] / (RGBSample[0] + RGBSample[1] + RGBSample[2]);

		if (redProportion >= 0 && redProportion <= 0.2) {
			return BlockColors.BLUE;
		} else if (redProportion >= 0.3 && redProportion <= 0.4) {
			return BlockColors.WHITE;
		} else if (redProportion >= 0.45 && redProportion <= 0.60) {
			return BlockColors.YELLOW;
		} else if (redProportion >= 0.7 && redProportion <= 1.0) {
			return BlockColors.RED;
		} else {
			return BlockColors.UNKNOWN;
		}
	}
}
