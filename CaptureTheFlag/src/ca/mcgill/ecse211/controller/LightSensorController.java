package ca.mcgill.ecse211.controller;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

/**
 * This class allows the control of a given EV3ColorSensor.
 * This class is used by any class that needs access to
 * data read by a light sensor. In particular, it is used
 * by LightLocalizer, OdometryCorrection, and FlagSearcher.
 * The class can fetch the color sample of the light sensor
 * as well as provide the color of a block.
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

	/**
	 * @param colorSensor the color sensor to use
	 * @param color the sensor mode
	 * @param colorSample the array to fetch the samples into
	 */
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
	 * Calculates the color of the block based on red proportion.
	 * 
	 * @param RGBSample The RGB sample of the block in front of the color sensor
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
