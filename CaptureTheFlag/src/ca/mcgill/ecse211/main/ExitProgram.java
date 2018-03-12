package ca.mcgill.ecse211.main;

import lejos.hardware.Button;

public class ExitProgram extends Thread {

	@Override
	public void run() {
		while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
		}
		System.exit(0);
	}

}