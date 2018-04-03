package ca.mcgill.ecse211.main;

/**
 * This class keeps track of the time since the beginning of the challenge.
 * The class will cancels the flagSearch if the time elapsed has
 * surpassed a given threshold.
 * 
 * @author Bijan Sadeghi
 */
public class Timer extends Thread {
	// The current time 
	private long currentTime;
	
	public Timer() {
		this.currentTime = System.currentTimeMillis();
	}

	@Override
	public void run() {
		
	}

}