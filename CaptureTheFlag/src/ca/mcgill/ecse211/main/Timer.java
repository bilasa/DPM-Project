package ca.mcgill.ecse211.main;

/**
 * Keeps track of the time since the beginning of the challenge
 * and cancels the flagSearch if the time elapsed is too long
 * 
 * @author Bijan Sadeghi
 */
public class Timer extends Thread {
	private long currentTime;
	
	public Timer() {
		this.currentTime = System.currentTimeMillis();
	}

	@Override
	public void run() {
		
	}

}