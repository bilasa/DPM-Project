package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.WiFi;
import lejos.hardware.Wifi;

/**
 * This class includes all flag searching tasks of the robot.
 * The class allows the navigation of the robot to the search
 * zone, as well as the execution of the search algorithm.
 * The FlagSearcher makes the robot search for its target block
 * by going on the perimeter of the search zone. If the target
 * block is found, the robot ends its search.
 * 
 * @author Bijan Sadeghi
 * @author Esa Khan
 */
public class FlagSearcher {
	// WiFi class
	private WiFi wifi;
	
	// Robot controller
	private RobotController rc;
	
	// Corner of the search zone closest to the tunnel/bridge exit
	private int[] closestSearchCorner;

	/**
	 * @param wifi the wifi object to get the challenge data from
	 * @param rc the robot controller to use
	 */
	public FlagSearcher(WiFi wifi, RobotController rc) {
		this.wifi = wifi;
		this.rc = rc;
		this.closestSearchCorner = getClosestSearchCorner();
	}

	/**
	 * Travels to the corner of the search zone closest to the robot
	 * after it has crossed the bridge/tunnel into the opponent
	 * team's zone.
	 */
	public void travelToSearchZone() {
		rc.travelTo(closestSearchCorner[0], closestSearchCorner[1], rc.FORWARD_SPEED, true);
	}

	/**
	 * Searches for the flag in the search zone. Navigates on the rectangular 
	 * perimeter of the search zone with the ultrasonic sensor facing the
	 * interior of the search zone. Continuously checks for falling edge signals,
	 * which would indicate the presence of a block. When a block is detected,
	 * the robot turns towards the interior of the search zone, approaches the
	 * block to a given threshold distance, and identifies the color of the block.
	 * If the block is the target, it beeps twice and ends its search. Otherwise,
	 * it backs up to the perimeter and continues its search.
	 * 
	 */
	public void searchFlag() {
		// TODO Auto-generated method stub
	}

	/**
	 * Gets the corner of the search zone closest to the robot after crossing
	 * the tunnel/bridge into the opponent's zone.
	 * 
	 * @return the corner of the search zone closest to the robot after it has crossed
	 */
	private int[] getClosestSearchCorner() {
		Team opponentTeam = wifi.getTeam();
		if (wifi.getTeam() == Team.GREEN) {
			opponentTeam = Team.RED;
		}else if(wifi.getTeam() == Team.RED){
			opponentTeam = Team.GREEN;
		}

		switch(wifi.getStartingCorner(opponentTeam)) {
		case 0:
			return wifi.getSearchZone(opponentTeam)[2];
		case 1:
			return wifi.getSearchZone(opponentTeam)[3];
		case 2:
			return wifi.getSearchZone(opponentTeam)[0];
		case 3:
			return wifi.getSearchZone(opponentTeam)[1];
		}

		return null;
	}

}
