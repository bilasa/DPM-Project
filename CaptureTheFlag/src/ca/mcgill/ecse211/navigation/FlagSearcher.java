package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.WiFi;
import lejos.hardware.Wifi;

/**
 * Includes all flag searching tasks of the robot
 * 
 * @author Bijan Sadeghi & Esa Khan
 */
public class FlagSearcher {
	// WiFi class
	private WiFi wifi;
	
	// Robot controller
	private RobotController rc;
	
	// Corner of the search zone closest to the tunnel/bridge exit
	private int[] closestSearchCorner;

	public FlagSearcher(WiFi wifi, RobotController rc) {
		this.wifi = wifi;
		this.rc = rc;
		this.closestSearchCorner = getClosestSearchCorner();
	}

	/**
	 * Travel to the closest corner of the search zone
	 */
	public void travelToSearchZone() {
		rc.travelTo(closestSearchCorner[0], closestSearchCorner[1], rc.FORWARD_SPEED, true);
	}

	/**
	 * Search for the flag in the search zone
	 */
	public void searchFlag() {
		// TODO Auto-generated method stub

	}

	/**
	 * @return the corner of the search zone closest to the tunnel/bridge exit
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
