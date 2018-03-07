package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Flag;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.WiFi;

public class Navigator {

	// *** Hardcoded Wifi variables (to remove) ***
	private int greenStartingCornerX;
	private int tn_ll_x = 3;
	private int tn_ll_y = 5;
	private int tn_ur_x = 4;
	private int tn_ur_y = 7;

	// Robot controller
	private RobotController rc;

	// Constants
	private final int FORWARD_SPEED;
	
	// WiFi class
	private WiFi wifi;

	public Navigator(RobotController rc, WiFi wifi) {
		this.FORWARD_SPEED = rc.FORWARD_SPEED;
		this.rc = rc;
		this.wifi = wifi;
	}

	/**
	 * Travel to the entrance of the tunnel
	 * 
	 * Assumptions before calling:
	 * 		If robot is on green team, robot is localized at its starting corner
	 * 		If robot is on red team, robot is at the search zone point closest to the tunnel's entrance
	 * 		
	 */
	public void travelToTunnel() {
		// ====== Select the team ====== //
		switch (wifi.getTeam()) {
		case GREEN:
			// ===============================================================// 
			// Select the starting corner of the green zone  				  // 
			// Then travel to closest point next to the tunnel in an L-shape  //
			// ===============================================================// 
			switch(wifi.getStartingCorner(wifi.getTeam())) {
			case 0:
				rc.travelTo(greenStartingCornerX, tn_ll_y - 1, FORWARD_SPEED, true);
				rc.travelTo(tn_ur_x, tn_ll_y - 1, FORWARD_SPEED, true);
				break;
			case 1:
				rc.travelTo(greenStartingCornerX, tn_ll_y - 1, FORWARD_SPEED, true);
				rc.travelTo(tn_ll_x, tn_ll_y - 1, FORWARD_SPEED, true);
				break;
			case 2:
				rc.travelTo(greenStartingCornerX, tn_ur_y + 1, FORWARD_SPEED, true);
				rc.travelTo(tn_ur_x, tn_ur_y + 1, FORWARD_SPEED, true);
				break;
			case 3:
				rc.travelTo(greenStartingCornerX, tn_ur_y + 1, FORWARD_SPEED, true);
				rc.travelTo(tn_ur_x - 1, tn_ur_y + 1, FORWARD_SPEED, true);
				break;
			}
			break;
		case RED:
			// ==========================================================// 
			// Select the starting corner   						     // 
			// Then directly travel to closest point next to the tunnel  //
			// ==========================================================// 
			
			break;
		}
	}

	/**
	 * Travel to the entrance of the bridge
	 */
	public void travelToBridge() {

	}

	/**
	 * Travel through the tunnel
	 */
	public void travelThroughTunnel() {
		
	}

	/**
	 * Travel through the bridge
	 */
	public void travelThroughBridge() {
		// TODO Auto-generated method stub
		
	}

	/**
	 * Return to the starting corner
	 */
	public void returnToStart() {
		// TODO Auto-generated method stub
		
	}


}
