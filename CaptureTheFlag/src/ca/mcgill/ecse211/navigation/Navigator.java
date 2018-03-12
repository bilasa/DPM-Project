package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Flag;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.WiFi;

public class Navigator {

	// *** Hardcoded Wifi variables (to remove) ***
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

	// Starting corner coordinates of the robot
	private int[] startingCornerCoords;

	// Coordinates of tunnel
	private int[][] tunnelZone;

	// Coordinates of bridge
	private int[][] bridgeZone;

	public Navigator(RobotController rc, WiFi wifi) {
		this.FORWARD_SPEED = rc.FORWARD_SPEED;
		this.rc = rc;
		this.wifi = wifi;
		this.startingCornerCoords = wifi.getStartingCornerCoords();
		this.tunnelZone = wifi.getTunnelZone();
		this.bridgeZone = wifi.getBridgeZone();
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
		// Extract the tunnel coordinates
		int[] tunnelLL = tunnelZone[0];
		int[] tunnelUR = tunnelZone[2];

		// ====== Select the team ====== //
		switch (wifi.getTeam()) {
		case GREEN:
			// ===============================================================// 
			// Check if the tunnel has a horizontal or vertical orientation  // 
			// ===============================================================// 
			if (wifi.isCrossingVert()) {
				// ===============================================================// 
				// Select the starting corner of the green zone  				  // 
				// Then travel to closest point next to the tunnel in an L-shape  //
				// ===============================================================// 
				switch(wifi.getStartingCorner(Team.GREEN)) {
				case 0:
				case 1:
					rc.travelTo(startingCornerCoords[0], tunnelLL[1] - 1, FORWARD_SPEED, true);
					rc.travelTo(tunnelLL[0], tunnelLL[1] - 1, FORWARD_SPEED, true);
					break;
				case 2:
				case 3:
					rc.travelTo(startingCornerCoords[0], tunnelUR[1] + 1, FORWARD_SPEED, true);
					rc.travelTo(tunnelUR[0], tunnelUR[1] + 1, FORWARD_SPEED, true);
					break;
				}
			} else {
				// ===============================================================// 
				// Select the starting corner of the green zone  				  // 
				// Then travel to closest point next to the tunnel in an L-shape  //
				// ===============================================================// 
				switch(wifi.getStartingCorner(Team.GREEN)) {
				case 0:
				case 3:
					rc.travelTo(tunnelLL[0] - 1, startingCornerCoords[1], FORWARD_SPEED, true);
					rc.travelTo(tunnelLL[0] - 1, tunnelLL[1] + 1, FORWARD_SPEED, true);
					break;
				case 1:
				case 2:
					rc.travelTo(tunnelUR[0] + 1, startingCornerCoords[1], FORWARD_SPEED, true);
					rc.travelTo(tunnelUR[0] + 1, tunnelUR[1] - 1, FORWARD_SPEED, true);
					break;
				}
			}
			break;
		case RED:
			// ===============================================================// 
			// Check if the tunnel has a horizontal or vertical orientation  // 
			// ===============================================================// 
			if (wifi.isCrossingVert()) {
				// ==========================================================// 
				// Select the starting corner   						     // 
				// Then directly travel to closest point next to the tunnel  //
				// ==========================================================// 
				switch(wifi.getStartingCorner(Team.GREEN)) {
				case 0:
				case 1:
					rc.travelTo(tunnelLL[0], tunnelLL[1] - 1, FORWARD_SPEED, true);
					break;
				case 2:
				case 3:
					rc.travelTo(tunnelUR[0], tunnelUR[1] + 1, FORWARD_SPEED, true);
					break;
				}
			} else {
				// ==========================================================// 
				// Select the starting corner   						     // 
				// Then directly travel to closest point next to the tunnel  //
				// ==========================================================// 
				switch(wifi.getStartingCorner(Team.GREEN)) {
				case 0:
				case 1:
					rc.travelTo(tunnelLL[0] - 1, tunnelLL[1] + 1, FORWARD_SPEED, true);
					break;
				case 2:
				case 3:
					rc.travelTo(tunnelUR[0] + 1, tunnelUR[1] - 1, FORWARD_SPEED, true);
					break;
				}
			}
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
