package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Flag;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.WiFi;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

/**
 * Allows the navigation of the robot through various parts of the playzone
 * 
 * @author Bijan Sadeghi & Esa Khan
 */
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

	// Odometer
	private Odometer odo;

	public Navigator(RobotController rc, WiFi wifi) {
		this.FORWARD_SPEED = rc.FORWARD_SPEED;
		this.rc = rc;
		this.wifi = wifi;
		this.startingCornerCoords = wifi.getStartingCornerCoords();
		this.tunnelZone = wifi.getTunnelZone();
		this.bridgeZone = wifi.getBridgeZone();
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Travel to the entrance of the tunnel.
	 * 
	 * Assumptions before calling:
	 * (1) If robot is on green team, robot is localized at its starting corner,
	 * (2) If robot is on red team, robot is at the search zone point closest to the tunnel's entrance.
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
			// Check if the tunnel has a horizontal or vertical orientation   // 
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
		// Extract the bridge coordinates
		int[] bridgeLL = bridgeZone[0];
		int[] bridgeUR = bridgeZone[2];

		// ====== Select the team ====== //
		switch (wifi.getTeam()) {
		case RED:
			// ===============================================================// 
			// Check if the bridge has a horizontal or vertical orientation   // 
			// ===============================================================// 
			if (wifi.isCrossingVert()) {
				// ===============================================================// 
				// Select the starting corner of the red zone  		   		      // 
				// Then travel to closest point next to the bridge in an L-shape  //
				// ===============================================================// 
				switch(wifi.getStartingCorner(Team.RED)) {
				case 0:
				case 1:
					rc.travelTo(startingCornerCoords[0], bridgeLL[1] - 1, FORWARD_SPEED, true);
					rc.travelTo(bridgeLL[0], bridgeLL[1] - 1, FORWARD_SPEED, true);
					break;
				case 2:
				case 3:
					rc.travelTo(startingCornerCoords[0], bridgeUR[1] + 1, FORWARD_SPEED, true);
					rc.travelTo(bridgeUR[0], bridgeUR[1] + 1, FORWARD_SPEED, true);
					break;
				}
			} else {
				// ===============================================================// 
				// Select the starting corner of the red zone  				      // 
				// Then travel to closest point next to the bridge in an L-shape  //
				// ===============================================================// 
				switch(wifi.getStartingCorner(Team.RED)) {
				case 0:
				case 3:
					rc.travelTo(bridgeLL[0] - 1, startingCornerCoords[1], FORWARD_SPEED, true);
					rc.travelTo(bridgeLL[0] - 1, bridgeLL[1] + 1, FORWARD_SPEED, true);
					break;
				case 1:
				case 2:
					rc.travelTo(bridgeUR[0] + 1, startingCornerCoords[1], FORWARD_SPEED, true);
					rc.travelTo(bridgeUR[0] + 1, bridgeUR[1] - 1, FORWARD_SPEED, true);
					break;
				}
			}
			break;
		case GREEN:
			// ===============================================================// 
			// Check if the bridge has a horizontal or vertical orientation  // 
			// ===============================================================// 
			if (wifi.isCrossingVert()) {
				// ==========================================================// 
				// Select the starting corner   						     // 
				// Then directly travel to closest point next to the bridge  //
				// ==========================================================// 
				switch(wifi.getStartingCorner(Team.GREEN)) {
				case 0:
				case 1:
					rc.travelTo(bridgeLL[0], bridgeLL[1] - 1, FORWARD_SPEED, true);
					break;
				case 2:
				case 3:
					rc.travelTo(bridgeUR[0], bridgeUR[1] + 1, FORWARD_SPEED, true);
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
					rc.travelTo(bridgeLL[0] - 1, bridgeLL[1] + 1, FORWARD_SPEED, true);
					break;
				case 2:
				case 3:
					rc.travelTo(bridgeUR[0] + 1, bridgeUR[1] - 1, FORWARD_SPEED, true);
					break;
				}
			}
			break;
		}
	}

	/**
	 * Travel through the tunnel
	 */
	public void travelThroughTunnel() {
		// Turn towards the tunnel
		turnToCrossing(tunnelZone);

		// Turn clockwise by 90 degrees
		rc.turnBy(90, true);

		// Travel forward by half a tile
		rc.travelDist(rc.TILE_SIZE / 2, true);

		// Turn counterclockwise by 90 degrees
		rc.turnBy(-90, true);

		// Travel through the tunnel/bridge by moving forward by 4 tiles
		rc.travelDist(4 * rc.TILE_SIZE, true);

		// Turn clockwise by 90 degrees
		rc.turnBy(90, true);

		// Travel forward by half a tile
		rc.travelDist(rc.TILE_SIZE / 2, true);

	}

	/**
	 * Travel through the bridge
	 */
	public void travelThroughBridge() {
		// Turn towards the tunnel
		turnToCrossing(bridgeZone);

		// Turn clockwise by 90 degrees
		rc.turnBy(90, true);

		// Travel forward by half a tile
		rc.travelDist(rc.TILE_SIZE / 2, true);

		// Turn counterclockwise by 90 degrees
		rc.turnBy(-90, true);

		// Travel through the tunnel/bridge by moving forward by 4 tiles
		rc.travelDist(4 * rc.TILE_SIZE, true);

		// Turn clockwise by 90 degrees
		rc.turnBy(90, true);

		// Travel forward by half a tile
		rc.travelDist(rc.TILE_SIZE / 2, true);

	}

	/**
	 * Turns the robot towards the crossing's lower-left coordinate
	 * in order to prepare to travel through the crossing
	 */
	private void turnToCrossing(int[][] crossingZone) {
		// Compute the nearest waypoint from the odometer reading
		int corrX = (int)Math.round(odo.getXYT()[0] / rc.TILE_SIZE);
		int corrY = (int)Math.round(odo.getXYT()[1] / rc.TILE_SIZE);

		// Find the closest corner of the crossing to the robot
		int closestPointIndex = 0;
		double minDist = 10000000;
		for (int i = 0; i < crossingZone.length; i++) {
			double dist = Math.hypot((corrX - crossingZone[i][0]), (corrY - crossingZone[i][1]));
			if (dist < minDist) {
				minDist = dist;
				closestPointIndex = i;
			}
		}

		// Crossing is vertical and robot is below crossing
		if (corrY - crossingZone[closestPointIndex][1] == -1) {
			rc.turnTo(0);
		}

		// Crossing is vertical and robot is above crossing
		else if (corrY - crossingZone[closestPointIndex][1] == 1) {
			rc.turnTo(180);
		}

		// Crossing is horizontal and robot is to the left of the crossing
		else if (corrX - crossingZone[closestPointIndex][0] == -1) {
			rc.turnTo(90);
		}

		// Crossing is horizontal and robot is to the right of the crossing
		else if (corrX - crossingZone[closestPointIndex][0] == 1) {
			rc.turnTo(270);
		}
	}

	/**
	 * Return to the starting corner
	 */
	public void returnToStart() {
		// TODO Auto-generated method stub

	}


}
