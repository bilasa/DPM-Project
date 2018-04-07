package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.controller.RobotController;
import ca.mcgill.ecse211.enumeration.Team;
import ca.mcgill.ecse211.main.WiFi;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.odometer.OdometryCorrection;

/**
 * This class allows the navigation of the robot through various parts of 
 * the playzone during the challenge. The Navigator includes methods to 
 * navigate to the tunnel/bridge, travel through the tunnel/bridge, and
 * travel back to the starting corner.
 * 
 * @author Bijan Sadeghi
 * @author Esa Khan
 */
public class Navigator {

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

	// OdometryCorrection
	private OdometryCorrection odoCorrection;
	
	// Flag searcher
	private FlagSearcher flagSearcher;

	// Corner of the search zone we start and end the search at
	private int[] startingSearchCorner;

	/**
	 * @param rc the robot controller to use
	 * @param wifi the wifi object to get the challenge data from
	 * @param flagSeacher the flag searcher to use during the flag search
	 */
	public Navigator(RobotController rc, WiFi wifi, FlagSearcher flagSearcher) {
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
		this.flagSearcher = flagSearcher;
	}

	/**
	 * Moves the robot to the entrance of the tunnel, taking account which
	 * team the robot is in, as well as the robot's starting corner and the
	 * orientation of the tunnel. The point the robot travels to is always
	 * the "lower-left" point relative to the tunnel. Uses an L-shape 
	 * to travel on the edges of the zone as to avoid colliding with blocks 
	 * in the search zone.
	 * 
	 */
	public void travelToTunnel() {
		// Extract the tunnel coordinates
		int[] tunnelLL = tunnelZone[0];
		int[] tunnelUR = tunnelZone[2];

		// Get the team
		Team team = wifi.getTeam();

		// ====== Select the team ====== //
		switch (team) {
		case GREEN:
			// ===============================================================//
			// Check if the tunnel has a horizontal or vertical orientation //
			// ===============================================================//
			if (wifi.isCrossingVert()) {
				// ===============================================================//
				// Select the starting corner of the green zone //
				// Then travel to closest point next to the tunnel in an L-shape //
				// ===============================================================//
				switch (wifi.getStartingCorner(team)) {
				case 0:
				case 1:
					rc.travelTo(startingCornerCoords[0], tunnelLL[1] - 1, FORWARD_SPEED);
					rc.travelTo(tunnelLL[0], tunnelLL[1] - 1, FORWARD_SPEED);
					break;
				case 2:
				case 3:
					rc.travelTo(startingCornerCoords[0], tunnelUR[1] + 1, FORWARD_SPEED);
					rc.travelTo(tunnelUR[0], tunnelUR[1] + 1, FORWARD_SPEED);
					break;
				}
			} else {
				// ===============================================================//
				// Select the starting corner of the green zone //
				// Then travel to closest point next to the tunnel in an L-shape //
				// ===============================================================//
				switch (wifi.getStartingCorner(team)) {
				case 0:
				case 3:
					rc.travelTo(tunnelLL[0] - 1, startingCornerCoords[1], FORWARD_SPEED);
					rc.travelTo(tunnelLL[0] - 1, tunnelLL[1] + 1, FORWARD_SPEED);
					break;
				case 1:
				case 2:
					rc.travelTo(tunnelUR[0] + 1, startingCornerCoords[1], FORWARD_SPEED);
					rc.travelTo(tunnelUR[0] + 1, tunnelUR[1] - 1, FORWARD_SPEED);
					break;
				}
			}
			break;
		case RED:
			// ===============================================================//
			// Check if the tunnel has a horizontal or vertical orientation //
			// ===============================================================//
			if (wifi.isCrossingVert()) {
				// ==========================================================//
				// Select the starting corner //
				// Then directly travel to closest point next to the tunnel //
				// ==========================================================//
				switch (wifi.getStartingCorner(team)) {
				case 0:
				case 1:
					rc.travelTo(tunnelUR[0], tunnelUR[1] + 1, FORWARD_SPEED);
					break;
				case 2:
				case 3:
					rc.travelTo(tunnelLL[0], tunnelLL[1] - 1, FORWARD_SPEED);
					break;
				}
			} else {
				// ==========================================================//
				// Select the starting corner //
				// Then directly travel to closest point next to the tunnel //
				// ==========================================================//
				switch (wifi.getStartingCorner(team)) {
				case 0:
				case 3:
					rc.travelTo(tunnelUR[0] + 1, tunnelUR[1] - 1, FORWARD_SPEED);
					break;
				case 1:
				case 2:
					rc.travelTo(tunnelLL[0] - 1, tunnelLL[1] + 1, FORWARD_SPEED);
					break;
				}
			}
			break;
		}
	}

	/**
	 * Moves the robot to the entrance of the bridge, taking account which
	 * team the robot is in, as well as the robot's starting corner and the
	 * orientation of the bridge. The point the robot travels to is always
	 * the "lower-left" point relative to the bridge. Uses an L-shape 
	 * to travel on the edges of the zone as to avoid colliding with blocks 
	 * in the search zone.
	 * 
	 */
	public void travelToBridge() {
		// Extract the bridge coordinates
		int[] bridgeLL = bridgeZone[0];
		int[] bridgeUR = bridgeZone[2];

		// Get the team
		Team team = wifi.getTeam();

		// ====== Select the team ====== //
		switch (team) {
		case RED:
			// ===============================================================//
			// Check if the bridge has a horizontal or vertical orientation //
			// ===============================================================//
			if (wifi.isCrossingVert()) {
				// ===============================================================//
				// Select the starting corner of the red zone //
				// Then travel to closest point next to the bridge in an L-shape //
				// ===============================================================//
				switch (wifi.getStartingCorner(team)) {
				case 0:
				case 1:
					rc.travelTo(startingCornerCoords[0], bridgeLL[1] - 1, FORWARD_SPEED);
					rc.travelTo(bridgeLL[0], bridgeLL[1] - 1, FORWARD_SPEED);
					break;
				case 2:
				case 3:
					rc.travelTo(startingCornerCoords[0], bridgeUR[1] + 1, FORWARD_SPEED);
					rc.travelTo(bridgeUR[0], bridgeUR[1] + 1, FORWARD_SPEED);
					break;
				}
			} else {
				// ===============================================================//
				// Select the starting corner of the red zone //
				// Then travel to closest point next to the bridge in an L-shape //
				// ===============================================================//
				switch (wifi.getStartingCorner(team)) {
				case 0:
				case 3:
					rc.travelTo(bridgeLL[0] - 1, startingCornerCoords[1], FORWARD_SPEED);
					rc.travelTo(bridgeLL[0] - 1, bridgeLL[1] + 1, FORWARD_SPEED);
					break;
				case 1:
				case 2:
					rc.travelTo(bridgeUR[0] + 1, startingCornerCoords[1], FORWARD_SPEED);
					rc.travelTo(bridgeUR[0] + 1, bridgeUR[1] - 1, FORWARD_SPEED);
					break;
				}
			}
			break;
		case GREEN:
			// ===============================================================//
			// Check if the bridge has a horizontal or vertical orientation //
			// ===============================================================//
			if (wifi.isCrossingVert()) {
				// ==========================================================//
				// Select the starting corner //
				// Then directly travel to closest point next to the bridge //
				// ==========================================================//
				switch (wifi.getStartingCorner(team)) {
				case 0:
				case 1:
					rc.travelTo(bridgeUR[0], bridgeUR[1] + 1, FORWARD_SPEED);
					break;
				case 2:
				case 3:
					rc.travelTo(bridgeLL[0], bridgeLL[1] - 1, FORWARD_SPEED);
					break;
				}
			} else {
				// ==========================================================//
				// Select the starting corner //
				// Then directly travel to closest point next to the tunnel //
				// ==========================================================//
				switch (wifi.getStartingCorner(team)) {
				case 0:
				case 1:
					rc.travelTo(bridgeUR[0] + 1, bridgeUR[1] - 1, FORWARD_SPEED);
					break;
				case 2:
				case 3:
					rc.travelTo(bridgeLL[0] - 1, bridgeLL[1] + 1, FORWARD_SPEED);
					break;
				}
			}
			break;
		}
	}

	/**
	 * Moves the robot through the tunnel by traveling until the robot is on the tile
	 * at the exit of the tunnel, taking into account the length of the tunnel. 
	 * It then corrects the robot using odometry correction and the robot ends up 
	 * at the relative upper-right/lower-left point with respect to the tunnel.
	 */
	public void travelThroughTunnel() {
		// Turn towards the tunnel
		turnToCrossing(tunnelZone);

		// Correct until the line
		odoCorrection.correct();

		rc.setSpeeds(5000, 5000);
		
		// Travel through the tunnel/bridge by moving forward by the (length of the crossing + 1.5)
		rc.travelDist((wifi.getCrossingLength() + 1.5) * rc.TILE_SIZE, true);

		rc.setSpeeds(rc.FORWARD_SPEED, rc.FORWARD_SPEED);

		
		// Correct until the line
		odoCorrection.correct();

		// Move back so the robot is on the line
		rc.travelDist(-rc.REAR_SENSOR_DIST, true);

		// Turn clockwise by 90 degrees
		rc.turnBy(90, true);

		// Travel forward by half a tile
		rc.travelDist(rc.TILE_SIZE / 2, true);

		// Correct until the line
		odoCorrection.correct();

	}

	/**
	 * Moves the robot through the bridge by traveling until the robot is on the tile
	 * at the exit of the bridge, taking into account the length of the bridge. 
	 * It then corrects the robot using odometry correction and the robot ends up 
	 * at the relative upper-right/lower-left point with respect to the bridge.
	 */
	public void travelThroughBridge() {
		// Turn towards the bridge
		turnToCrossing(bridgeZone);

		// Correct until the line
		odoCorrection.correct();
		
		rc.setSpeeds(1000, 1000);
		
		// Travel through the tunnel/bridge by moving forward by the (length of the crossing + 1.5)
		rc.travelDist((wifi.getCrossingLength() + 1.5) * rc.TILE_SIZE, true);

		rc.setSpeeds(rc.FORWARD_SPEED, rc.FORWARD_SPEED);
		
		// Correct until the line
		odoCorrection.correct();

		// Move back so the robot is on the line
		rc.travelDist(-rc.REAR_SENSOR_DIST, true);

		// Turn clockwise by 90 degrees
		rc.turnBy(90, true);

		// Travel forward by half a tile
		rc.travelDist(rc.TILE_SIZE / 2, true);

		// Correct until the line
		odoCorrection.correct();

	}

	/**
	 * Positions the robot so that it is facing the entrance of the tunnel/bridge
	 * and is ready to travel through it.
	 * 
	 * @return the angle the robot is facing when it will cross the tunnel/bridge
	 */
	private void turnToCrossing(int[][] crossingZone) {
		// Compute the nearest waypoint from the odometer reading
		int corrX = (int) Math.round(odo.getXYT()[0] / rc.TILE_SIZE);
		int corrY = (int) Math.round(odo.getXYT()[1] / rc.TILE_SIZE);

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
			rc.turnTo(90);
		}

		// Crossing is vertical and robot is above crossing
		else if (corrY - crossingZone[closestPointIndex][1] == 1) {
			rc.turnTo(270);
		}

		// Crossing is horizontal and robot is to the left of the crossing
		else if (corrX - crossingZone[closestPointIndex][0] == -1) {
			rc.turnTo(180);
		}

		// Crossing is horizontal and robot is to the right of the crossing
		else if (corrX - crossingZone[closestPointIndex][0] == 1) {
			rc.turnTo(0);
		}

		// Correct until the line
		odoCorrection.correct();

		// Travel forward by half a tile
		rc.travelDist(rc.TILE_SIZE / 2 - rc.REAR_SENSOR_DIST, true);

		// Turn counterclockwise by 90 degrees
		rc.turnBy(-90, true);
	}

	/**
	 * Returns the robot to its starting corner by doing the reverse of the L-shape
	 * path done in travelToTunnel() or travelToBridge()
	 */
	public void returnToStart() {
		int[] crossingLL = { 0, 0 };
		int[] crossingUR = { 0, 0 };

		// Extract the correct crossing coordinates
		if (wifi.getTeam() == Team.GREEN) {
			crossingLL = bridgeZone[0];
			crossingUR = bridgeZone[2];
		} else if (wifi.getTeam() == Team.RED) {
			crossingLL = tunnelZone[0];
			crossingUR = tunnelZone[2];
		}

		// ===============================================================//
		// Check if the bridge has a horizontal or vertical orientation //
		// ===============================================================//
		if (wifi.isCrossingVert()) {
			// ===============================================================//
			// Select the starting corner of the red zone //
			// Then travel to closest point next to the bridge in an L-shape //
			// ===============================================================//
			switch (wifi.getStartingCorner(wifi.getTeam())) {
			case 0:
			case 1:
				rc.travelTo(startingCornerCoords[0], crossingLL[1] - 1, FORWARD_SPEED);
				rc.travelTo(startingCornerCoords[0], startingCornerCoords[1], FORWARD_SPEED);
				break;
			case 2:
			case 3:
				rc.travelTo(startingCornerCoords[0], crossingUR[1] + 1, FORWARD_SPEED);
				rc.travelTo(startingCornerCoords[0], startingCornerCoords[1], FORWARD_SPEED);
				break;
			}
		} else {
			// ===============================================================//
			// Select the starting corner of the red zone //
			// Then travel to closest point next to the bridge in an L-shape //
			// ===============================================================//
			switch (wifi.getStartingCorner(wifi.getTeam())) {
			case 0:
			case 3:
				rc.travelTo(crossingLL[0] - 1, startingCornerCoords[1], FORWARD_SPEED);
				rc.travelTo(startingCornerCoords[0], startingCornerCoords[1], FORWARD_SPEED);
				break;
			case 1:
			case 2:
				rc.travelTo(crossingUR[0] + 1, startingCornerCoords[1], FORWARD_SPEED);
				rc.travelTo(startingCornerCoords[0], startingCornerCoords[1], FORWARD_SPEED);
				break;
			}
		}
	}

	/**
	 * Set the OdometryCorrection object to be used by the robot controller
	 * 
	 * @param odoCorrection the OdometryCorrection object to be used
	 */
	public void setOdoCorrection(OdometryCorrection odoCorrection) {
		this.odoCorrection = odoCorrection;
	}


	/**
	 * Gets the corrected angle of the robot given the odometer's theta reading
	 * when the robot's intended path is vertical or horizontal.
	 * 
	 * @return the correct angle the robot's odometer must use to correct itself
	 */
	/*private double getCorrTheta() {
		double corrTheta = 0;
		double[] odoData = { 0, 0, 0 };

		// Check which way robot is facing
		try {
			odoData = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e) {
			// Do nothing
			e.printStackTrace();
		}

		if (odoData[2] > 350 || odoData[2] < 10) {
			corrTheta = 0;
		} else if (odoData[2] > 80 && odoData[2] < 100) {
			corrTheta = 90;
		} else if (odoData[2] > 170 && odoData[2] < 190) {
			corrTheta = 180;
		} else if (odoData[2] > 260 && odoData[2] < 280) {
			corrTheta = 270;
		}

		return corrTheta;
	}
*/
	
	
	
	/**
	 * Travels to the corner of the search zone closest to the robot
	 * after it has crossed the bridge/tunnel into the opponent
	 * team's zone.
	 */
	public void travelToSearchZone() {
		startingSearchCorner = flagSearcher.getClosestSearchCorner();
		rc.travelTo(startingSearchCorner[0], startingSearchCorner[1], rc.FORWARD_SPEED);
	}
}
