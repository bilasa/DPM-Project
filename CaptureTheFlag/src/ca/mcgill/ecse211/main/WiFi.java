package ca.mcgill.ecse211.main;

import java.util.Enumeration;
import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import ca.mcgill.ecse211.enumeration.Flag;
import ca.mcgill.ecse211.enumeration.Team;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

/**
 * This class fetches all data sent to it from the server JAR
 * file. The class contains methods to fetch attributes of the
 * given challenge, including one's team, arrays for search zones,
 * team zones, the tunnel and the bridge, the length of the tunnel/
 * bridge, and whether the tunnel/bridge are vertical or horizontal. 
 * This class is used by all classes in the second layer (navigation)
 * since they need access to the attributes of the challenge.
 * 
 * @author Esa Khan
 */
public class WiFi {

	// ** Set these as appropriate for your team and current situation **

	// ***Bijan's***
	//private static final String SERVER_IP = "192.168.2.10";
	// ***Esa's***
	private static final String SERVER_IP = "192.168.2.20";
	// ***Gui's***
	//private static final String SERVER_IP = "192.168.2.44";
	// ***TA's***
	//private static final String SERVER_IP = "192.168.2.3";

	private static final int TEAM_NUMBER = 8;

	// Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = false;

	// Create Map variable
	private Map data;

	@SuppressWarnings("rawtypes")
	public WiFi() {
		// Store the data
		this.getData();

		// Clear the console
		System.out.flush();
	}

	/**
	 * Stores all the data sent by the server JAR file into a Map object.
	 */
	public void getData() {
		System.out.println("Running..");
		// Initialize WifiConnection class
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might
		// occur
		try {
			/*
			 * getData() will connect to the server and wait until the user/TA
			 * presses the "Start" button in the GUI on their laptop with the
			 * data filled in. Once it's waiting, you can kill it by pressing
			 * the upper left hand corner button (back/escape) on the EV3.
			 * getData() will throw exceptions if it can't connect to the server
			 * (e.g. wrong IP address, server not running on laptop, not
			 * connected to WiFi router, etc.). It will also throw an exception
			 * if it connects but receives corrupted data or a message from the
			 * server saying something went wrong. For example, if TEAM_NUMBER
			 * is set to 1 above but the server expects teams 17 and 5, this
			 * robot will receive a message saying an invalid team number was
			 * specified and getData() will throw an exception letting you know.
			 */
			data = conn.getData();

		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}
	}

	/**
	 * Gets the color of the team you are in.
	 * 
	 * @return Team color enumeration for the team you are in
	 */
	public Team getTeam() {
		// Return the corresponding team colour for the team number (8)
		if (((Long) data.get("RedTeam")).intValue() == TEAM_NUMBER) {
			return Team.RED;
		} else if (((Long) data.get("GreenTeam")).intValue() == TEAM_NUMBER) {
			return Team.GREEN;
		} else {
			return null;
		}
	}

	/**
	 * Gets the starting corner of the given team.
	 * 
	 * @return an int representing the starting corner of the team.
	 */
	public int getStartingCorner(Team team) {
		// Check which team we are
		if (team == Team.RED) {
			// Return the corresponding starting corner for our team
			return ((Long) data.get("RedCorner")).intValue();
		} else if (team == Team.GREEN) {
			// Repeat for green team
			return ((Long) data.get("GreenCorner")).intValue();
		}
		return -1;
	}

	/**
	 * Gets the starting corner coordinates of your team.
	 * 
	 * @return an int array containing the x and y coordinates of your starting corner
	 */
	public int[] getStartingCornerCoords() {
		int[] coords = { 0, 0 };
		switch (getStartingCorner(this.getTeam())) {
		case 0:
			coords[0] = 1;
			coords[1] = 1;
			break;
		case 1:
			coords[0] = 11;
			//coords[0] = 7;
			coords[1] = 1;
			break;
		case 2:
			coords[0] = 11;
			//coords[0] = 7;
			coords[1] = 11;
			//coords[1] = 7;
			break;
		case 3:
			coords[0] = 1;
			coords[1] = 11;
			//coords[1] = 7;
			break;
		}
		return coords;
	}

	/**
	 * Gets the color of target flag of your team (from the opponent's search zone).
	 * 
	 * @return Flag color enumeration representing your target flag color 
	 */
	public Flag getFlagColor() {
		// Check which team we are
		if (getTeam() == Team.RED) {
			// Return the approriate color for the number
			switch (((Long) data.get("OR")).intValue()) {
			case 1:
				return Flag.RED;
			case 2:
				return Flag.BLUE;
			case 3:
				return Flag.YELLOW;
			case 4:
				return Flag.WHITE;
			}
		} else if (getTeam() == Team.GREEN) {
			// Return the approriate color for the number
			switch (((Long) data.get("OG")).intValue()) {
			case 1:
				return Flag.RED;
			case 2:
				return Flag.BLUE;
			case 3:
				return Flag.YELLOW;
			case 4:
				return Flag.WHITE;
			}
		}
		// Return null if no match found
		return null;
	}

	/**
	 * Gets the coordinates of the red zone.
	 * 
	 * @return two dimensional int array containing the four corners of the red zone as (x, y) pairs
	 */
	public int[][] getRedZone() {
		// Get coords of red zone
		int lowerLeftX = ((Long) data.get("Red_LL_x")).intValue(),
				lowerLeftY = ((Long) data.get("Red_LL_y")).intValue(),
				upperRightX = ((Long) data.get("Red_UR_x")).intValue(),
				upperRightY = ((Long) data.get("Red_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		int[][] RedZone = { { lowerLeftX, lowerLeftY }, { upperRightX, lowerLeftY }, { upperRightX, upperRightY },
				{ lowerLeftX, upperRightY } };

		return RedZone;
	}

	/**
	 * Gets the coordinates of the green zone.
	 * 
	 * @return two dimensional int array containing the four corners of the green zone as (x, y) pairs
	 */
	public int[][] getGreenZone() {
		// Get coords of green zone
		int lowerLeftX = ((Long) data.get("Green_LL_x")).intValue(),
				lowerLeftY = ((Long) data.get("Green_LL_y")).intValue(),
				upperRightX = ((Long) data.get("Green_UR_x")).intValue(),
				upperRightY = ((Long) data.get("Green_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		int[][] GreenZone = { { lowerLeftX, lowerLeftY }, { upperRightX, lowerLeftY }, { upperRightX, upperRightY },
				{ lowerLeftX, upperRightY } };

		return GreenZone;
	}

	/**
	 * Gets the coordinates of the tunnel.
	 * 
	 * @return two dimensional int array containing the four corners of the tunnel as (x, y) pairs
	 */
	public int[][] getTunnelZone() {
		// Get coords of tunnel zone
		int lowerLeftX = ((Long) data.get("TN_LL_x")).intValue(), lowerLeftY = ((Long) data.get("TN_LL_y")).intValue(),
				upperRightX = ((Long) data.get("TN_UR_x")).intValue(),
				upperRightY = ((Long) data.get("TN_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		int[][] TunnelZone = { { lowerLeftX, lowerLeftY }, { upperRightX, lowerLeftY }, { upperRightX, upperRightY },
				{ lowerLeftX, upperRightY } };

		return TunnelZone;
	}

	/**
	 * Gets the coordinates of the bridge.
	 * 
	 * @return two dimensional int array containing the four corners of the bridge as (x, y) pairs
	 */
	public int[][] getBridgeZone() {
		// Get coords of tunnel zone
		int lowerLeftX = ((Long) data.get("BR_LL_x")).intValue(), lowerLeftY = ((Long) data.get("BR_LL_y")).intValue(),
				upperRightX = ((Long) data.get("BR_UR_x")).intValue(),
				upperRightY = ((Long) data.get("BR_UR_y")).intValue();

		// Corner convention:
		// [0] = Lower Left
		// [1] = Lower Right
		// [2] = Upper Right
		// [3] = Upper Left
		int[][] BridgeZone = { { lowerLeftX, lowerLeftY }, { upperRightX, lowerLeftY }, { upperRightX, upperRightY },
				{ lowerLeftX, upperRightY } };

		return BridgeZone;
	}

	/**
	 * Gets the search zone of the specified team
	 * 
	 * @param team the team of the search zone wanted
	 * @return two dimensional int array containing the four corners of the search zone as (x, y) pairs
	 */
	public int[][] getSearchZone(Team team) {
		int lowerLeftX, lowerLeftY, upperRightX, upperRightY;

		switch (team) {
		case RED:
			// Get coords of red search zone
			lowerLeftX = ((Long) data.get("SR_LL_x")).intValue();
			lowerLeftY = ((Long) data.get("SR_LL_y")).intValue();
			upperRightX = ((Long) data.get("SR_UR_x")).intValue();
			upperRightY = ((Long) data.get("SR_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] RedSearchZone = { { lowerLeftX, lowerLeftY }, { upperRightX, lowerLeftY },
					{ upperRightX, upperRightY }, { lowerLeftX, upperRightY } };

			return RedSearchZone;
		case GREEN:
			// Get coords of red search zone
			lowerLeftX = ((Long) data.get("SG_LL_x")).intValue();
			lowerLeftY = ((Long) data.get("SG_LL_y")).intValue();
			upperRightX = ((Long) data.get("SG_UR_x")).intValue();
			upperRightY = ((Long) data.get("SG_UR_y")).intValue();

			// Corner convention:
			// [0] = Lower Left
			// [1] = Lower Right
			// [2] = Upper Right
			// [3] = Upper Left
			int[][] GreenSearchZone = { { lowerLeftX, lowerLeftY }, { upperRightX, lowerLeftY },
					{ upperRightX, upperRightY }, { lowerLeftX, upperRightY } };

			return GreenSearchZone;
		default:
			return null;
		}
	}

	/**
	 * Determines the orientation of the crossings by checking which side 
	 * of the tunnel intersects with the greenzone (both point of the side).
	 * 
	 * @return whether or not the bridge/tunnel are placed vertically in the playzone
	 */
	public boolean isCrossingVert() {
		int[][] tunnelZone = getTunnelZone();
		int[][] greenZone = getGreenZone();
		// Crossing is vertical if the difference between tunnel's lower-left x
		// and lower-right x is 1
		/*if (tunnelZone[1][0] - tunnelZone[0][0] == 1)
			return true;
		return false;*/
		
		boolean bottomLeft = false;
		boolean bottomRight = false;
		boolean upperRight = false;
		boolean upperLeft = false;
		
		// Check if the bottom side of the tunnel touches the green zone
		if(tunnelZone[0][0] >= greenZone[0][0] && tunnelZone[0][0] <= greenZone[2][0] && tunnelZone[0][1] >= greenZone[0][1] && tunnelZone[0][1] <= greenZone[2][1])
			bottomLeft = true;

		// Check if the right side of the tunnel touches the green zone
		if(tunnelZone[1][0] >= greenZone[0][0] && tunnelZone[1][0] <= greenZone[2][0] && tunnelZone[1][1] >= greenZone[0][1] && tunnelZone[1][1] <= greenZone[2][1])
			bottomRight = true;

		// Check if the bottom side of the tunnel touches the green zone
		if(tunnelZone[2][0] >= greenZone[0][0] && tunnelZone[2][0] <= greenZone[2][0] && tunnelZone[2][1] >= greenZone[0][1] && tunnelZone[2][1] <= greenZone[2][1])
			upperRight = true;

		// Check if the bottom side of the tunnel touches the green zone
		if(tunnelZone[3][0] >= greenZone[0][0] && tunnelZone[3][0] <= greenZone[2][0] && tunnelZone[3][1] >= greenZone[0][1] && tunnelZone[3][1] <= greenZone[2][1])
			upperLeft = true;
		
		// If the bottom or top side of the tunnel touches the greenzone, the crossings must be vertical
		if((bottomLeft && bottomRight) || (upperLeft && upperRight)) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Gets the length of the bridge and tunnel.
	 * 
	 * @return the length of the bridge/tunnel
	 */
	public int getCrossingLength() {
		int[][] tunnelZone = getTunnelZone();

		return Math.max(Math.abs(tunnelZone[0][0] - tunnelZone[2][0]), Math.abs(tunnelZone[0][1] - tunnelZone[2][1]));
	}


}