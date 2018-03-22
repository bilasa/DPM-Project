# DPM-Project

## 1.0
### Initial Commit
By Bijan Sadeghi

## 1.1
### Change Front Color Sensor Mode (18/03/05)
* Changed the front color sensor mode to RGB
* Changed the track width to 20 - Will need to adjust  

By Guillaume Richard

## 1.2
### Adjust Wheel Radius and Track Width (18/03/05)
* Changed the wheel radius to 1.67
* Changed the track width to 20.9
* Added `static` to every variable and class in the [CaptureTheFlag.java](https://github.com/BijanSadeghi/DPM-Project/tree/master/CaptureTheFlag/src/ca/mcgill/ecse211/main/CaptureTheFlag.java) class

By Guillaume Richard

## 1.3
### Implement Main Code Hierarchy and Initial FlagSearcher (18/03/07)
* Implemented step by step method calls for the challenge in the main method
* Implemented "getClosestSearchCorner()" in FlagSearcher to return the closest point in the search zone to the bridge/tunnel

By Bijan Sadeghi & Esa Khan

## 1.4
### Update Odometer Class and Localization Class (18/03/09)
* Switched Odometer class to the class used in Lab5
* Updated generalLightLocalize() to localize at any point instead of just the origin

By Bijan Sadeghi & Esa Khan

## 1.5
### Finish travelToTunnel() (18/03/12)
* Implemented all switch cases for travelToTunnel()

By Bijan Sadeghi & Esa Khan

## 1.6
### Finish travelToBridge() and Start travelThrough (18/03/12)
* Used same logic as travelToTunnel() for travelToBridge()
* Started travelThrough() using turnToCrossing() method

By Bijan Sadeghi & Esa Khan

## 1.7
### Adjust Wheel Radius and Track Width (18/03/14)
* Changed the wheel radius to 1.66
* Changed the track width to 17.8

By Guillaume Richard

## 1.7.1
### Complete javadoc and create class diagram (18/03/15)
* Wrote all javadoc comments
* Generated javadoc API
* Created class diagram file

By Bijan Sadeghi

## 1.8
### Fix travelTo and Implement returnToStart (18/03/21)
* Fixed flawed switch cases in travelTo methods
* Implemented returnToStart
* Regenerated javadoc API

By Bijan Sadeghi

## 1.9
### Adjust Initial Localization Correction Factor (18/03/22)
* Correction factor set in `initialLightLocalize` to 24 degrees
* `ROTATE_SPEED` set to 250
