public class trackingModule {
    private static double shooterAngle = 0;
    private static double distanceFromTopHub = 0;
    private static double hubStackHeight = 109/8; // 8 feet 5 5/8 in
    private static double LimelightSensorHeight = 13/3; // 4 ft 4 in (Flexable but lets assume its max height)
    private static double distanceFromHubBase = 0;
    public static void main(String[] args) {
        getShooterAngle();
    }
    public static void getShooterAngle() {
        // Get this data when testing (hopefully next wendsday)
            // Get time to return from Limelight in ms
            // Convert that to Feet via manual testing
        distanceFromTopHub = 12.37; // Lets say its 10.13 feet
        if(distanceFromTopHub<LimelightSensorHeight) {
            System.out.println("Distance from Top Hub cannot be negetive");
            return;
        }
        hubStackHeight = hubStackHeight-LimelightSensorHeight; // Lets update the height of the hubs from the groud to account for the Limelight's offset
        distanceFromHubBase = Math.sqrt(Math.pow(distanceFromTopHub,2)-Math.pow(hubStackHeight,2)); // Now lets find out how far away from the hubs we are. This can be used for later calculation but is not needed now.
        shooterAngle = Math.asin((hubStackHeight)/(distanceFromTopHub)); // Calculate shooter angle of Limelight sensor to lid of the top Hub.
        System.out.println("Shooter Angle in Radians: "+shooterAngle);
        System.out.println("Shooter Angle in Degrees: "+180*shooterAngle / Math.PI+"°");
        System.out.println("Distance from the top Hub: "+distanceFromTopHub+" ft");
        System.out.println("Distance from Base of the Hub Stack: "+distanceFromHubBase+" ft");
    }
    public static void fireCargo() {
        getShooterAngle(); //Gets the Angle should be aimed at
    }
}

