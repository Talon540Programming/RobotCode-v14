class Main {
    // Height to top of hubs in Meters from the floor
    private static final double HUB_HEIGHT = 2.6416;
    // Height to Limelight from floor in meters on 13.0
    private static final double LIMELIGHT_HEIGHT = 0.4882896; //meters
    // Height difference between Hubs and Limelight
    private static final double HEIGHTDIFFERENCE = HUB_HEIGHT-LIMELIGHT_HEIGHT;; // Height difference between shooter/Limelight and HubStackHeight
    // Angle of Limelight to the ground (63° on 13.0, used for testing)
    private static final double LIME_FIXED_ANGLE = 63 ; // Fixed Limelight Angle
    // Angle of Entry into the hub
    private static final double ENTRY_ANGLE = -45;

    public static void main(String[] args) {
        double LimelightFluidAngle = -22; //Replace with LL data
        double distance = 0;
        while (true) {
            // Check if target is present in FOV if not spin till found
            // Maximum Horizontal FOV IS 54°
            // Maximum Vertical FOV is 41°
            distance = HEIGHTDIFFERENCE/(Math.tan(((Math.toRadians(LIME_FIXED_ANGLE))+(Math.toRadians(LimelightFluidAngle)))));
            double angle = calculateAngle(distance);
            double velocity = calculateVelocity(distance, angle);

            System.out.println("Distance from Hubs: "+distance);
            System.out.println("Shooter Angle: "+angle+"°");
            System.out.println("Ideal Ball Veloity: "+ velocity +" m/s");
        }

        //System.out.println("Code exited with code: 0");
        //System.exit(0);
    }

    public static double calculateAngle(double d) { // Takes distance between shooter and the hub (will be from limelight)
        return Math.toDegrees(Math.atan((Math.tan(Math.toRadians(ENTRY_ANGLE))*(d)-(2*HEIGHTDIFFERENCE))/(-d)));
        // Return the necessary angle the shooter should actuate to
    }

    public static double calculateVelocity(double d, double a) {
        return Math.sqrt(-1*((9.8*d*d*(1 + (Math.pow(Math.tan(Math.toRadians(a)), 2))) )/((2*HEIGHTDIFFERENCE)-(2*d*Math.tan(Math.toRadians(a))))));
        // Returns the ideal velocity the ball should be accelerated to by the flywheel.
    }
}
