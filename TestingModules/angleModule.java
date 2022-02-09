/** 
All measurments are in the metric system, and were converted from the game manual's customary measurments
for stuff like the hub height, dimensions, and other feild parameters for the shooting mechanism code.
**/

class Main {
    // Height to top of hubs in Meters from the floor
    private static final double HUB_HEIGHT = 72/39.37;
    // Height to Limelight from floor in meters on 13.0
    private static final double LIMELIGHT_HEIGHT = 17/39.37; //meters
    // Height difference between Hubs and Limelight
    private static final double HEIGHTDIFFERENCE = HUB_HEIGHT-LIMELIGHT_HEIGHT;; // Height difference between shooter/Limelight and HubStackHeight
    // Angle of Limelight to the ground (63° on 13.0, used for testing)
    private static final double LIME_FIXED_ANGLE = 14.7734450937; // Fixed Limelight Angle //13.0 Angle = 14.7734450937
    // Angle of Entry into the hub
    private static final double ENTRY_ANGLE = -45;

    public static void main(String[] args) {
        // System.out.println(getShooterInfo(-22)[0]);
        //Get Data from Limelight (Tv, Tx, Ty, Tl)
        double Tv = 1; // 0 if no Target 1 if Target
        double Tx = 1; // Degree Measure from Straight forward target (horizontal)
        double Ty = 1.66; // Degree measure from Fixed Angle: https://www.desmos.com/calculator/6vvjdedpc6
        double Tl = 1;
        double[] LimelightInfo = {Tv,Tx,Ty,Tl};

        double[] shooterCalculations = getShooterInfo(LimelightInfo);
        System.out.println("Distance: "+shooterCalculations[0]+" m");
        System.out.println("Shooter Angle: "+shooterCalculations[1]+"°");
        System.out.println("Ideal Ball Velocity :"+shooterCalculations[2]+" m/s");
    }

    public static double[] getShooterInfo(double[] LLData) {

        double LimelightFluidAngle = LLData[2]; 
        double distance = 0;

        // Check if target is present in FOV if not spin till found
        // Maximum Horizontal FOV IS 54°
        // Maximum Vertical FOV is 41°

        distance = HEIGHTDIFFERENCE / (Math.tan(((Math.toRadians(LIME_FIXED_ANGLE)) + (Math.toRadians(LimelightFluidAngle)))));
        double angle = calculateAngle(distance);
        double velocity = calculateVelocity(distance, angle);

        double[] shooterInfo = {distance, angle, velocity};
        return shooterInfo;

    }

    public static double calculateAngle(double d) { // Takes distance between shooter and the hub (will be from limelight)
        return Math.toDegrees(Math.atan((Math.tan(Math.toRadians(ENTRY_ANGLE)) * (d)-(2 * HEIGHTDIFFERENCE)) / (-d)));
        // Return the necessary angle the shooter should actuate to
    }

    public static double calculateVelocity(double d, double a) {
        return Math.sqrt(-1 * ((9.8 * d * d * (1 + (Math.pow(Math.tan(Math.toRadians(a)), 2))) )/((2 * HEIGHTDIFFERENCE)-(2 * d * Math.tan(Math.toRadians(a))))));
        // Returns the ideal velocity the ball should be accelerated to by the flywheel.
    }
}
