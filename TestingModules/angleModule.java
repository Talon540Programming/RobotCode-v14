import java.util.Scanner;
class Main {
    // Height to top of hubs in Meters from the floor
    private static final double HubHeight = 2.64;
    // Height to Limelight from floor in meters on 13.0
    private static final double LimelightHeight = 0.4882896; //meters
    // Height difference between Hubs and Limelight
    private static final double HEIGHTDIFFERENCE = HubHeight-LimelightHeight;; // Height difference between shooter/Limelight and HubStackHeight
    // Angle of Limelight to the ground (63° on 13.0, used for testing)
    private static final double LimelightFixedAngle = 63 ; // Fixed Limelight Angle
    // Angle of Entry into the hub
    private static final double IdealEntryAngle = -20;

    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);
        System.out.println("Enter ⍬ (from LimeLight) in Degrees: ");
        double LimelightFluidAngle = scan.nextDouble();
        long startNano = System.nanoTime();
        double distance = 0;

        distance = HEIGHTDIFFERENCE/(Math.tan(((Math.toRadians(LimelightFixedAngle))+(Math.toRadians(LimelightFluidAngle)))));
        
        double angle = calculateAngle(distance)
        double velocity = calculateVelocity(distance, angle)
            
        System.out.println("Distance from Hubs: "+distance);
        System.out.println("Shooter Angle: "+angle+"°");
        System.out.println("Ideal Ball Veloity: "+ velocity +" m/s");
        // Calculate Time
        long finishNano = System.nanoTime();
        long elapsedTimeNano = finishNano - startNano; 
        //Time for the Calculation to complete
        System.out.println("Calculation Time: "+elapsedTimeNano/1000000+" ms");
    }

    public static double calculateAngle(double d) { // Takes distance between shooter and the hub (will be from limelight)
        return Math.toDegrees(Math.atan((Math.tan(Math.toRadians(IdealEntryAngle))*(d)-(2*HEIGHTDIFFERENCE))/(-d)));
        // Return the necessary angle the shooter should actuate to
    }

    public static double calculateVelocity(double d, double a) {
        return Math.sqrt(-1*((9.8*d*d*(1 + (Math.pow(Math.tan(Math.toRadians(a)), 2))) )/((2*HEIGHTDIFFERENCE)-(2*d*Math.tan(Math.toRadians(a))))));
        // Returns the ideal velocity the ball should be accelerated to by the flywheel.
    }
}
