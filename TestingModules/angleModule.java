import java.util.Scanner;
class Main {
    private static final double HubHeight = 2.64; //meters
    private static final double LimelightHeight = 0.4882896; //meters
    private static final double HEIGHTDIFFERENCE = HubHeight-LimelightHeight;; // Height difference between shooter/Limelight and HubStackHeight
    private static final double LimelightFixedAngle = 63 ; // Fixed Limelight Angle

    public static void main(String[] args) {
        Scanner scan = new Scanner(System.in);
        System.out.println("Enter ⍬ (from LimeLight): ");
        double LimelightFluidAngle = scan.nextDouble();
        long startNano = System.nanoTime();
        double distance = 0;

        // if(LimelightFluidAngle < 0) {
        //     distance = 10;
        // } else {
        //     distance = HEIGHTDIFFERENCE/(Math.tan(((Math.toRadians(LimelightFixedAngle))+(Math.toRadians(LimelightFluidAngle)))));
        // }
        distance = HEIGHTDIFFERENCE/(Math.tan(((Math.toRadians(LimelightFixedAngle))+(Math.toRadians(LimelightFluidAngle)))));

        System.out.println("Distance from Hubs: "+distance);
        System.out.println("Shooter Angle: "+calculateAngle(distance)+"°");
        System.out.println("Ideal Ball Veloity: "+calculateVelocity(distance)+" m/s");
        // Calculate Time
        long finishNano = System.nanoTime();
        long elapsedTimeNano = finishNano - startNano; 
        //Time for the Calculation to complete
        System.out.println("Calculation Time: "+elapsedTimeNano/1000000+" ms");
    }

    public static double calculateAngle(double d) { // Takes distance between shooter and the hub (will be from limelight)
        return Math.toDegrees(Math.atan((Math.tan(Math.toRadians(-20))*(d)-(2*HEIGHTDIFFERENCE))/(-d)));
        // Return the necessary angle the shooter should actuate to
    }

    public static double calculateVelocity(double d) {
        return Math.sqrt(-1*((9.8*d*d*(1 + (Math.pow(Math.tan(Math.toRadians(calculateAngle(d))), 2))) )/((2*HEIGHTDIFFERENCE)-(2*d*Math.tan(Math.toRadians(calculateAngle(d)))))));
        // Returns the ideal velocity the ball should be accelerated to by the flywheel.
    }
}