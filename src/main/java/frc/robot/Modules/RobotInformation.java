package frc.robot.Modules;

public class RobotInformation {
    // General Information
    public static final double deadbandAngle = 0.5;
    public static String allianceColor = "red";

    // Driver Information - Ojas!!
    public static final double driverPercentage = 0.8;

    // Robot information
    public static final double robotWeight = 86.484;

    public static final double botlengthInches = 30;
    public static final double botlengthMeters = (botlengthInches/39.37);

    public static final double botwidthInches = 28;
    public static final double botwidthMeters = (botwidthInches/39.37);

    public static final double botlengthBumpersInches = 38.478;
    public static final double botlengthBumpersMeters = (botlengthBumpersInches/39.37);

    public static final double botwidthBumpersInches = 36.478;
    public static final double botwidthBumpersMeters = (botwidthBumpersInches/39.37);

    // Limelight Measurements
    public static final double LimelightAngleDegrees = 40; //Degrees: https://www.desmos.com/calculator/zmzfln2j6v
    public static final double LimelightAngleRadians = Math.toRadians(40);
    public static final double LimelightHeightInches = 22.76;
    public static final double LimelightHeightMeters = (LimelightHeightInches/39.37);

    // Feild Data
    public static final double upperHubHeightInches = 104;
    public static final double upperHubHeightMeters = (upperHubHeightInches/39.37);
    public static final double lowerHubHeightInches = 41;
    public static final double lowerHubHeightMeters = (lowerHubHeightInches/39.37);

    public static final double tarmacLengthInches = 84.75;
    public static final double tarmacLengthMeters = (tarmacLengthInches/39.37);
    public static final double tarmacWidthInches = 153;
    public static final double tarmacWidthMeters = (tarmacWidthInches/39.37);

    // Shooter
    public static final double hubEntryAngle = 45;

    // Flywheel
    public static final double flywheelTransferPercentage = 0.35;

    // RobotMap
    // Drive train ports
    public static final int DRIVETRAIN_FRONTRIGHT = 10;
    public static final int DRIVETRAIN_BACKRIGHT = 11;
    public static final int DRIVETRAIN_FRONTLEFT = 8;
    public static final int DRIVETRAIN_BACKLEFT = 9;


    // Shooter ports
    public static final int SHOOTER_FLY = 3;
    public static final int SHOOTER_HOOD = 3;

    // Intake ports
    public static final int INTAKE_WRIST = 1;
    public static final int INTAKE_ROLLERS = 5;

    // Climb port
    public static final int CLIMBEXTENSION = 7;
    public static final int CLIMBROTATION = 7;

    // CONSTANTS
    // private static final double WHEEL_DIAMETER = 6; // inches
    // //NEED TO FIGURE OUT ENCODER MODEL- changes CPR for drive
    // private static final double cpr = 360; //if am-3132
    // //private static final double cpr = 7/4; //if am-2861a
    // // private static final double cpr = 5; //if am-3314a
    // // private static final double cpr = 1024; //if am-3445
    // // private static final double cpr = 64; //if am-4027
    // private static final double drive_dpp = (Math.PI*WHEEL_DIAMETER/cpr); // Gives in inches per rev FOR DRIVETRAIN WHEELS ONLY

    // 13.0
    public static final double oldlimelightAngle = 14.7734450937;
    public static final double oldlimelightHeight = 17; // inches
}