package frc.robot.constants;

public class Measurements {
    public static class Robot {
        public static final double robotWeight = 86.484; // lbs

        public static final double botlengthInches = 30;
        public static final double botlengthMeters = (botlengthInches / 39.37);

        public static final double botwidthInches = 28.0;
        public static final double botwidthMeters = (botwidthInches / 39.37);

        public static final double botlengthBumpersInches = 38.478;
        public static final double botlengthBumpersMeters = (botlengthBumpersInches / 39.37);

        public static final double botwidthBumpersInches = 36.478;
        public static final double botwidthBumpersMeters = (botwidthBumpersInches / 39.37);

        // Limelight Measurements
        public static final double LimelightAngleDegrees = 40;
        public static final double LimelightAngleRadians = Math.toRadians(LimelightAngleDegrees);
        public static final double LimelightHeightInches = 24.5;
        public static final double LimelightHeightMeters = (LimelightHeightInches / 39.37);

        // Flywheel Measurements
        public static final double shooterHoodAngle = 70;
        public static final double flywheelHeightInches = 23.312;
        public static final double flywheelHeightMeters = flywheelHeightInches / 39.37;

        public static final double drivetrainWheelRadiusInches = 3.0;
        public static final double drivetrainWheelRadiusMeters = 3.0 / 39.37;
        public static final double flywheelRadiusInches = 2.0;
        public static final double flywheelRadiusMeters = flywheelRadiusInches / 39.37;
        public static final double flywheelWidthInches = 2.0;
        public static final double flywheelWidthMeters = flywheelRadiusInches / 39.37;

        public static class GearRatios {
            public static final double drivetrain = 54.0 / 20.0;
            public static final double climbers = 90.0;
            public static final double shooter = (4.0/1.0) * (2.0 / 5.0);
            public static final double rollers = 7;
            public static final double wrist = 10;
        }
    }

    public static class Field {
        /** Upper hub height in inches */
        public static final double upperHubHeightInches = 104;
        /** Upper hub height in meters */
        public static final double upperHubHeightMeters = (upperHubHeightInches / 39.37);
        /** Lower hub height in inches */
        public static final double lowerHubHeightInches = 41;
        /** Lower hub height in meters */
        public static final double lowerHubHeightMeters = (lowerHubHeightInches / 39.37);

        /** Tarmac length in inches */
        public static final double tarmacLengthInches = 84.75;
        /** Tarmac length in meters */
        public static final double tarmacLengthMeters = (tarmacLengthInches / 39.37);
        /** Tarmac width in inches */
        public static final double tarmacWidthInches = 153;
        /** Tarmac width in meters */
        public static final double tarmacWidthMeters = (tarmacWidthInches / 39.37);

    }

    public static class Calculations {
        public static final double kMaxDrivetrainTranslationVelocity = 0.0; // TODO
        public static final double kMaxDrivetrainTranslationAcceleration = 0.0; // TODO
        public static final double kMaxDrivetrainRotationalVelocity = 2 * Math.PI; // TODO
        public static final double kMaxDrivetrainRotationalAcceleration = 2 * kMaxDrivetrainRotationalVelocity; // TODO

        public static final double limelightCenteringDeadbandAngleDeg = 0.5;

        public static final double limelightDistanceOffsetMeters = 0.1;
        // public static final double limelightDistanceOffsetMeters = 0.4;


    }
}
