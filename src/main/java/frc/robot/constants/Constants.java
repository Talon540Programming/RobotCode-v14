package frc.robot.constants;

public class Constants {
    // General Information

    public static final double kMaxDrivetrainTranslationVelocity = 0.0; // TODO
    public static final double kMaxDrivetrainTranslationAcceleration = 0.0; // TODO
    public static final double kMaxDrivetrainRotationalVelocity = 0.0; // TODO
    public static final double kMaxDrivetrainRotationalAcceleration = 0.0; // TODO

    /** Minimum Angle that wont trigger the robot moving */
    public static final double deadbandAngle = 0.5; // Last Known Working = 0.5

    public static final double driveToDistanceOffset = 0.1; // meters
    // 13.0
    /** Fixed Limelight Angle on Talon540 2021 13.0 */
    public static final double oldlimelightAngle = 14.7734450937;

    /**
     * <p>
     * Limelight Height on Talon540 2021 13.0:
     * <ul>
     * <li>note - this is most likely wrong due to inaccurate cad measurments
     * </ul>
     */
    public static final double oldlimelightHeight = 17; // inches

    /**
     * <p>
     * Information specific to Robot Driver and ButtonMan:
     * <ul>
     * <li>driverPercentage - What percent of input is translated to robot speed
     * (slows the bot down).
     * </ul>
     */
    public static class DriveTeamInfo {
        // Driver Information - Ojas!!
        /** What percent of input speed is translated to actual speed */
        public static final double driverPercentage = 0.75;

        /** Intake Transfer Percentage */
        public static final double wristTransferPercentage = 0.15;

        /** Roller Speed */
        public static final double rollerHighPercentage = 1;
        public static final double rollerLowPercentage = 0.5;

        public static final double safeBatteryLevel = 11; // Volts

        public static final double teleopModeSwitchTimeout = 1; // one second

        public static final double AttackJoystickDeadband = 0.2;
        public static final double XboxControllerDeadband = 0.05;
    }

    /**
     * <p>
     * Information specific to 2022 RAPID REACTᵀᴹ Field:
     * <ul>
     * <li>Upper Hub Height - Height to Upper Hub in both meters and inches
     * <li>Lower Hub Height - Height to Lower Hub in both meters and inches
     * <li>Tarmac Width and Height - Tarmac Width and Height in both meters and
     * inches
     * </ul>
     */
    public static class FieldData {
        // Field Data
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

        public static enum ValidTargets {
            upper_hub,
            lower_hub,
            ball
        }
    }

    /**
     * <p>
     * PID values for different motors
     * <ul>
     * <li>LOL:
     * https://github.com/team422/FRC-22/commit/404fcaf11cdc40c7e1d994e7b2edd22b0ad7d308
     * <li>Didn't take us three years 540>422
     * </ul>
     */
    public static class PID_Values {
        /** Flywheel PID values */
        public static class flywheel {
            /** Feed Forward Term */
            public static final double kF = 0;
            /** Proportional Term */
            public static final double kP = 0;
            /** Integral term */
            public static final double kI = 0;
            /** Differentiable Term */
            public static final double kD = 0;
        }

        public static class climbRotation {
            /** Feed Forward Term */
            public static final double kF = 0;
            /** Proportional Term */
            public static final double kP = 0;
            /** Integral term */
            public static final double kI = 0;
            /** Differentiable Term */
            public static final double kD = 0;
        }

        public static class drivetrain {
            public static class translation {
                public static final double kP = 0;
                public static final double kI = 0;
                public static final double kD = 0;
            }

            public static class rotation {
                public static final double kP = 0;
                public static final double kI = 0;
                public static final double kD = 0;
            }
        }

        public static class centering {
            /** Feed Forward Term */
            public static final double kF = 0;
            /** Proportional Term */
            public static final double kP = 0;
            /** Integral term */
            public static final double kI = 0;
            /** Differentiable Term */
            public static final double kD = 0;
        }
    }

    /**
     * Both general and specific data regarding the Talon540 2022 14.0 "Triclops"
     * robot:
     * <ul>
     * <li>RobotPorts - Robot Map of all ports
     * <ul>
     * <li>Drivetrain motors
     * <li>shooter motors
     * <li>Intake motors
     * <li>Climb motors
     * </ul>
     * <li>RobotMeasurement - Physical measurement of Robot, with and without
     * bumpers
     * <li>ShooterData - Shooter and Flywheel data
     * <li>MotorData - Motor Values specific to each type of motor and their
     * respective atributes
     * 
     */
    public static class RobotData {
        /** Robot Map Ports for Motors */
        public static class RobotPorts {
            // RobotMap
            // Drive train ports
            public static final int DRIVETRAIN_FRONTRIGHT = 10;
            public static final int DRIVETRAIN_BACKRIGHT = 11;
            public static final int DRIVETRAIN_FRONTLEFT = 8;
            public static final int DRIVETRAIN_BACKLEFT = 9;

            // Shooter ports
            public static final int SHOOTER_FLY = 3;

            // Intake ports
            public static final int INTAKE_WRIST = 1;
            public static final int INTAKE_ROLLERS = 5;

            // Climb port
            public static final int CLIMBEXTENSION = 7;
            public static final int CLIMBROTATION = 6;
        }

        /** Physical Measurements */
        public static class RobotMeasurement { // One time Chirayu told me we should do all one unit for Robot Code, I
                                               // laughed and added all data in kilometers
            // Robot information
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
            public static final double flywheelHeightMeters = (flywheelHeightInches / 39.37);

            public static class WheelData {
                public static class Flywheel {
                    public static double FlywheelDiameter = 4;
                    public static double FlywheelWidth = 4.469;
                    public static double FlywheelCircumference = (FlywheelDiameter * Math.PI);
                    /** Area of one side of the wheel */
                    public static double FlywheelArea = (Math.PI * (Math.pow(FlywheelDiameter, 2))) / 4;
                }

                public static class DriveTrain {
                    public static final double wheelRadiusMeters = 0.0762;

                    public static class Omni {
                        public static double OmniDiameter = 6;
                        public static double OmniWidth = 0; // TODO: Find This
                        public static double OmniCircumference = (OmniDiameter * Math.PI);
                        /** Area of one side of the wheel */
                        public static double OmnilArea = (Math.PI * (Math.pow(OmniDiameter, 2))) / 4;
                    }

                    public static class Pneumatic {
                        public static double PneumaticDiameter = 6;
                        public static double PneumaticWidth = 0; // TODO: Find This
                        public static double PneumaticCircumference = (PneumaticDiameter * Math.PI);
                        /** Area of one side of the wheel */
                        public static double PneumaticArea = (Math.PI * (Math.pow(PneumaticDiameter, 2))) / 4;
                    }
                }
            }
        }

        /** Shooter and Flywheel specifc measurments */
        public static class ShooterData {
            // Shooter
            public static final int hubEntryAngle = 45; // What angle we want to enter the hub at

            // Flywheel
            public static final double flywheelTransferPercentage = 0.35;
        }

        /**
         * Motor Specific data
         * <ul>
         * <li>Different Motor Types
         * <ul>
         * <li>Falon500
         * <li>Lorem
         * <li>Ipsum
         * <li>Dolores
         * </ul>
         * <li>Drivetrain motors
         * <ul>
         * <li>Gear Ratio
         * <li>Max Velocity
         * </ul>
         * <li>Climber motors
         * <ul>
         * <li>Gear Ratio
         * <li>Max Velocity
         * </ul>
         * <li>Shooter motors
         * <ul>
         * <li>Gear Ratio
         * <li>Max Velocity
         * </ul>
         * <li>Intake motors
         * <ul>
         * <li>Gear Ratio
         * <li>Max Velocity
         * </ul>
         * </ul>
         */
        public static class MotorData {
            // TalonFX integrated sensor is 2048 units per rotation
            /**
             * Different Motor Types
             * <ul>
             * <li>Falon500
             * <li>Lorem
             * <li>Ipsum
             * <li>Dolores
             * </ul>
             */
            public static class motorTypes {
                public static enum Motors {
                    Drivetrain,
                    Shooter,
                    Wrist,
                    Rollers,
                    Extension,
                    Rotation
                }

                public static class Falcon500 {
                    public static final int maxRPM = 6380;
                }

                public static class M_775 {
                    public static final int maxRPM = 18700;
                }
            }

            public static double TalonDistance(double sensorCounts) {
                double motorRotations = (double) sensorCounts / 2048;
                double wheelRotations = motorRotations / 4;
                double positionMeters = wheelRotations * (Math.PI * (6 / 39.37));
                return positionMeters;
            }

            /** Drivetrain Motors */
            public static class Drivetrain {
                // Drivetrain Motors
                /** Gear Ratio of Motor */
                public static final double gearRatio = 54.0 / 20.0;
                /** Max Velocity of Motor */
                public static final double maxVelocity = (motorTypes.Falcon500.maxRPM / 600) * (2048 / gearRatio);
                public static final int maxRPM = motorTypes.Falcon500.maxRPM;

            }

            /** Climber Motors */
            public static class Climbers {
                /** Climb Rotation Motor */
                public static class ClimbRotation {
                    /** Gear Ratio of Motor */
                    public static final int gearRatio = 10;
                    /** Max Velocity of Motor */
                    public static final double maxVelocity = (motorTypes.Falcon500.maxRPM / 600) * (2048 / gearRatio);
                    public static final int maxRPM = motorTypes.Falcon500.maxRPM;
                }

                /** Climb Extension Motor */
                public static class ClimbExtension {
                    /** Gear Ratio of Motor */
                    public static final int gearRatio = 10;
                    /** Max Velocity of Motor */
                    public static final double maxVelocity = (motorTypes.Falcon500.maxRPM / 600) * (2048 / gearRatio);
                    public static final int maxRPM = motorTypes.Falcon500.maxRPM;
                }
            }

            /** Shooter Motors */
            public static class Shooter {
                /** Flywheel Motor */
                public static class Flywheel {
                    /** Gear Ratio of Motor */
                    public static final double gearRatio = (1.0 / 4.0) * (4.0 / 1.0);
                    /** Max Velocity of Motor */
                    public static final double maxVelocity = (motorTypes.Falcon500.maxRPM / 600) * (2048 / gearRatio);
                    public static final int maxRPM = motorTypes.Falcon500.maxRPM;
                }
            }

            /** Intake Motors */
            public static class Intake {
                /** Roller Motor */
                public static class Rollers {
                    /** Gear Ratio of Motor */
                    public static final int gearRatio = 7;
                    /** Max Velocity of Motor */
                    public static final double maxVelocity = (motorTypes.M_775.maxRPM / 600) * (2048 / gearRatio);
                    public static final int maxRPM = motorTypes.M_775.maxRPM;
                }

                /** Wrist Motor */
                public static class Wrist {
                    /** Gear Ratio of Motor */
                    public static final int gearRatio = 10;
                    /** Max Velocity of Motor */
                    public static final double maxVelocity = (motorTypes.Falcon500.maxRPM / 600) * (2048 / gearRatio);
                    public static final int maxRPM = motorTypes.Falcon500.maxRPM;

                }
            }
        }

        public static class AutonomousData {
            public static final double flywheelPeriod = 2; // How long to run the flywheel and centerAim before shooting
            public static final double rollersPeriod = 3; // How long till we start driving after engaging rollers

            public static final double taxiDriveSpeed = 0.5;
            public static final double taxiDriveDistance = (RobotData.RobotMeasurement.botlengthBumpersMeters
                    + FieldData.tarmacLengthMeters);
        }
    }
}
