package frc.robot.constants;

public class Flags {
    public static enum OperatorModes {
        /** The robot is being controlled using only the xbox controller */
        XBOX_ONLY,
        /** The robot is being controlled using only the attack joysticks */
        ATTACK_ONLY,
        /**
         * The robot is being controlled using both the xbox controller and the attack
         * joysticks
         */
        XBOX_AND_ATTACK
    }
}
