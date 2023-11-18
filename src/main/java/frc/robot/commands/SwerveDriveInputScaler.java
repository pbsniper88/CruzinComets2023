package frc.robot.commands;

public class SwerveDriveInputScaler {
    private static final double OMEGA_SCALE = 1.0 / 45.0; // Or 1.0 / 30.0 for snappier action

    /**
     * Scales the inputs for the swerve drive.
     * 
     * @param strafeInput  Input for strafing (left/right movement, typically left
     *                     stick X axis).
     * @param forwardInput Input for forward/backward movement (typically left stick
     *                     Y axis).
     * @param omegaInput   Input for rotation (typically right stick X axis).
     * @return Scaled inputs as an array [scaledStrafe, scaledForward, scaledOmega].
     */
    public double[] scaleInputs(double strafeInput, double forwardInput, double omegaInput) {
        double scaledStrafe = strafeInput; // Assuming no scaling is needed for strafe
        double scaledForward = forwardInput; // Assuming no scaling is needed for forward
        double scaledOmega = omegaInput * OMEGA_SCALE; // Scale the rotation input

        return new double[] { scaledStrafe, scaledForward, scaledOmega };
    }
}
