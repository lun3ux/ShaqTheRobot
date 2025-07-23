package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveDrive swerveDrive;

  // Constants
  public static final double MAX_LINEAR_SPEED = 2.5;
  public static final double MAX_ANGULAR_SPEED = Math.PI * 2;
  public static final double DEAD_BAND_THRESHOLD = 0.05;
  public static final double TRANSLATION_SCALING_FACTOR = 0.8;

  public SwerveDriveSubsystem(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  // Apply deadband to joystick inputs
  private static double applyDeadband(double value, double deadband) {
    return (Math.abs(value) > deadband) ? value : 0.0;
  }

  // Maximum velocity getters
  public double getMaximumVelocity() {
    return MAX_LINEAR_SPEED;
  }
  
  public double getMaximumAngularVelocity() {
    return MAX_ANGULAR_SPEED;
  }

  // Periodic updates for SmartDashboard (for debugging)
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot X", getPose().getX());
    SmartDashboard.putNumber("Robot Y", getPose().getY());
    SmartDashboard.putNumber("Heading", getPose().getRotation().getDegrees());
  }

  // Get the robot's pose
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  // Stop the swerve drive
  public void stop() {
    swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
  }

  // Simulation periodic (empty for now, add simulation logic if necessary)
  @Override
  public void simulationPeriodic() {
    // Simulation-specific code goes here
  }

  /**
   * Generate a drive command based on joystick input.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                              DoubleSupplier headingX, DoubleSupplier headingY) {
    return new RunCommand(() -> {
        // Apply deadband to the inputs
        double xInput = applyDeadband(translationX.getAsDouble(), DEAD_BAND_THRESHOLD);
        double yInput = applyDeadband(translationY.getAsDouble(), DEAD_BAND_THRESHOLD);

        // Scale inputs for better control
        Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(xInput, yInput), TRANSLATION_SCALING_FACTOR);

        // Calculate the target speeds for the swerve drive
        var targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(
            scaledInputs.getX(),
            scaledInputs.getY(),
            headingX.getAsDouble(),
            headingY.getAsDouble(),
            swerveDrive.getOdometryHeading().getRadians(),
            getMaximumVelocity()
        );

        // Drive the robot with the calculated speeds
        swerveDrive.drive(targetSpeeds);
    }, this);
  }
}
