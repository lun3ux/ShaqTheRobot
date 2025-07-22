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


  public static final double MAX_LINEAR_SPEED = 2.5;
  public static final double MAX_ANGULAR_SPEED = Math.PI *2;

  public SwerveDriveSubsystem(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  private static double applyDeadband(double value, double deadband) {
    return (Math.abs(value) > deadband) ? value : 0.0;
  }

  public Command exampleMethodCommand() {
    return runOnce(() -> {
    });
  }

  public double getMaximumVelocity() {
    return MAX_LINEAR_SPEED;
  }
  
  public double getMaximumAngularVelocity() {
    return MAX_ANGULAR_SPEED;
  }
  

  @Override
public void periodic() {
  SmartDashboard.putNumber("Robot X", getPose().getX());
  SmartDashboard.putNumber("Robot Y", getPose().getY());
  SmartDashboard.putNumber("Heading", getPose().getRotation().getDegrees());
}

  public Pose2d getPose() {
  return swerveDrive.getPose();
}

  public void stop() {
    swerveDrive.drive(new Translation2d(0, 0), 0, false, false);
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * 
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    return new RunCommand(() -> {
        double xInput = applyDeadband(translationX.getAsDouble(), 0.05);
        double yInput = applyDeadband(translationY.getAsDouble(), 0.05);
        Translation2d scaledInputs = SwerveMath.scaleTranslation(
            new Translation2d(xInput, yInput), 0.8);

        var targetSpeeds = swerveDrive.swerveController.getTargetSpeeds(
            scaledInputs.getX(),
            scaledInputs.getY(),
            headingX.getAsDouble(),
            headingY.getAsDouble(),
            swerveDrive.getOdometryHeading().getRadians(),
            getMaximumVelocity()
        );

        swerveDrive.drive(targetSpeeds);
    }, this);
  }
}