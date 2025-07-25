package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;




public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;

  // constants...
  public static final double MAX_LINEAR_SPEED = 2.5;
  public static final double MAX_ANGULAR_SPEED = Math.PI * 2;
  public static final double DEAD_BAND_THRESHOLD = 0.05;
  public static final double TRANSLATION_SCALING_FACTOR = 0.8;
  
  public SwerveDriveSubsystem(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }


  public SwerveModule[] getModules() {
    return swerveDrive.getModules();
  }

  public void updateOdometry() {
    swerveDrive.updateOdometry();
}

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                              DoubleSupplier rotation, boolean isFieldRelative) {
    return new RunCommand(() -> {
      double x = applyDeadband(translationX.getAsDouble());
      double y = applyDeadband(translationY.getAsDouble());
      double rot = applyDeadband(rotation.getAsDouble());

      Translation2d scaledTranslation = SwerveMath.scaleTranslation(
          new Translation2d(x, y),
          TRANSLATION_SCALING_FACTOR
      );

      swerveDrive.drive(
          scaledTranslation,
          rot * MAX_ANGULAR_SPEED,
          isFieldRelative,
          true
      );
    }, this);
  }


  public Pose2d getPose() {
    return swerveDrive.getPose();
}

private double applyDeadband(double input) {
  return Math.abs(input) > 0.05 ? input : 0.0;
}

  @Override
  public void periodic() {
      var pose = swerveDrive.getPose();
      swerveDrive.getModulePositions();
      for (int i = 0; i < swerveDrive.getModules().length; i++) {
        var module = swerveDrive.getModules()[i];
        double rawAngle = module.getAbsoluteEncoder().getAbsolutePosition();
        double correctedAngle = swerveDrive.getModulePositions()[i].angle.getDegrees();
        SmartDashboard.putNumber("Module " + i + " Raw Abs Angle", rawAngle);
        SmartDashboard.putNumber("Module " + i + " Corrected Angle", correctedAngle);
    }
      }
    }
