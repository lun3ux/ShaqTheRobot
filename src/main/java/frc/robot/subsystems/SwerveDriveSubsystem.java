package frc.robot.subsystems;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.SwerveModule;


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

  // other existing methods...

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, 
                              DoubleSupplier rotation, boolean isFieldRelative) {
    return new RunCommand(() -> {
      double x = applyDeadband(translationX.getAsDouble(), DEAD_BAND_THRESHOLD);
      double y = applyDeadband(translationY.getAsDouble(), DEAD_BAND_THRESHOLD);
      double rot = applyDeadband(rotation.getAsDouble(), DEAD_BAND_THRESHOLD);

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

  public void zeroGyro() {
    swerveDrive.zeroGyro();
}

  private static double applyDeadband(double value, double deadband) {
    return (Math.abs(value) > deadband) ? value : 0.0;
  }

  @Override
  public void periodic() {
      var pose = swerveDrive.getPose();
      SmartDashboard.putNumber("Robot X", pose.getX());
      SmartDashboard.putNumber("Robot Y", pose.getY());
      SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());
  
      var modulePositions = swerveDrive.getModulePositions(); // Actual states from YAGSL
  
      for (int i = 0; i < modulePositions.length; i++) {
          var pos = modulePositions[i];
          double angleDeg = pos.angle.getDegrees();
          double distance = pos.distanceMeters; // or distanceMeters, depending on YAGSL version
  
          SmartDashboard.putNumber("Module " + i + " Angle (deg)", angleDeg);
          SmartDashboard.putNumber("Module " + i + " Distance (m)", distance);
      }
  }

}
