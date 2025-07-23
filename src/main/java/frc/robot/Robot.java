package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;

import java.io.File;

public class Robot extends TimedRobot {

  // Controllers
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  // Starting pose
  private static final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0));

  // Swerve Subsystem
  private SwerveDriveSubsystem swerve;

  @Override
  public void robotPeriodic() {
    swerve.periodic(); // update SmartDashboard, pose, etc.

    
  }

  @Override
  public void robotInit() {
    File configFolder = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      SwerveDrive swerveDrive = new SwerveDrive(configFolder);
      swerve = new SwerveDriveSubsystem(swerveDrive);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
  

  @Override
  public void teleopPeriodic() {
    // Drive the robot using joystick input
    swerve.driveCommand(
        () -> driverController.getLeftX(),
        () -> driverController.getLeftY(),
        () -> driverController.getRightX(),
        () -> driverController.getRightY()
    ).schedule();
  }
}
