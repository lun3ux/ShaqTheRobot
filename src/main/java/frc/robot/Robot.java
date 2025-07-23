package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveDriveSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;

public class Robot extends TimedRobot {

  // Controllers
  public final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);
   private static final Pose2d INITIAL_POSE = new Pose2d(0, 0, new Rotation2d(0)); // Starting at origin (0,0), heading = 0 degrees.\
  
  // Swerve Drive Subsystem
  SwerveDriveConfiguration swerveDriveConfiguration = new SwerveDriveConfiguration(null, null, null, isAutonomous(), null, null);
  private SwerveDriveSubsystem swerve;

  @Override
  public void robotInit() {
  }
    // Initialize the SwerveDriveSubsystem with the appropriate swerveDrive object
    SwerveDrive swerveDrive = new SwerveDrive(swerveDriveConfiguration, null, SwerveDriveSubsystem.MAX_LINEAR_SPEED, INITIAL_POSE);
    swerve = new SwerveDriveSubsystem(swerveDrive);
  }

  // Called every 20 ms, runs the Scheduler (which schedules and runs commands)
  @Override
  public void robotPeriodic() {
    // Optionally, put debug information on the SmartDashboard
    swerve.periodic();
  }

  @Override
  public void disabledInit() {
    // Logic for when the robot is disabled
  }

  @Override
  public void disabledPeriodic() {
    // Any periodic logic during disabled mode (diagnostics, etc.)
  }

  @Override
  public void autonomousInit() {
    // Schedule autonomous commands if you have any
  }

  @Override
  public void autonomousPeriodic() {
    // Autonomous periodic logic
  }

  @Override
  public void teleopInit() {
    // Logic for when teleop starts
  }

  @Override
  public void teleopPeriodic() {
    // Get joystick inputs
    double translationX = driverController.getLeftX();  // X-axis for translation
    double translationY = driverController.getLeftY();  // Y-axis for translation
    double headingX = driverController.getRightX();     // X-axis for rotation
    double headingY = driverController.getRightY();     // Y-axis for rotation (if used)

    // Apply joystick inputs to the swerve drive
    swerve.driveCommand(
      () -> translationX, // Translation in X direction (DoubleSupplier)
      () -> translationY, // Translation in Y direction (DoubleSupplier)
      () -> headingX,     // Rotation speed based on X axis
      () -> headingY      // (Optionally) rotation speed based on Y axis (if used)
  ).schedule();  // Schedules the command to run
}
}

  @Override
  public void testInit() {
    // Cancel all running commands at the start of test mode
  }

  @Override
  public void testPeriodic() {
    // Periodic logic for test mode
  }

  @Override
  public void simulationInit() {
    // Initialize simulation (if necessary)
  }

  @Override
  public void simulationPeriodic() {
    // Periodic simulation logic (if necessary)
  }
}
