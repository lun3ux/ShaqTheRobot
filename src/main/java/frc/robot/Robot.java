package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;

public class Robot extends TimedRobot {

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private XboxController driverController;

    @Override
    public void robotInit() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            File swerveFolder = new File(Filesystem.getDeployDirectory(), "swerve");
            SwerveDrive swerveDrive = new SwerveParser(swerveFolder).createSwerveDrive(4);
            swerveDriveSubsystem = new SwerveDriveSubsystem(swerveDrive);
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        driverController = new XboxController(0);
        CommandScheduler.getInstance().registerSubsystem(swerveDriveSubsystem);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // Optional: cancel autonomous or reset pose
    }

    @Override
    public void teleopPeriodic() {
        swerveDriveSubsystem.driveCommand(
            () -> driverController.getLeftX(),
            () -> driverController.getRightY(), // Strafe left/right
            () -> driverController.getRightY(), // Rotation
            true                                 // Field-relative
        ).schedule();
    }
}

