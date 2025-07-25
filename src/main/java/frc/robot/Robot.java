package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;

public class Robot extends TimedRobot {

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private XboxController driverController;


    @Override
    public void robotInit() {
        try {
            File swerveFolder = new File(Filesystem.getDeployDirectory(), "swerve");
            SwerveDrive swerveDrive = new SwerveParser(swerveFolder).createSwerveDrive(4);
            swerveDriveSubsystem = new SwerveDriveSubsystem(swerveDrive);
        } catch (Exception e) {
            e.printStackTrace();
        }
        driverController = new XboxController(0);
        swerveDriveSubsystem.zeroGyro();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        if (swerveDriveSubsystem != null) {
            // Schedule the drive command with joystick inputs
            swerveDriveSubsystem.driveCommand(
                () -> driverController.getLeftY(),  // forward/backward
                () -> driverController.getLeftX(),   // strafe
                () -> driverController.getRightX(),  // rotation
                true                                // field-relative driving enabled
            ).schedule();
        }
    }
}
