package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SwerveDriveSubsystem;

import swervelib.SwerveDrive;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class Robot extends TimedRobot {

    // Controllers
    private final XboxController driverController = new XboxController(0);

    // Subsystems
    private SwerveDriveSubsystem swerve;

    @Override
    public void robotInit() {
      try {
    PIDFConfig headingPIDF = new PIDFConfig(1.0, 0.0, 0.1, 0.0);

    File swerveFolder = new File(Filesystem.getDeployDirectory(), "swerve");
    File jsonFile = new File(swerveFolder, "swerve.json");
    System.out.println("Looking for: " + jsonFile.getAbsolutePath());
    System.out.println("Exists? " + jsonFile.exists());
    
    BufferedReader reader = new BufferedReader(new FileReader(jsonFile));
    String line;
    while ((line = reader.readLine()) != null) {
        System.out.println(line);
    }
    reader.close();

    SwerveDrive swerveDrive = new SwerveParser(swerveFolder).createSwerveDrive(2);
    swerve = new SwerveDriveSubsystem(swerveDrive);

} catch (Exception e) {
    e.printStackTrace();
    SmartDashboard.putString("Swerve Init Error", e.getMessage());
}
    }
    @Override
    public void teleopPeriodic() {
        if (swerve != null) {
            // Drive using Xbox joysticks
            swerve.driveCommand(
                () -> driverController.getLeftX(),
                () -> driverController.getLeftY(),
                () -> driverController.getRightX(),
                () -> driverController.getRightY()
            ).schedule();
        }
    }
}
