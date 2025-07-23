package frc.robot;

import swervelib.motors.SwerveMotor;
import swervelib.motors.SparkMaxSwerve;
import swervelib.encoders.CANCoderSwerve;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.json.modules.ConversionFactorsJson;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.PIDFConfig;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConfig {

    // Initialize motors (assuming you're using CANSparkMax here)
    SparkMaxSwerve frontLeftDriveMotor = new SparkMaxSwerve(1, true, null);  // Motor controller for drive motor
    SparkMaxSwerve frontLeftAngleMotor = new SparkMaxSwerve(2); // Motor controller for angle motor

    // Initialize encoders (assuming you're using CANCoder)
    CANCoderSwerve frontLeftEncoder = new CANCoderSwerve(3);  // Encoder for swerve module (adjust ID as needed)

    // Create ConversionFactorsJson (check the appropriate values)
    ConversionFactorsJson conversionFactors = new ConversionFactorsJson(1.0, 1.0);  // Example values, replace with actual conversion factors

    // Create PIDF configurations (these will need to be tuned)
    PIDFConfig anglePIDF = new PIDFConfig(0.0, 0.0, 0.0, 0.0);  // Example PIDF values for angle motor (adjust as needed)
    PIDFConfig velocityPIDF = new PIDFConfig(0.0, 0.0, 0.0, 0.0);  // Example PIDF values for drive motor (adjust as needed)

    // Create Physical Characteristics (you'll need to define this class)
    SwerveModulePhysicalCharacteristics physicalCharacteristics = new SwerveModulePhysicalCharacteristics(
        0.1,  // Module radius (adjust as needed)
        0.2   // Module width (adjust as needed)
    );

    // Initialize Swerve Module Configuration
    SwerveModuleConfiguration frontLeftModuleConfig = new SwerveModuleConfiguration(
        frontLeftDriveMotor,
        frontLeftAngleMotor,
        conversionFactors,
        frontLeftEncoder,
        0.0,  // Angle offset (adjust as needed)
        0.5,  // X position in meters (adjust as needed)
        0.5,  // Y position in meters (adjust as needed)
        anglePIDF,
        velocityPIDF,
        physicalCharacteristics,
        false,  // Absolute encoder not inverted
        false,  // Drive motor not inverted
        false,  // Angle motor not inverted
        "Front Left Module",  // Name for the module
        true  // Use cosine compensation
    );

    // Similarly, you can define other modules such as frontRightModuleConfig, backLeftModuleConfig, backRightModuleConfig...
}
