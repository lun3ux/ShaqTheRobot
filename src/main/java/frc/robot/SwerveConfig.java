public package frc.robot;

import swervelib.motors.SwerveMotor;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.PIDFConfig;
import swervelib.parser.json.modules.ConversionFactorsJson;
import swervelib.SwerveModulePhysicalCharacteristics;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveConfig {
  
  // Example of front-left swerve module configuration
  public static SwerveModuleConfiguration frontLeftConfig() {
    
    // Create motor instances (replace with actual motor constructors)
    SwerveMotor frontLeftDriveMotor = new SwerveMotor(1); // Example ID 1 for drive motor
    SwerveMotor frontLeftAngleMotor = new SwerveMotor(2); // Example ID 2 for angle motor
    
    // Create encoder (example, assuming using absolute encoder CANCoder)
    SwerveAbsoluteEncoder frontLeftEncoder = new SwerveAbsoluteEncoder(3); // Example ID 3 for absolute encoder
    
    // Define conversion factors for drive motor and angle motor
    ConversionFactorsJson conversionFactors = new ConversionFactorsJson(0.5, 360.0);  // Example: 0.5 meters per rotation, 360 degrees per steering rotation
    
    // Define PIDF Configurations
    PIDFConfig anglePIDF = new PIDFConfig(0.1, 0.0, 0.0, 0.0);  // Example PIDF for angle motor
    PIDFConfig velocityPIDF = new PIDFConfig(0.05, 0.0, 0.0, 0.0);  // Example PIDF for drive motor
    
    // Define physical characteristics (e.g., wheel diameter, etc.)
    SwerveModulePhysicalCharacteristics physicalCharacteristics = new SwerveModulePhysicalCharacteristics(0.1, 0.02);  // Example wheel diameter and track width
    
    // Define the module location (distance from robot center, in meters)
    Translation2d moduleLocation = new Translation2d(0.5, 0.5);  // Example front-left module at (0.5, 0.5)
    
    // Create the SwerveModuleConfiguration for the front-left swerve module
    return new SwerveModuleConfiguration(
        frontLeftDriveMotor,
        frontLeftAngleMotor,
        conversionFactors,
        frontLeftEncoder,
        0.0,  // Angle offset in degrees (no offset)
        moduleLocation.getX(),
        moduleLocation.getY(),
        anglePIDF,
        velocityPIDF,
        physicalCharacteristics,
        false,  // Encoder not inverted
        false,  // Drive motor not inverted
        false,  // Angle motor not inverted
        "Front Left",  // Name for debugging
        true   // Use cosine compensation
    );
  }

  // Repeat for other swerve modules (front-right, rear-left, rear-right)
}
 {
    
}
