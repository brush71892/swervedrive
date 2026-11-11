package frc.robot;

import java.util.Map;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.things.ModuleConstants;

public final class Constants {
    

    public static class SwerveConstants {
         public static final IdleMode driveIdleMode = IdleMode.kBrake;
        public static final IdleMode angleIdleMode = IdleMode.kBrake;
        public static final double drivePower = 1;
        public static final double anglePower = .9;

        public static final boolean invertGyro = false; 

        public static final double trackWidth = Units.inchesToMeters(29); 
        public static final double wheelBase = Units.inchesToMeters(16); 
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), 
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), 
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)); 

        // gear ratios
        public static final double driveGearRatio = (6.12 / 1.0);
        public static final double angleGearRatio = ((150.0 / 7.0) / 1.0);

        // encoder stuff
        // meters per rotation
        public static final double driveRevToMeters =  wheelCircumference / (driveGearRatio);
        public static final double driveRpmToMetersPerSecond = driveRevToMeters / 60 ;
        // the number of degrees that a single rotation of the turn motor turns the wheel.
        public static final double DegreesPerTurnRotation = 360 / angleGearRatio;

        
        // motor inverts, check these
        public static final boolean angleMotorInvert = true;
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveSupplyCurrentLimit = 60;
        public static final boolean driveSupplyCurrentLimitEnable = true;
        public static final int driveSupplyCurrentThreshold = 60;
        public static final double driveSupplyTimeThreshold = 0.1;
        
        public static final boolean driveEnableCurrentLimit = false;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double turnKP = 0.015;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnKF = 0;

        /* Drive Motor PID Values */

        public static final double driveKP = 0.01; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;

        public static final double driveKS = (0.32 / 12);
        public static final double driveKV = (1.988 / 12);
        public static final double driveKA = (1.0449 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double kPhysicalMaxSpeed = 5.0;
        public static final double kMaxTeleDriveSpeed = 4.5;
        // radians per sec
        public static final double kPhysicalMaxAngularSpeed = 2 * 2 * Math.PI;
        public static final double kMaxTeleAngularSpeed = kPhysicalMaxAngularSpeed / 2;

        public static final double kMaxAngularAccelerationSpeed = 4 / Math.PI;
        public static final double kMaxTeleAngularAccelerationSpeed = kMaxAngularAccelerationSpeed / 2;
        /** Radians per Second */

        public static final double kDeadband = 0.08;

        public static final Map<Integer, Double> distances = Map.of(
        0, 0.0,
        1, 1.0,
        2, 2.0,
        3, 3.0,
        4, 4.0
        );

        public static int targetPosition = 0;
    }



    public static final class ModConstants {
    public static final class Mod0 { //frontLeft
      public static final int driveMotorID = 8; // GOOD
      public static final int angleMotorID = 1;
      public static final int canCoderID = 20; // GOOD
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.322); //238.79882812499997-180
      public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

        /* Front Right Module - Module 1 */
    public static final class Mod1 { //frontRight
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 28;
      public static final int canCoderID = 21;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.095);
      public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
        
        /* Back Left Module - Module 2 */
    public static final class Mod2 { //backLeft
      public static final int driveMotorID = 7; // GOOD
      public static final int angleMotorID = 2; 
      public static final int canCoderID = 22;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.5688);
      public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

        /* Back Right Module - Module 3 */
    public static final class Mod3 { //backRight
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.8469);
      public static final ModuleConstants constants = new ModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }
}
