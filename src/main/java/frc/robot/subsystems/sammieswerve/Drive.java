package frc.robot.subsystems.sammieswerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.sammieswerve.GyroIO.GyroIOInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {

  private final GyroIO gyroIO;
  private Module[] modules = new Module[4];
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];

  public SwerveDriveOdometry swerveOdometry;
  public SwerveDrivePoseEstimator poseEstimator;
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  public Drive(GyroIO gyroIO, ModuleIO frontLeftIO, ModuleIO frontRightIO, ModuleIO backLeftIO, ModuleIO backRightIO) {
    this.gyroIO = gyroIO;

    modules = new Module[] {
      new Module(frontLeftIO, 0, Constants.ModConstants.Mod0.constants),
      new Module(frontRightIO, 1, Constants.ModConstants.Mod1.constants),
      new Module(backLeftIO, 2, Constants.ModConstants.Mod2.constants),
      new Module(backRightIO, 3, Constants.ModConstants.Mod3.constants)
    };

    resetModules();
    zeroYaw();
  }

  public void resetModules() {
    
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = 
      Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds (
          translation.getX(),
          translation.getY(),
          rotation,
          rawGyroRotation
        )
        : new ChassisSpeeds(
          translation.getX(),
          translation.getY(),
          rotation
        )
      );

    for (Module module : modules) {
      module.setDesiredState(swerveModuleStates[module.id], isOpenLoop);
    }
  }
  
  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    rawGyroRotation = gyroInputs.yawPosition;

    for (Module module : modules) {
      module.periodic();
    }

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      modulePositions[moduleIndex] = modules[moduleIndex].getModulePosition();
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] =
          new SwerveModulePosition(
      modulePositions[moduleIndex].distanceMeters, modulePositions[moduleIndex].angle);
    }
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, true);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.kMaxTeleAngularSpeed);

    for (Module module : modules) {
      module.setDesiredState(desiredStates[module.id], false);
    }
  }

  public Rotation2d getYaw() {
    return gyroIO.getRotation2d();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (Module module : modules) {
      positions[module.id] = module.getModulePosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (Module module : modules) {
      states[module.id] = module.getState();
    }
    return states;
  }

  public void setYaw(Rotation2d angle) {
    gyroIO.setYaw(angle);
    rawGyroRotation = angle;
  }

  public Command zeroYaw() {
    return Commands.runOnce(() -> this.setYaw(Rotation2d.kZero), this);
  }

  public void resetGyro() {
    gyroIO.reset();
    rawGyroRotation = Rotation2d.kZero;
  }

  public void offsetGyro(double offset) {
    gyroIO.setAngleAdjustment(offset);
  }
}
