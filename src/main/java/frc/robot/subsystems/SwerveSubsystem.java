package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.things.SwerveModule;

public class SwerveSubsystem extends SubsystemBase {

  public static SwerveSubsystem instance;
  public SwerveModule[] swerveModules;
  public SwerveDriveOdometry swerveOdometry;

  public SwerveDrivePoseEstimator poseEstimator;
  public AHRS gyro;
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  
  private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];

  public SwerveSubsystem() {
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.zeroYaw();

    for (int i=0; i<4; i++) {
      lastModulePositions[i] = new SwerveModulePosition(0, rawGyroRotation);
    }

    swerveModules = new SwerveModule[] {
      new SwerveModule(0, Constants.ModConstants.Mod0.constants),
      new SwerveModule(1, Constants.ModConstants.Mod1.constants),
      new SwerveModule(2, Constants.ModConstants.Mod2.constants),
      new SwerveModule(3, Constants.ModConstants.Mod3.constants)
    };

    resetModules();

    swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, rawGyroRotation, getModulePositions());
    
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

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(swerveModuleStates[module.id], isOpenLoop);
    }
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, true);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.kMaxTeleAngularSpeed);

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.id], false);
    }
  }

  public Rotation2d getYaw() {
    return gyro.getRotation2d();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule module : swerveModules) {
      positions[module.id] = module.getModulePosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : swerveModules) {
      states[module.id] = module.getState();
    }
    return states;
  }

  public static SwerveSubsystem getInstance() {
    if (instance == null) {
      instance = new SwerveSubsystem();
    }
    return instance;
  }

  private void resetModules() {
    for (SwerveModule module : swerveModules) {
      module.reset();
    }
  }

  public void resetGyro() {
    gyro.reset();
    rawGyroRotation = Rotation2d.kZero;
  }

  public void offsetGyro(double offset) {
    gyro.setAngleAdjustment(offset);
  }

  @Override
  public void periodic() {
    SwerveModulePosition[] currentModulePositions = new SwerveModulePosition[4];
    SwerveModulePosition[] modulePositionChanges = new SwerveModulePosition[4];

    for (SwerveModule module : swerveModules) {
      int id = module.getModuleID();
      currentModulePositions[id] = module.getModulePosition();
      modulePositionChanges[id] = new SwerveModulePosition(currentModulePositions[id].distanceMeters - lastModulePositions[id].distanceMeters, currentModulePositions[id].angle);
      lastModulePositions[id] = currentModulePositions[id];
    }

    if (gyro.isConnected()) {
      rawGyroRotation = getYaw();
    }
    else {
      Twist2d twist = SwerveConstants.swerveKinematics.toTwist2d(modulePositionChanges);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }
  }
}
