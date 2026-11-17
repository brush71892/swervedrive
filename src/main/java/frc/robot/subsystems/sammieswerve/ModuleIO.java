package frc.robot.subsystems.sammieswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    public static class ModuleIOInputs {
        public SwerveModuleState state = new SwerveModuleState();

        public boolean driveConnected = false;
        public SwerveModulePosition drivePosition = new SwerveModulePosition();
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public double turnAbsolutePositionDeg = 0.0;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }    

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setTurnPosition(Rotation2d rotation) {}

    public default void setAngle(SwerveModuleState desiredState) {}

    public default void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {}
}