package frc.robot.subsystems.sammieswerve;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double yawVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void setYaw(Rotation2d angle) {}

    public default Rotation2d getRotation2d() {
        return Rotation2d.kZero;
    }

    public default void reset() {}
    
    public default void setAngleAdjustment(double offset) {}
}
