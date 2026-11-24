package frc.robot.subsystems.intake;

public interface IntakeIO {

    public static class IntakeIOInputs {
        public boolean leftPivotConnected = false;
        public double leftPivotVelocityRadPerSec = 0.0;
        public double leftPivotAppliedVolts = 0.0;
        public double leftPivotCurrentDrawAmps = 0.0;

        public boolean rightPivotConnected = false;
        public double rightPivotVelocityRadPerSec = 0.0;
        public double rightPivotAppliedVolts = 0.0;
        public double rightPivotCurrentDrawAmps = 0.0;

        public boolean driveConnected = false;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentDrawAmps = 0.0;
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setPivotPosition(double positionRotations) {}

    default void setPivotSpeed(double speed) {}
    
    default void setWheelSpeed(double speed) {}
}
