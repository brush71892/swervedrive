

public interface ModuleIO {
    public static class ModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnceted = false;
        public Rotation2d turnAbsolutePosition = Rotation2d.kZero;
        public Rotation2d turnPosition = Rotation2d.kZero;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }    

    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDriveOpenLoop(double output) {}

    public default void setTurnOpenLoop(double output) {}

    public default void setDriveVelocity(double driveVelocityRadPerSec) {}

    public default void setTurnPosition(Rotation2d rotation) {}
}