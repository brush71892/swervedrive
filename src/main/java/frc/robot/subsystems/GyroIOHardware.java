

public class GyroIOHardware implements GyroIO{
    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI, (byte) DriveConstants.kOdometryFrequency);

    public GyroIOHardware() {}

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.yawPosition = Rotation2d.fromDegrees(-navx.getYaw());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navx.getRawGyroZ());
    }

    @Override
    public void setYaw(Rotation2d angle) {
        navx.setAngleAdjustment(angle.getDegrees());
    }
}
