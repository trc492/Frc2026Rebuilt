package teamcode.simulation;
import trclib.sensor.TrcGyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import trclib.timer.TrcTimer;

public class TrcSimGyro extends TrcGyro
{
    private final GyroSimulation gyroSimulation;

    public TrcSimGyro(GyroSimulation gyroSimulation) {
        super("SimulatedGyro", 1, GYRO_HAS_Z_AXIS, null);
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public SensorData<Double> getRawXData(DataType dataType) {
        throw new UnsupportedOperationException("Gyro does not support x-axis.");
    }

    @Override
    public SensorData<Double> getRawYData(DataType dataType) {
        throw new UnsupportedOperationException("Gyro does not support y-axis.");
    }

    @Override
    public SensorData<Double> getRawZData(DataType dataType) {
        double value = dataType == DataType.ROTATION_RATE
            ? gyroSimulation.getMeasuredAngularVelocity().in(DegreesPerSecond)
            : gyroSimulation.getGyroReading().getDegrees();
        // MapleSim uses WPILib CCW+; TrcLib expects CW+.
        value = -value;
        return new SensorData<>(TrcTimer.getCurrentTime(), value);
    }

    @Override
    public void resetZIntegrator() {
        gyroSimulation.setRotation(new Rotation2d(0));
    }
    private class GyroInfo implements Sendable
    {
        @Override
        public void initSendable(SendableBuilder builder)
        {
            builder.setSmartDashboardType("Simulated Gyro");
            builder.addDoubleProperty("Value", () -> getZHeading().value, null);
        }   //initSendable

    }   //class GyroInfo

    /**
     * This method creates a GyroInfo object and returns it.
     *
     * @return created GyroInfo object.
     */
    public Sendable getGyroSendable()
    {
        GyroInfo gyroInfo = new GyroInfo();
        SendableRegistry.setName(gyroInfo, toString());
        return gyroInfo;
    }   //getGyroSendable
}   //class TrcSimGyro
