package teamcode.simulation;
import frclib.drivebase.FrcSwerveBase;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcEncoder;
import trclib.robotcore.TrcDbgTrace;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import trclib.dataprocessor.TrcUtil;
public class SimFrcSwerveBase extends FrcSwerveBase {
    private static final ThreadLocal<SwerveDriveSimulation> pendingSimulation = new ThreadLocal<>();
    // MapleSim module order is assumed FL, FR, BL, BR. Adjust if MapleSim returns a different order.
    private static final int[] MAPLESIM_MODULE_ORDER = {0, 1, 2, 3};
    private SwerveDriveSimulation swerveDriveSimulation;

    public static SimFrcSwerveBase create(SwerveInfo swerveInfo, SwerveDriveSimulation swerveDriveSimulation) {
        pendingSimulation.set(swerveDriveSimulation);
        try {
            return new SimFrcSwerveBase(swerveInfo);
        } finally {
            pendingSimulation.remove();
        }
    }

    private static class SimSteerEncoder implements TrcEncoder {
        private final SwerveModuleSimulation moduleSimulation;
        private double scale = 1.0;
        private double offset = 0.0;
        private double zeroOffset = 0.0;
        private boolean inverted = false;

        SimSteerEncoder(SwerveModuleSimulation moduleSimulation) {
            this.moduleSimulation = moduleSimulation;
        }

        @Override
        public void reset(double position) {
            zeroOffset = position;
        }

        @Override
        public double getRawPosition() {
            double rotations = moduleSimulation.getSteerAbsoluteFacing().getRotations();
            // MapleSim uses WPILib CCW+; TrcLib expects CW+.
            rotations = TrcUtil.modulo(-rotations, 1.0);
            return inverted ? -rotations : rotations;
        }

        @Override
        public double getScaledPosition() {
            return (getRawPosition() - zeroOffset) * scale + offset;
        }

        @Override
        public double getRawVelocity() {
            double velocity = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RotationsPerSecond);
            // MapleSim uses WPILib CCW+; TrcLib expects CW+.
            velocity = -velocity;
            return inverted ? -velocity : velocity;
        }

        @Override
        public double getScaledVelocity() {
            return getRawVelocity() * scale;
        }

        @Override
        public void setInverted(boolean inverted) {
            this.inverted = inverted;
        }

        @Override
        public boolean isInverted() {
            return inverted;
        }

        @Override
        public void setScaleAndOffset(double scale, double offset, double zeroOffset) {
            this.scale = scale;
            this.offset = offset;
            this.zeroOffset = zeroOffset;
        }
    }

    private SimFrcSwerveBase(SwerveInfo swerveInfo) {
        super(swerveInfo);
    }

    private SwerveDriveSimulation getSwerveDriveSimulation() {
        if (swerveDriveSimulation == null) {
            swerveDriveSimulation = pendingSimulation.get();
        }
        if (swerveDriveSimulation == null) {
            throw new IllegalStateException("SwerveDriveSimulation is not available for SimFrcSwerveBase.");
        }
        return swerveDriveSimulation;
    }

    private SwerveModuleSimulation getModuleByTrcIndex(int trcIndex) {
        SwerveModuleSimulation[] modules = getSwerveDriveSimulation().getModules();
        return modules[MAPLESIM_MODULE_ORDER[trcIndex]];
    }

    @Override
    protected TrcEncoder[] createSteerEncoders() {
        // MapleSim handles steer encoder internally via SwerveModuleSimulation.
        TrcEncoder[] encoders = new TrcEncoder[swerveInfo.steerEncoderNames.length];
        for (int i = 0; i < encoders.length; i++) {
            SimSteerEncoder encoder = new SimSteerEncoder(getModuleByTrcIndex(i));
            encoder.setScaleAndOffset(
                swerveInfo.steerEncoderScale, 0.0,
                swerveInfo.steerEncoderZeros != null ? swerveInfo.steerEncoderZeros[i] : 0.0);
            if (swerveInfo.steerEncoderInverted != null && swerveInfo.steerEncoderInverted.length > i) {
                encoder.setInverted(swerveInfo.steerEncoderInverted[i]);
            }
            encoders[i] = encoder;
        }
        return encoders;
    }

    @Override
    protected TrcMotor[] createSteerMotors() {
        TrcMotor[] motors = new TrcMotor[4];
        String[] names = swerveInfo.steerMotorNames;

        for (int i = 0; i < 4; i++) {
            motors[i] = new TrcMapleSimMotor(names[i], getModuleByTrcIndex(i), false);
            // motors[i].setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG, true, false, null);
            motors[i].setPositionSensorScaleAndOffset(swerveInfo.steerMotorPosScale, 0.0);
            if (swerveInfo.swerveParams != null && swerveInfo.swerveParams.steerMotorPidParams != null)
            {
                TrcMotor.PidParams baseParams = swerveInfo.swerveParams.steerMotorPidParams;
                TrcMotor.PidParams simParams = new TrcMotor.PidParams()
                    .setPidCoefficients(baseParams.pidCoeffs)
                    .setFFCoefficients(baseParams.ffCoeffs)
                    .setPidControlParams(baseParams.pidTolerance, baseParams.pidSettling, true, baseParams.enableSquid);
                motors[i].setPositionPidParameters(simParams, null);
            }
        }
        return motors;
    }
}