package teamcode.simulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import trclib.motor.TrcMotor;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

public class TrcMapleSimMotor extends TrcMotor {
    /**
     * MapleSim calls this controller on every 4 ms physics sub-tick. Keeping the steering position loop here prevents
     * the module from running open-loop between 20 ms robot-code updates.
     */
    private static class SteerPositionController implements SimulatedMotorController {
        private static final double POSITION_KP_VOLTS_PER_RADIAN = 6.0;
        private static final double VELOCITY_KD_VOLTS_PER_RAD_PER_SEC = 0.2;

        private final SimulatedMotorController.GenericMotorController voltageController;
        private Double targetRadians = null;
        private double feedForwardVolts = 0.0;
        private double maxVoltage = 12.0;

        SteerPositionController(SimulatedMotorController.GenericMotorController voltageController) {
            this.voltageController = voltageController;
        }

        void requestVoltage(Voltage voltage) {
            targetRadians = null;
            voltageController.requestVoltage(voltage);
        }

        void requestPosition(double targetRotations, Double powerLimit, double feedForward, double batteryVoltage) {
            targetRadians = targetRotations * 2.0 * Math.PI;
            feedForwardVolts = feedForward * batteryVoltage;
            maxVoltage = Math.abs((powerLimit != null ? powerLimit : 1.0) * batteryVoltage);
        }

        @Override
        public Voltage updateControlSignal(
            Angle mechanismAngle, AngularVelocity mechanismVelocity, Angle encoderAngle,
            AngularVelocity encoderVelocity)
        {
            if (targetRadians != null) {
                // The target supplied by TrcSwerveModule is already the nearest continuous revolution. Modulus is an
                // additional guard against a discontinuity if a target crosses the +/-180 degree boundary.
                double errorRadians = MathUtil.angleModulus(
                    targetRadians - mechanismAngle.in(edu.wpi.first.units.Units.Radians));
                double velocityRadiansPerSecond =
                    mechanismVelocity.in(edu.wpi.first.units.Units.RadiansPerSecond);
                double outputVolts =
                    POSITION_KP_VOLTS_PER_RADIAN * errorRadians -
                    VELOCITY_KD_VOLTS_PER_RAD_PER_SEC * velocityRadiansPerSecond +
                    feedForwardVolts;
                voltageController.requestVoltage(
                    Volts.of(MathUtil.clamp(outputVolts, -maxVoltage, maxVoltage)));
            }
            return voltageController.updateControlSignal(
                mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
        }
    }

    private final SimulatedMotorController.GenericMotorController motorController;
    private final SteerPositionController steerPositionController;
    private final SwerveModuleSimulation moduleSimulation;
    private final boolean isDrive;
    private boolean inverted = false;
    // FrcSwerveDrive passes WPILib module angles through directly, so the simulation adapter exposes CCW-positive
    // steering feedback as well.
    private final double wpiToTrcSign;
    private double maxMotorVelocity = 1.0;

    public TrcMapleSimMotor(String name, SwerveModuleSimulation moduleSimulation, boolean isDrive) {
        super(name);
        this.moduleSimulation = moduleSimulation;
        this.isDrive = isDrive;
        this.wpiToTrcSign = 1.0;
        if (isDrive) {
            motorController = moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));
            steerPositionController = null;
        } else {
            motorController = new SimulatedMotorController.GenericMotorController(
                moduleSimulation.getSteerMotorConfigs().motor).withCurrentLimit(Amps.of(20));
            steerPositionController = new SteerPositionController(motorController);
            moduleSimulation.useSteerMotorController(steerPositionController);
        }
    }

    /**
     * Sets the simulated motor free speed in raw sensor rotations per second. This is used to turn a native velocity
     * request into the voltage fraction expected by MapleSim's generic motor controller.
     */
    public void setMaxMotorVelocity(double maxMotorVelocity) {
        this.maxMotorVelocity = Math.abs(maxMotorVelocity);
    }

    @Override
    public void setMotorPower(double power) {
        double appliedPower = (inverted ? -1.0 : 1.0) * wpiToTrcSign * power;
        Voltage voltage = Volts.of(appliedPower * RobotController.getBatteryVoltage());
        if (steerPositionController != null) {
            steerPositionController.requestVoltage(voltage);
        } else {
            motorController.requestVoltage(voltage);
        }
    }

    @Override
    public double getMotorPower() {
        double appliedPower = motorController.getAppliedVoltage().in(Volts) / RobotController.getBatteryVoltage();
        appliedPower *= wpiToTrcSign;
        return inverted ? -appliedPower : appliedPower;
    }

    @Override
    public double getMotorVelocity() {
        double velocity = isDrive
            ? moduleSimulation.getDriveEncoderUnGearedSpeed().in(RotationsPerSecond)
            : moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RotationsPerSecond);
        velocity *= wpiToTrcSign;
        return inverted ? -velocity : velocity;
    }

    @Override
    public double getMotorPosition() {
        double position = isDrive
            ? moduleSimulation.getDriveEncoderUnGearedPosition().in(Rotation)
            : moduleSimulation.getSteerAbsoluteAngle().in(Rotation);
        position *= wpiToTrcSign;
        return inverted ? -position : position;
    }

    @Override
    public void resetMotorPosition(double position) { /* MapleSim manages this internally */ }

    @Override
    public void setMotorInverted(boolean inverted) { this.inverted = inverted; }

    @Override
    public boolean isMotorInverted() { return inverted; }

    @Override
    public double getMotorCurrent() { return 0.0; /* Current sensing not supported in sim */ }

    @Override
    public double getBusVoltage() { return RobotController.getBatteryVoltage(); }

    @Override
    public void setMotorVelocityPidCoefficients(TrcPidController.PidCoefficients pidCoeffs) {
        // Velocity control is implemented by setMotorVelocity below.
    }

    @Override
    public void setMotorPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeffs) {
        // The MapleSim-native steering controller has gains expressed in physical voltage/radian units above.
    }

    @Override
    public void setMotorCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeffs) {
        throw new UnsupportedOperationException("Current PID not supported in sim.");
    }

    @Override
    public void setMotorVelocity(double velocity, double acceleration, double feedForward) {
        setMotorPower(TrcUtil.clipRange(velocity / maxMotorVelocity));
    }

    @Override
    public void setMotorPosition(double position, Double powerLimit, double velocity, double feedForward) {
        if (steerPositionController == null) {
            throw new UnsupportedOperationException("Drive position control is not supported in simulation.");
        }
        double direction = (inverted ? -1.0 : 1.0) * wpiToTrcSign;
        steerPositionController.requestPosition(
            position / direction, powerLimit, feedForward * direction, RobotController.getBatteryVoltage());
    }
    @Override
    public void setMotorCurrent(double current) {
        throw new UnsupportedOperationException("Current control not supported in sim.");
    }

    @Override
    public TrcPidController.PidCoefficients getMotorVelocityPidCoefficients() {
        throw new UnsupportedOperationException();
    }

    @Override
    public TrcPidController.PidCoefficients getMotorPositionPidCoefficients() {
        throw new UnsupportedOperationException();
    }

    @Override
    public TrcPidController.PidCoefficients getMotorCurrentPidCoefficients() {
        throw new UnsupportedOperationException();
    }

    @Override
    public TrcPidController.FFCoefficients getMotorVelocityFFCoefficients() {
        throw new UnsupportedOperationException();
    }

    @Override
    public TrcPidController.FFCoefficients getMotorPositionFFCoefficients() {
        throw new UnsupportedOperationException();
    }

    @Override
    public TrcPidController.FFCoefficients getMotorCurrentFFCoefficients() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMotorVelocityFFCoefficients(TrcPidController.FFCoefficients ffCoeffs) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMotorPositionFFCoefficients(TrcPidController.FFCoefficients ffCoeffs) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMotorCurrentFFCoefficients(TrcPidController.FFCoefficients ffCoeffs) {
        throw new UnsupportedOperationException();
    }

    @Override public void enableMotorRevLimitSwitch(boolean normalClose)  { }
    @Override public void enableMotorFwdLimitSwitch(boolean normalClose)  { }
    @Override public void disableMotorRevLimitSwitch()                    { }
    @Override public void disableMotorFwdLimitSwitch()                    { }
    @Override public void setMotorRevLimitSwitchInverted(boolean inv)     { }
    @Override public void setMotorFwdLimitSwitchInverted(boolean inv)     { }
    @Override public boolean isMotorRevLimitSwitchActive()                { return false; }
    @Override public boolean isMotorFwdLimitSwitchActive()                { return false; }
    @Override public boolean isMotorRevLimitSwitchEnabled()               { return false; }
    @Override public boolean isMotorFwdLimitSwitchEnabled()               { return false; }
    @Override public void setMotorRevSoftPositionLimit(Double limit) { }
    @Override public void setMotorFwdSoftPositionLimit(Double limit) { }
    @Override public void setMotorPositionSensorInverted(boolean inv) { }
    @Override public boolean isMotorPositionSensorInverted()          { return false; }
    @Override public void resetFactoryDefault()                              { }
    @Override public void setBrakeModeEnabled(boolean enabled)               { }
    @Override public void setOpenLoopRampRate(double rampTime)               { }
    @Override public void setCloseLoopRampRate(double rampTime)              { }
    @Override public void setCurrentLimit(double l, double t, double time)   { }
    @Override public void setStatorCurrentLimit(double limit)                { }
}
