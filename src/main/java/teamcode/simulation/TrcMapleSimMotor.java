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
import edu.wpi.first.wpilibj.RobotController;

public class TrcMapleSimMotor extends TrcMotor {
    private final SimulatedMotorController.GenericMotorController motorController;
    private final SwerveModuleSimulation moduleSimulation;
    private final boolean isDrive;
    private boolean inverted = false;
    // MapleSim uses WPILib CCW+; TrcLib expects CW+ for steering.
    private final double wpiToTrcSign;

    public TrcMapleSimMotor(String name, SwerveModuleSimulation moduleSimulation, boolean isDrive) {
        super(name);
        this.moduleSimulation = moduleSimulation;
        this.isDrive = isDrive;
        this.wpiToTrcSign = isDrive ? 1.0 : -1.0;
        this.motorController = isDrive
            ? moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60))
            : moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));
    }

    @Override
    public void setMotorPower(double power) {
        double appliedPower = (inverted ? -1.0 : 1.0) * wpiToTrcSign * power;
        motorController.requestVoltage(Volts.of(appliedPower * RobotController.getBatteryVoltage()));
    }

    @Override
    public double getMotorPower() {
        double appliedPower = motorController.getAppliedVoltage().in(Volts) / RobotController.getBatteryVoltage();
        return inverted ? -appliedPower : appliedPower;
    }

    @Override
    public double getMotorVelocity() {
        double velocity = isDrive
            ? moduleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond)
            : moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RotationsPerSecond);
        velocity *= wpiToTrcSign;
        return inverted ? -velocity : velocity;
    }

    @Override
    public double getMotorPosition() {
        double position = isDrive
            ? moduleSimulation.getDriveWheelFinalPosition().in(Rotation)
            : TrcUtil.modulo(moduleSimulation.getSteerAbsoluteFacing().getRotations(), 1.0);
        if (!isDrive)
        {
            position = TrcUtil.modulo(-position, 1.0);
        }
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
        setVelocityPidParameters(
            new PidParams()
                .setPidCoefficients(pidCoeffs)
                .setPidControlParams(
                    velPidParams != null ? velPidParams.pidTolerance : 1.0,
                    velPidParams != null ? velPidParams.pidSettling  : 0.0,
                    true),
            null);
    }

    @Override
    public void setMotorPositionPidCoefficients(TrcPidController.PidCoefficients pidCoeffs) {
        setPositionPidParameters(
            new PidParams()
                .setPidCoefficients(pidCoeffs)
                .setPidControlParams(
                    posPidParams != null ? posPidParams.pidTolerance : 1.0,
                    posPidParams != null ? posPidParams.pidSettling  : 0.0,
                    true),
            null);
    }

    @Override
    public void setMotorCurrentPidCoefficients(TrcPidController.PidCoefficients pidCoeffs) {
        throw new UnsupportedOperationException("Current PID not supported in sim.");
    }

    @Override
    public void setMotorVelocity(double velocity, double acceleration, double feedForward) {
        // Called if software PID isn't set up yet - just apply as power - really shouldn't come here but like... 
        setMotorPower(velocity / RobotController.getBatteryVoltage());
    }

    @Override
    public void setMotorPosition(double position, Double powerLimit, double velocity, double feedForward) {}
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