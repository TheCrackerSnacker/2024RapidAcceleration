package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.NeckConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final Encoder heightEncoder = NeckConstants.primaryNeckEncoder;
    private final PWMSparkMax elevatorMotor = ClimberConstants.climberMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                ElevatorConstants.ELEVATOR_PID.kD);

    private static final double[] setpoints = {0, 0, -6000, -23000};

    private boolean pidOn = false;

    private double getElevatorLevelSetpoint(ElevatorConstants.ElevatorLevel level) {
        switch (level){
            case L1:
            return setpoints[0];
            case L2:
            return setpoints[1];
            case L3:
            return setpoints[2];
            case L4:
            return setpoints[3];
            default:
            return -1;
        }
    }

    public void setPIDEnabled(boolean enabled) {
        pidOn = enabled;
    }

    public void raiseElevatorToLevel(ElevatorConstants.ElevatorLevel level) {
        elevatorPID.setSetpoint(getElevatorLevelSetpoint(level));
    }

    public void resetEncoder() {
        heightEncoder.reset();
    }

    public void setSetpoint(double setpoint) {
        elevatorPID.setSetpoint(setpoint);
    }

    public void moveElevatorUp() {
        if (pidOn) {
            elevatorPID.setSetpoint(elevatorPID.getSetpoint() - ElevatorConstants.elevatorManualMovementSpeed);
        }
        else {
            elevatorMotor.set(-1);
            setSetpoint(heightEncoder.get());
        }
    }

    public void moveElevatorDown() {
        if (pidOn) {
            elevatorPID.setSetpoint(elevatorPID.getSetpoint() + ElevatorConstants.elevatorManualMovementSpeed);
        }
        else {
            elevatorMotor.set(1);
            setSetpoint(heightEncoder.get());
        }
    }

    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    public Command configureSetpointForTopOfElevatorCommand() {
        return runOnce(() -> setSetpoint(heightEncoder.get()));
    }

    public Command togglePIDEnabledCommand() {
        return runOnce(() -> setPIDEnabled(!pidOn));
    }

    public Command moveElevatorUpCommand() {
        return runOnce(() -> moveElevatorUp());
    }

    public Command moveElevatorDownCommand() {
        return runOnce(() -> moveElevatorDown());
    }

    public Command resetEncoderCommand() {
        return runOnce(() -> resetEncoder());
    }

    public void maintainLevel() {
        elevatorMotor.set(0);
        if (!pidOn) return;
        //elevatorMotor.set(elevatorPID.calculate(heightEncoder.get()));
    }

    public void periodic() {
        System.out.println("PID On? "+pidOn+", Setpoint:"+elevatorPID.getSetpoint()+", encoder: "+heightEncoder.get()+", PID output: "+elevatorPID.calculate(heightEncoder.get()));
    }
}