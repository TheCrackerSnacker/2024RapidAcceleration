package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final Encoder heightEncoder = NeckConstants.primaryNeckEncoder;
    private final int encoderCountsPerRot = 2048;

    private final PWMSparkMax elevatorMotor = ClimberConstants.climberMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                ElevatorConstants.ELEVATOR_PID.kD);

    private static final double[] setpoints = {0, 0, 3, 11.5};

    private boolean pidOn = true;

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

    /**
     * Gets the normalized encoder value.
     * @return The encoder reading, in rotations, with increasing positive meaning increasing elevator height.
     */
    public double getEncoderValue() {
        return heightEncoder.get() / encoderCountsPerRot * -1;
    }
    
    /**
     * Resets the encoder count to 0.
     */
    public void resetEncoder() {
        System.out.println("Resetting Encoder");
        heightEncoder.reset();
    }

    /**
     * Sets the PID height setpoint of the elevator.
     * @param setpoint The setpoint height, in normalized encoder revolutions, that the elevator will attempt to reach.
     */
    public void setSetpoint(double setpoint) {
        elevatorPID.setSetpoint(setpoint);
    }

    public double getSetpoint() {
        return elevatorPID.getSetpoint();
    }

    private PIDController getCurrentPIDController() {
        return elevatorPID;
    }

    public void moveElevatorUp() {
        if (pidOn) {
            elevatorPID.setSetpoint(elevatorPID.getSetpoint() + ElevatorConstants.elevatorManualMovementSpeed);
        }
        else {
            elevatorMotor.set(-1);
            setSetpoint(getEncoderValue());
        }
    }

    public void moveElevatorDown() {
        if (pidOn) {
            elevatorPID.setSetpoint(elevatorPID.getSetpoint() - ElevatorConstants.elevatorManualMovementSpeed);
        }
        else {
            elevatorMotor.set(1);
            setSetpoint(getEncoderValue());
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

    /**PAY ATTENTION!!!!! This method assumes that moving the elevator up is done by setting the motor speed to a positive value, and assumes that a higher encoder value means that the elevator is higher.*/
    public void runPID() {

        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            elevatorMotor.set(0);
            return;
        }

        if (isTopLimitSwitchPressed()) {

            if (getSetpoint() > getEncoderValue()) {
                setSetpoint(getEncoderValue());
            }
            // NOTE!!!! This if statement assumes that moving the elevator up is done by setting the motor speed to a negative value
            if (getCurrentPIDController().calculate(getSetpoint()) < 0) {
                elevatorMotor.set(0);
            }
            else {
                elevatorMotor.set(-getCurrentPIDController().calculate(getEncoderValue()));
            }
            return;
        }

        if (isBottomLimitSwitchPressed()) {
            resetEncoder();
            if (getSetpoint() < getEncoderValue()) {
                setSetpoint(getEncoderValue());
            }
            // NOTE!!!! This if statement assumes that moving the elevator down is done by setting the motor speed to a positive value
            if (getCurrentPIDController().calculate(getSetpoint()) > 0) {
                elevatorMotor.set(0);
            }
            else {
                elevatorMotor.set(-getCurrentPIDController().calculate(getEncoderValue()));
            }
            return;
        }
        System.out.println("Here!");
        elevatorMotor.set(-getCurrentPIDController().calculate(getEncoderValue()));
    }
    // Test change
    // This method is being used to run safety code which should be executed, no matter what.
    public void periodic() {
        System.out.println("PID On? "+pidOn+", Motor Speed: "+elevatorMotor.get()+", Setpoint:"+getSetpoint()+", encoder: "+getEncoderValue()+", PID output: "+elevatorPID.calculate(getEncoderValue()));
        System.out.println(isTopLimitSwitchPressed());
        
        // NOTE!!!! This if statement assumes that moving the elevator up is done by setting the motor speed to a negative value
        if (isTopLimitSwitchPressed() && elevatorMotor.get() < 0) {
            System.out.println("Here");
            elevatorMotor.set(0);
        }
           
        // NOTE!!!! This if statement assumes that moving the elevator down is done by setting the motor speed to a positive value
        if (isBottomLimitSwitchPressed() && elevatorMotor.get() > 0) {
            elevatorMotor.set(0);
        }
        
    }
}