package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final Encoder heightEncoder = NeckConstants.primaryNeckEncoder;

    // when set to -1, elevator goes down. when set to 1, elevator goes up.
    private final PWMSparkMax elevatorMotor = ClimberConstants.climberMotor;

    private final DigitalInput topLimitSwitch = ElevatorConstants.topLimitSwitch;
    private final DigitalInput bottomLimitSwitch = ElevatorConstants.bottomLimitSwitch;

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                ElevatorConstants.ELEVATOR_PID.kD);
    /**@return The height encoder, with increasing elevator height being an encoder value closer to positive infinity. */
    private double readEncoderNormalized() {
        return heightEncoder.get() * -1;
    }

    public void moveElevatorUp() {
        elevatorPID.setSetpoint(elevatorPID.getSetpoint() + ElevatorConstants.elevatorManualMovementSpeed);
        //elevatorMotor.set(1);
    }

    public void moveElevatorDown() {
        elevatorPID.setSetpoint(elevatorPID.getSetpoint() - ElevatorConstants.elevatorManualMovementSpeed);
        //elevatorMotor.set(-1);
    }

    public boolean isTopLimitSwitchPressed() {
        return !topLimitSwitch.get();
    }

    public boolean isBottomLimitSwitchPressed() {
        return !bottomLimitSwitch.get();
    }

    /**ATTENTION!<p>
     * This method (and those it calls) assumes that moving the elevator up is done by
     * setting the motor speed to a negative value,
     * and assumes that a lowers encoder value means that the elevator is higher.
     */
    public void runPID() {
        elevatorMotor.set(0);
        
        if (isTopLimitSwitchPressed() && isBottomLimitSwitchPressed()) {
            //elevatorMotor.set(0);
            return;
        }

        if (isTopLimitSwitchPressed()) {
            handleTopLSPressed();
        }
        else if (isBottomLimitSwitchPressed()) {
            handleBottomLSPressed();
        }
        else {
            elevatorMotor.set(elevatorPID.calculate(readEncoderNormalized())); 
        }
    }

    private void handleTopLSPressed() {
        if (elevatorPID.getSetpoint() < heightEncoder.get()) {
            elevatorPID.setSetpoint(heightEncoder.get());
        }
        // NOTE!!!! This if statement assumes that moving the elevator up is done by setting the motor speed to a negative value
        if (elevatorPID.calculate(heightEncoder.get()) < 0) {
            elevatorMotor.set(0);
        }
        else {
            elevatorMotor.set(elevatorPID.calculate(readEncoderNormalized()));
        }
    }

    private void handleBottomLSPressed() {
        //heightEncoder.reset();

        if (elevatorPID.getSetpoint() > heightEncoder.get()) {
            elevatorPID.setSetpoint(heightEncoder.get());
        }
        // NOTE!!!! This if statement assumes that moving the elevator down is done by setting the motor speed to a positive value
        if (elevatorPID.calculate(heightEncoder.get()) > 0) {
            elevatorMotor.set(0);
            heightEncoder.get();
            heightEncoder.getDistance();
        }
        else {
            elevatorMotor.set(elevatorPID.calculate(readEncoderNormalized()));
        }
    }

    // This method is being used to run safety code which should be executed, no matter what.
    public void periodic() {
        if(!DriverStation.isEnabled())
            return;
        System.out.println( "Motor Speed: "+elevatorMotor.get()+
                            ", Setpoint:"+elevatorPID.getSetpoint()+
                            ", encoder: "+readEncoderNormalized()+
                            ", PID output: "+(elevatorPID.calculate(readEncoderNormalized()))
                            );
    }
     
    // Factory Command Section: Add all command factory methods beneath this line.

    public Command moveElevatorUpCommand() {
        return runOnce(() -> moveElevatorUp());
    }

    public Command moveElevatorDownCommand() {
        return runOnce(() -> moveElevatorDown());
    }
}