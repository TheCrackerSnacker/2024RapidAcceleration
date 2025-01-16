package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final Encoder heightEncoder = NeckConstants.primaryNeckEncoder;
    private final PWMSparkMax elevatorMotor = ClimberConstants.climberMotor;

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                ElevatorConstants.ELEVATOR_PID.kD);

    private static final double[] setpoints = {0, 1000, 2000, 3000};

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

    public void raiseElevatorToLevel(ElevatorConstants.ElevatorLevel level) {
        elevatorPID.setSetpoint(getElevatorLevelSetpoint(level));
    }

    public Command raiseElevatorToL1Command() {
        return run(() -> raiseElevatorToLevel(ElevatorLevel.L1));
    }

    public Command raiseElevatorToL2Command() {
        return run(() -> raiseElevatorToLevel(ElevatorLevel.L2));
    }

    public Command raiseElevatorToL3Command() {
        return run(() -> raiseElevatorToLevel(ElevatorLevel.L3));
    }

    public Command raiseElevatorToL4Command() {
        return run(() -> raiseElevatorToLevel(ElevatorLevel.L4));
    }

    @Override
    public void periodic() {
        elevatorMotor.set(elevatorPID.calculate(heightEncoder.get()));
    }
}