package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.NeckConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final Encoder primaryNeckEncoder = NeckConstants.primaryNeckEncoder;

    private final PIDController elevatorPID = new PIDController(ElevatorConstants.ELEVATOR_PID.kP,
                                                                ElevatorConstants.ELEVATOR_PID.kI,
                                                                ElevatorConstants.ELEVATOR_PID.kD);

    public void RaiseElevatorToLevel(ElevatorConstants.ElevatorLevel level) {

    }
}
