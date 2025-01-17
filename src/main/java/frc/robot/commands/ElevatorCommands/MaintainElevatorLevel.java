package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class MaintainElevatorLevel extends Command{
    
    private final ElevatorSubsystem subsystem;

    public MaintainElevatorLevel(ElevatorSubsystem subsystem) {
        this.subsystem = subsystem;
    }

    @Override
    public void execute() {
        subsystem.maintainLevel();
    }
}
