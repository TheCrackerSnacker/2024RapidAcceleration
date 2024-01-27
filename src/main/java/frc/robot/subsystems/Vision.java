package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Vision extends SubsystemBase {

    private final NetworkTable table;
    private final SwerveSubsystem swerveSubsystem;

    public Vision(SwerveSubsystem swerveSubsystem) {
       this.swerveSubsystem = swerveSubsystem;

       table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void alignWithAprilTag() {
        NetworkTableEntry ty = table.getEntry("ty");
        double verticalOffset = ty.getDouble(0.0);

        System.out.println(verticalOffset);



        if (verticalOffset > 0) {
             swerveSubsystem.drive(new Translation2d(0.0, verticalOffset), 0.0, true);
        } else if (verticalOffset < 0) {
             swerveSubsystem.drive(new Translation2d(0.0, -verticalOffset), 0.0, true);
        }
    }

    public void periodic() {

    }
}