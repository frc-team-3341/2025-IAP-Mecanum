package frc.robot.commands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoDrive extends SequentialCommandGroup {
    public AutoDrive() {
        addCommands(
            new PathPlannerAuto("Auto")
        );
    }
}
