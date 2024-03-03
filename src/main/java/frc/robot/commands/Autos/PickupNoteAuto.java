package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;

public class PickupNoteAuto extends ParallelCommandGroup {

    public PickupNoteAuto(Intake intake) {
        addCommands(
            new IntakeNoteAuto(intake)
        );
    }
  }
