package frc.robot.commands.Autos;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.LauncherFeeder;

public class FireNoteAuto extends ParallelRaceGroup {

    public FireNoteAuto(Launcher launcher, LauncherFeeder launcherFeeder, Supplier<Boolean> isLauncherUpSupplier, Supplier<Boolean> hasNoteSupplier) {
        addCommands(
            new LauncherSpinUpAuto(launcher, isLauncherUpSupplier),
            new SequentialCommandGroup(
                new WaitCommand(.6),
                new FeedNoteAuto(launcherFeeder, hasNoteSupplier)
            )
        );
    }
  }
