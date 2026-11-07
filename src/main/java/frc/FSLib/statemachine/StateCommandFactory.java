package frc.FSLib.statemachine;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class StateCommandFactory<T extends Enum<T>> {
  private final StateSubsystemBase<T> m_subsystem;

  public StateCommandFactory(StateSubsystemBase<T> subsystem) {
    m_subsystem = subsystem;
  }

  public Command build(T nextState) {
    return Commands.runEnd(
      () -> m_subsystem.transitionAction(nextState),
      () -> m_subsystem.onEndBehavior(nextState)
    ).until(() -> m_subsystem.isAtState(nextState));
  }

  public Command stay(T state) {
    return Commands.run(() -> m_subsystem.transitionAction(state));
  }
}
