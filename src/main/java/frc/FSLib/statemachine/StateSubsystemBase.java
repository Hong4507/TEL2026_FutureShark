package frc.FSLib.statemachine;

import static org.wpilib.util.ErrorMessages.requireNonNullParam;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;

public abstract class StateSubsystemBase<T extends Enum<T>> implements Subsystem, TelemetryLoggable {
  private final Set<StateSubsystemComponent> m_components;
  private final StateMachine<T> m_machine;

  @SuppressWarnings("this-escape")
  public StateSubsystemBase() {
    m_components = new HashSet<>();
    m_machine = new StateMachine<>(this);

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public final void addComponents(StateSubsystemComponent ...components) {
    for (StateSubsystemComponent component : components) {
      m_components.add(requireNonNullParam(component, "components", "addComponent"));
    }
  }

  public final void createFSM(Map<T, List<T>> adjList, T initialState) {
    m_machine.create(
      adjList,
      requireNonNullParam(initialState, "initialState", "createFSM")
    );
  }

  public final T getCurrentState() {
    return m_machine.getCurrentState();
  }

  public final T getNextState() {
    return m_machine.getNextState();
  }

  public final T getGoalState() {
    return m_machine.getGoalState();
  }
  
  public final void setGoalState(T newGoal) {
    m_machine.setGoalState(newGoal);
  }

  public final Command setGoalStateCommand(T newGoal) {
    return Commands.defer(
      () -> Commands.runOnce(() -> setGoalState(newGoal)).andThen(Commands.waitUntil(() -> m_machine.atGoalState())), Set.of(this)
    );
  }

  protected abstract void transitionAction(T state);
  protected abstract boolean isAtState(T state);
  protected abstract void onEndBehavior(T state);

  @Override
  public final void periodic() {
    for (StateSubsystemComponent component : m_components) {
      component.periodic();
    }
    m_machine.periodic();
  }

  @Override
  public final void simulationPeriodic() {
    for (StateSubsystemComponent component : m_components) {
      component.simulationPeriodic();
    }
  }

  @Override
  public final void logTo(TelemetryTable table) {
    table.log("AtGoalState", m_machine.atGoalState());
    table.log("CurrentState", getCurrentState().name());
    table.log("NextState", getNextState().name());
    table.log("GoalState", getGoalState().name());
  }

  @Override
  public String getTelemetryType() {
    return "StateSubsystem";
  }
}
