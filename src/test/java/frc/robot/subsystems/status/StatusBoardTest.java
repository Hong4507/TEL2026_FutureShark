package frc.robot.subsystems.status;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.Test;

/** §8.9 READY 條件。 */
class StatusBoardTest {
  @Test
  void readyWhenEverythingOk() {
    assertTrue(StatusBoard.readyProblems(List.of(), 12.8, 15, true).isEmpty());
  }

  @Test
  void eachConditionBlocksReady() {
    assertEquals(List.of("IMU"), StatusBoard.readyProblems(List.of("IMU"), 12.8, 15, true));
    assertEquals(1, StatusBoard.readyProblems(List.of(), 12.5, 15, true).size()); // 門檻是 > 12.5
    assertEquals(1, StatusBoard.readyProblems(List.of(), 12.8, 10, true).size()); // 要 > 10 fps
    assertEquals(List.of("Auto 未選"), StatusBoard.readyProblems(List.of(), 12.8, 15, false));
  }

  @Test
  void lowBatteryAlertIsReplacedByReadyMessage() {
    List<String> r = StatusBoard.readyProblems(List.of("電池低"), 11.0, 15, true);
    assertEquals(1, r.size());
    assertTrue(r.get(0).startsWith("電池"));
  }
}
