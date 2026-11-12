// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.Constants.RobotMode;

public class MockDS {
  private Thread m_thread;
  /**
   * 封包序號。放在欄位而不是執行緒裡，讓 stop() 後再 start() 時序號接續下去。
   * 若每次重開都從 0 開始，DS socket 會把序號倒退的封包當成重複封包擋掉（Blocking colliding packet），
   * FlySky 短暫斷線後要多等一段時間才能重新 enable。
   */
  private volatile short m_sendCount = 0;

  private void generateEnabledDsPacket(byte[] data, short sendCount) {
    data[0] = (byte) (sendCount >> 8);
    data[1] = (byte) sendCount;
    data[2] = 0x01; // general data tag
    data[3] = 0x04; // teleop enabled
    data[4] = 0x10; // normal data request
    data[5] = 0x03; // red 1 station
  }

  public Command startMockDS() {
    return Commands.runOnce(() -> start()).ignoringDisable(true);
  }

  public Command stopMockDS() {
    return Commands.runOnce(() -> {
      Constants.kRobotMode = RobotMode.kDisable;
      Logger.recordOutput("Mode", "disable");
      stop();
    }).ignoringDisable(true);
  }

  /** Start the mock DS thread. */
  public void start() {
    stop(); // 避免重複 start 時兩個執行緒同時送封包
    m_thread = new Thread(
        () -> {
          DatagramSocket socket;
          try {
            socket = new DatagramSocket();
          } catch (SocketException e1) {
            e1.printStackTrace();
            return;
          }
          InetSocketAddress addr = new InetSocketAddress("127.0.0.1", 1110);
          byte[] sendData = new byte[6];
          DatagramPacket packet = new DatagramPacket(sendData, 0, 6, addr);
          int initCount = 0;
          while (!Thread.currentThread().isInterrupted()) {
            try {
              Thread.sleep(10);
              generateEnabledDsPacket(sendData, m_sendCount++);
              // ~50 disabled packets are required to make the robot actually enable
              // 1 is definitely not enough.
              if (initCount < 50) {
                initCount++;
                sendData[3] = 0;
              }
              packet.setData(sendData);
              socket.send(packet);
            } catch (InterruptedException ex) {
              Thread.currentThread().interrupt();
            } catch (IOException ex) {
              ex.printStackTrace();
            }
          }
          socket.close();
        });
    // Because of the test setup in Java, this thread will not be stopped
    // So it must be a daemon thread
    m_thread.setDaemon(true);
    m_thread.start();
  }

  /** Stop the mock DS thread. */
  public void stop() {
    if (m_thread == null) {
      return;
    }
    m_thread.interrupt();
    try {
      m_thread.join(500);
    } catch (InterruptedException ex) {
      ex.printStackTrace();
    }
  }
}
