/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2023, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : demo_twist.cpp
 *      Project   : moveit_servo
 *      Created   : 06/01/2023
 *      Author    : V Mohammed Ibrahim
 *
 *      Description : Example of controlling a robot through twist commands via the C++ API.
 *
 *      ===== PRZEWODNIK DLA STUDENTÓW PRz =====
 *      
 *      CEL PROGRAMU:
 *      Ten program demonstruje sterowanie robotem w czasie rzeczywistym przez
 *      komendy twist (prędkości liniowe i kątowe). Jest to kluczowa technika
 *      dla teleoperacji i reaktywnego sterowania.
 *      
 *      KLUCZOWE KONCEPCJE:
 *      - Servo: Moduł MoveIt do sterowania w czasie rzeczywistym
 *      - Twist: Reprezentacja prędkości (3D prędkość liniowa + 3D prędkość kątowa)
 *      - Planning Scene Monitor: Śledzi aktualny stan robota i otoczenia
 *      
 *      ZASTOSOWANIA:
 *      - Sterowanie joystickiem / gamepadem
 *      - Teleoperacja robota przez operatora
 *      - Reaktywne sterowanie na podstawie czujników
 *      - Delikatne pozycjonowanie (np. montaż części)
 *      
 *      DLA UNITREE G1:
 *      Możesz używać tego typu sterowania do:
 *      - Teleoperacji ramion robota
 *      - Precyzyjnego pozycjonowania chwytaków
 *      - Śledzenia ruchu człowieka (motion capture)
 */

#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

using namespace moveit_servo;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.twist_demo");
}

int main(int argc, char* argv[])
{
  // ===== KROK 1: Inicjalizacja ROS 2 =====
  // Musimy zainicjalizować system ROS przed użyciem jakichkolwiek funkcjonalności
  rclcpp::init(argc, argv);

  // ===== KROK 2: Utworzenie node ROS =====
  // Node to podstawowa jednostka programu w ROS 2. Każdy program ROS działa jako node.
  // Ten node będzie zarządzał Servo i komunikacją z kontrolerem robota.
  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");

  // ===== KROK 3: Załadowanie parametrów Servo =====
  // Servo wymaga konfiguracji (limity prędkości, nazwy topic'ów, itp.)
  // Parametry są zazwyczaj w pliku servo_config.yaml
  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  // ===== KROK 4: Utworzenie Publisher'a do wysyłania komend =====
  // Publisher wysyła wiadomości JointTrajectory do kontrolera robota.
  // To jest ostateczny output Servo - pozycje stawów do wykonania.
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());

  // ===== KROK 5: Utworzenie Planning Scene Monitor =====
  // Planning Scene Monitor śledzi:
  // - Aktualną pozycję robota (joint_states)
  // - Przeszkody w otoczeniu
  // - Model kinematyczny robota (URDF)
  // Jest konieczny do wykrywania kolizji i bezpiecznego planowania.
  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  
  // ===== KROK 6: Utworzenie obiektu Servo =====
  // Servo to główny obiekt wykonujący sterowanie w czasie rzeczywistym.
  // Konwertuje komendy twist (prędkości) na pozycje stawów.
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  // ===== KROK 7: Oczekiwanie na załadowanie sceny =====
  // Dajemy czas na załadowanie modelu robota i sceny w RViz.
  // UWAGA: W produkcyjnej aplikacji użyj synchronizacji przez topic'i,
  // a nie sleep() - to tylko dla demonstracji!
  std::this_thread::sleep_for(std::chrono::seconds(3));

  // ===== KROK 8: Ustawienie trybu sterowania =====
  // CommandType::TWIST oznacza, że będziemy wysyłać komendy prędkości.
  // Alternatywy: JOINT_JOG (bezpośrednie prędkości stawów), POSE (pozycja docelowa)
  servo.setCommandType(CommandType::TWIST);

  // ===== KROK 9: Definicja komendy twist =====
  // TwistCommand składa się z:
  // - planning_frame: układ współrzędnych odniesienia (zazwyczaj "world" lub "base_link")
  // - 6 wartości: [vx, vy, vz, ωx, ωy, ωz]
  //   * vx, vy, vz: prędkości liniowe w [m/s] wzdłuż osi X, Y, Z
  //   * ωx, ωy, ωz: prędkości kątowe w [rad/s] wokół osi X, Y, Z
  //
  // W tym przykładzie:
  // - vz = 0.1 m/s → ruch w górę z prędkością 10 cm/s
  // - ωz = 0.5 rad/s → obrót wokół osi Z z prędkością 0.5 rad/s (~28.6°/s)
  TwistCommand target_twist{ servo_params.planning_frame, { 0.0, 0.0, 0.1, 0.0, 0.0, 0.5 } };

  // ===== KROK 10: Ustawienie częstotliwości publikacji =====
  // Servo działa w pętli - musimy regularnie wysyłać komendy.
  // publish_period określa jak często (zazwyczaj 100 Hz = co 0.01s)
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  // ===== KROK 11: Ustawienie timeout'u =====
  // Dla bezpieczeństwa demo zatrzyma się po 5 sekundach.
  // W prawdziwej aplikacji można sterować w nieskończoność lub do osiągnięcia celu.
  std::chrono::seconds timeout_duration(5);
  std::chrono::seconds time_elapsed(0);
  auto start_time = std::chrono::steady_clock::now();

  // ===== KROK 12: Główna pętla sterowania =====
  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());
  while (rclcpp::ok())
  {
    // Oblicz następny stan stawów na podstawie komendy twist
    // Servo używa kinematyki i bieżącego stanu do obliczenia:
    // "jeśli chcemy poruszać się z prędkością twist, jakie pozycje stawów osiągniemy?"
    const KinematicState joint_state = servo.getNextJointState(target_twist);
    const StatusCode status = servo.getStatus();

    // Sprawdź czy upłynął czas timeout
    auto current_time = std::chrono::steady_clock::now();
    time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
    if (time_elapsed > timeout_duration)
    {
      RCLCPP_INFO_STREAM(LOGGER, "Timed out");
      break;
    }
    // Jeśli stan jest poprawny (brak kolizji, w limitach), wyślij komendę
    else if (status != StatusCode::INVALID)
    {
      // Konwertuj KinematicState na wiadomość JointTrajectory
      // i opublikuj do kontrolera robota
      trajectory_outgoing_cmd_pub->publish(composeTrajectoryMessage(servo_params, joint_state));
    }
    // Poczekaj do następnej iteracji (zgodnie z publish_period)
    rate.sleep();
  }

  // ===== KROK 13: Zakończenie =====
  RCLCPP_INFO(LOGGER, "Exiting demo.");
  rclcpp::shutdown();
}
