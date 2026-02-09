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

/*      Title     : demo_joint_jog.cpp
 *      Project   : moveit_servo
 *      Created   : 05/27/2023
 *      Author    : V Mohammed Ibrahim
 *
 *      Description : Example of controlling a robot through joint jog commands via the C++ API.
 *
 *      ===== PRZEWODNIK DLA STUDENTÓW PRz =====
 *      
 *      CEL PROGRAMU:
 *      Ten program demonstruje najprostszą formę sterowania - bezpośrednie komendy
 *      prędkości dla poszczególnych stawów robota (joint jogging).
 *      
 *      KLUCZOWE KONCEPCJE:
 *      - Joint Jog: Bezpośrednie sterowanie prędkością każdego stawu
 *      - Joint Space: Przestrzeń konfiguracji (kąty stawów), nie kartezjańska
 *      - Najprostsza forma sterowania - bez kinematyki odwrotnej
 *      
 *      PORÓWNANIE TRYBÓW:
 *      1. JOINT_JOG (ten program):
 *         - Sterowanie: Bezpośrednie prędkości stawów [rad/s]
 *         - Użycie: Manualne pozycjonowanie, testowanie stawów
 *         - Przykład: "Obróć staw 7 z prędkością 1.0 rad/s"
 *      
 *      2. TWIST (demo_twist.cpp):
 *         - Sterowanie: Prędkości kartezjańskie end-effectora [m/s, rad/s]
 *         - Użycie: Intuicyjne sterowanie w przestrzeni 3D
 *         - Przykład: "Przesuń chwytaka w górę z prędkością 0.1 m/s"
 *      
 *      3. POSE (demo_pose.cpp):
 *         - Sterowanie: Pozycja docelowa w przestrzeni
 *         - Użycie: Śledzenie celu, precyzyjne pozycjonowanie
 *         - Przykład: "Osiągnij pozycję (x, y, z) z orientacją (qx, qy, qz, qw)"
 *      
 *      ZASTOSOWANIA JOINT_JOG:
 *      - Testowanie zakresu ruchu stawów
 *      - Ręczne pozycjonowanie (np. przyciski +/- dla każdego stawu)
 *      - Kalibracja robota
 *      - Prosta teleoperacja
 *      
 *      DLA UNITREE G1:
 *      - Testowanie pojedynczych stawów ramienia
 *      - Manualne ustawienie pozycji początkowej
 *      - Diagnostyka i identyfikacja problemów ze stawami
 */

#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace moveit_servo;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.joint_jog_demo");
}

int main(int argc, char* argv[])
{
  // ===== KROK 1-4: Inicjalizacja (identyczna jak w demo_twist.cpp) =====
  rclcpp::init(argc, argv);

  const rclcpp::Node::SharedPtr demo_node = std::make_shared<rclcpp::Node>("moveit_servo_demo");

  const std::string param_namespace = "moveit_servo";
  const std::shared_ptr<const servo::ParamListener> servo_param_listener =
      std::make_shared<const servo::ParamListener>(demo_node, param_namespace);
  const servo::Params servo_params = servo_param_listener->get_params();

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub =
      demo_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(servo_params.command_out_topic,
                                                                         rclcpp::SystemDefaultsQoS());

  const planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
      createPlanningSceneMonitor(demo_node, servo_params);
  Servo servo = Servo(demo_node, servo_param_listener, planning_scene_monitor);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  // ===== KROK 5: Ustawienie trybu JOINT_JOG =====
  // CommandType::JOINT_JOG - bezpośrednie sterowanie prędkościami stawów.
  // To jest najprostszy tryb - nie wymaga kinematyki odwrotnej ani planowania.
  servo.setCommandType(CommandType::JOINT_JOG);
  
  // ===== KROK 6: Definicja komendy Joint Jog =====
  // JointJogCommand składa się z dwóch wektorów:
  // 1. Nazwy stawów które chcemy poruszać
  // 2. Prędkości dla tych stawów [rad/s]
  //
  // W tym przykładzie:
  // - Poruszamy tylko jednym stawem: "panda_joint7" (ostatni staw ramienia Panda)
  // - Z prędkością: 1.0 rad/s (~57.3°/s)
  //
  // UWAGA dla G1: Zamień "panda_joint7" na nazwę stawu z modelu G1,
  // np. "r_wrist_yaw" dla prawego nadgarstka.
  JointJogCommand joint_jog{ { "panda_joint7" }, { 1.0 } };
  
  // Przykład: Poruszanie wieloma stawami jednocześnie
  // JointJogCommand joint_jog{
  //     { "panda_joint6", "panda_joint7" },  // Nazwy stawów
  //     { 0.5, 1.0 }                          // Odpowiednie prędkości [rad/s]
  // };

  // ===== KROK 7: Konfiguracja pętli sterowania =====
  rclcpp::WallRate rate(1.0 / servo_params.publish_period);

  // Timeout 3 sekundy (krócej niż w demo_twist dla bezpieczeństwa)
  std::chrono::seconds timeout_duration(3);
  std::chrono::seconds time_elapsed(0);
  auto start_time = std::chrono::steady_clock::now();

  // ===== KROK 8: Główna pętla sterowania =====
  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());
  while (rclcpp::ok())
  {
    // Oblicz następny stan stawów
    // W trybie JOINT_JOG jest to bardzo proste:
    // nowa_pozycja = aktualna_pozycja + (prędkość * czas)
    const KinematicState joint_state = servo.getNextJointState(joint_jog);
    const StatusCode status = servo.getStatus();

    auto current_time = std::chrono::steady_clock::now();
    time_elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time);
    
    // Sprawdź timeout
    if (time_elapsed > timeout_duration)
    {
      RCLCPP_INFO_STREAM(LOGGER, "Timed out");
      break;
    }
    // Jeśli status jest OK (brak kolizji, w limitach), wyślij komendę
    else if (status != StatusCode::INVALID)
    {
      // Konwertuj stan stawów na wiadomość trajektorii i publikuj
      trajectory_outgoing_cmd_pub->publish(composeTrajectoryMessage(servo_params, joint_state));
    }
    
    // Poczekaj do następnej iteracji
    rate.sleep();
  }

  // ===== KROK 9: Zakończenie =====
  // Po 3 sekundach staw powinien obrócić się o ~3 radiany (~172°)
  // Kalkulacja: 1.0 rad/s * 3s = 3 rad
  RCLCPP_INFO(LOGGER, "Exiting demo.");
  rclcpp::shutdown();
}

/*
 * ===== PODSUMOWANIE DLA STUDENTÓW =====
 * 
 * Ten program pokazuje najprostsze sterowanie - JOINT_JOG.
 * 
 * ZALETY:
 * + Bardzo prosty - bezpośrednie sterowanie stawami
 * + Nie wymaga rozwiązywania kinematyki
 * + Przewidywalny - wiesz dokładnie który staw się porusza
 * + Dobry do testowania i diagnostyki
 * 
 * WADY:
 * - Trudny do użycia dla zadań w przestrzeni kartezjańskiej
 * - Operatorowi trudno myśleć o kątach stawów
 * - Nie ma automatycznego unikania kolizji w przestrzeni roboczej
 * 
 * KIEDY UŻYWAĆ:
 * ✓ Testowanie zakresu ruchu poszczególnych stawów
 * ✓ Manualna kalibracja
 * ✓ Diagnostyka problemów
 * ✓ Prosty interfejs sterowania (przyciski +/- dla każdego stawu)
 * 
 * KIEDY NIE UŻYWAĆ:
 * ✗ Zadania wymagające pozycjonowania w przestrzeni 3D
 *   (użyj TWIST lub POSE)
 * ✗ Złożone manipulacje
 *   (użyj Move Group planning)
 * ✗ Teleoperacja wymagająca intuicyjności
 *   (użyj TWIST)
 * 
 * EKSPERYMENTOWANIE:
 * Spróbuj zmienić:
 * - Nazwę stawu (np. "panda_joint6")
 * - Prędkość (np. 0.5 rad/s dla wolniejszego ruchu)
 * - Poruszać wieloma stawami jednocześnie
 * - Timeout (dłuższy czas dla większego obrotu)
 */
