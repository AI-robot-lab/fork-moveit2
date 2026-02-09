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

/*      Title     : demo_pose.cpp
 *      Project   : moveit_servo
 *      Created   : 06/07/2023
 *      Author    : V Mohammed Ibrahim
 *      Description : Example of controlling a robot through pose commands via the C++ API.
 *
 *      ===== PRZEWODNIK DLA STUDENTÓW PRz =====
 *      
 *      CEL PROGRAMU:
 *      Ten program demonstruje śledzenie dynamicznie zmieniającej się pozycji docelowej
 *      (pose tracking). Robot śledzi poruszający się cel w czasie rzeczywistym.
 *      
 *      KLUCZOWE KONCEPCJE:
 *      - Pose Tracking: Ciągłe śledzenie ruchomego celu
 *      - Multi-threading: Osobny wątek dla śledzenia pozycji
 *      - Mutex: Synchronizacja dostępu do współdzielonych danych
 *      - Atomic: Bezpieczna komunikacja między wątkami
 *      
 *      RÓŻNICA OD demo_twist.cpp:
 *      - twist: Wysyłamy komendy PRĘDKOŚCI (jak szybko się ruszać)
 *      - pose: Wysyłamy POZYCJĘ DOCELOWĄ (gdzie być)
 *      
 *      ZASTOSOWANIA:
 *      - Śledzenie obiektu wykrytego przez kamerę
 *      - Reaktywne dostosowywanie trajektorii
 *      - Kompensacja ruchu (np. montaż na linii produkcyjnej)
 *      - Współpraca robot-człowiek (follow-human)
 *      
 *      DLA UNITREE G1:
 *      - Śledzenie ruchu drugiej ręki (koordynacja obu ramion)
 *      - Chwytanie poruszających się obiektów
 *      - Interakcja z człowiekiem (podążanie za gestem)
 */

#include <atomic>
#include <chrono>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

using namespace moveit_servo;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.pose_demo");
}

int main(int argc, char* argv[])
{
  // ===== KROK 1: Inicjalizacja (jak w demo_twist.cpp) =====
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

  // ===== KROK 2: Synchronizacja wątków =====
  // Mutex (mutual exclusion) - zapewnia, że tylko jeden wątek na raz
  // może modyfikować target_pose. Bez tego mogłyby wystąpić race conditions.
  std::mutex pose_guard;
  
  // Atomic - zmienna bezpiecznie dostępna z wielu wątków bez mutex'a.
  // Używana do sygnalizowania "stop" wątkowi śledzącemu.
  std::atomic<bool> stop_tracking = false;

  // ===== KROK 3: Ustawienie trybu POSE =====
  // CommandType::POSE - Servo będzie śledzić pozycję docelową w przestrzeni,
  // automatycznie obliczając komendy do osiągnięcia tej pozycji.
  servo.setCommandType(CommandType::POSE);

  // ===== KROK 4: Inicjalizacja pozycji docelowej =====
  // Zaczynamy od aktualnej pozycji end-effectora.
  // W rzeczywistej aplikacji mogłaby to być pozycja z czujnika wizji.
  PoseCommand target_pose;
  target_pose.frame_id = servo_params.planning_frame;
  target_pose.pose = servo.getEndEffectorPose();

  // ===== KROK 5: Definicja funkcji śledzenia pozycji =====
  // Ta funkcja będzie działać w osobnym wątku, ciągle wysyłając komendy
  // aby robot śledził poruszający się cel.
  auto pose_tracker = [&]() {
    KinematicState joint_state;
    rclcpp::WallRate tracking_rate(1 / servo_params.publish_period);
    
    // Pętla działa aż do otrzymania sygnału stop
    while (!stop_tracking && rclcpp::ok())
    {
      {
        // Sekcja krytyczna - chroniona przez mutex
        // Tylko jeden wątek może być tutaj jednocześnie
        std::lock_guard<std::mutex> pguard(pose_guard);
        
        // Oblicz następny stan stawów aby zbliżyć się do target_pose
        joint_state = servo.getNextJointState(target_pose);
      }
      
      // Sprawdź status (czy robot może bezpiecznie wykonać ruch)
      StatusCode status = servo.getStatus();
      if (status != StatusCode::INVALID)
        trajectory_outgoing_cmd_pub->publish(composeTrajectoryMessage(servo_params, joint_state));

      tracking_rate.sleep();
    }
  };

  // ===== KROK 6: Definicja pozycji końcowej =====
  // Robot będzie śledzić ruchomy cel, aż ten osiągnie terminal_pose.
  // W tym przykładzie: obrót o 45° wokół osi Z i przesunięcie w dół o 10cm.
  Eigen::Isometry3d terminal_pose = target_pose.pose;
  terminal_pose.rotate(Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()));  // +45° obrót
  terminal_pose.translate(Eigen::Vector3d(0.0, 0.0, -0.1));  // -10cm w osi Z

  // ===== KROK 7: Uruchomienie wątku śledzącego =====
  // Tworzymy osobny wątek dla pose_tracker.
  // detach() - wątek działa niezależnie (nie czekamy na join() w tym miejscu)
  std::thread tracker_thread(pose_tracker);
  tracker_thread.detach();

  // ===== KROK 8: Konfiguracja kroków ruchu =====
  // Cel porusza się małymi krokami w każdej iteracji.
  // linear_step_size: -2mm w każdym kroku w dół (oś Z)
  Eigen::Vector3d linear_step_size{ 0.0, 0.0, -0.002 };
  
  // angular_step_size: 0.01 rad (~0.57°) obrotu wokół osi Z w każdym kroku
  Eigen::AngleAxisd angular_step_size(0.01, Eigen::Vector3d::UnitZ());

  // ===== KROK 9: Pętla aktualizacji celu =====
  // Główny wątek przesuwa cel, wątek śledzący stara się za nim nadążyć.
  rclcpp::WallRate command_rate(50);  // 50 Hz - 20ms między aktualizacjami
  RCLCPP_INFO_STREAM(LOGGER, servo.getStatusMessage());

  while (!stop_tracking && rclcpp::ok())
  {
    {
      // Sekcja krytyczna - synchronizacja z wątkiem śledzącym
      std::lock_guard<std::mutex> pguard(pose_guard);
      
      // Aktualizuj target_pose na podstawie aktualnej pozycji end-effectora
      target_pose.pose = servo.getEndEffectorPose();
      
      // Sprawdź czy osiągnęliśmy cel (w granicach tolerancji)
      const bool satisfies_linear_tolerance = target_pose.pose.translation().isApprox(
          terminal_pose.translation(), servo_params.pose_tracking.linear_tolerance);
      const bool satisfies_angular_tolerance =
          target_pose.pose.rotation().isApprox(terminal_pose.rotation(), servo_params.pose_tracking.angular_tolerance);
      
      // Jeśli osiągnęliśmy cel w obu aspektach (pozycja i orientacja), zatrzymaj
      stop_tracking = satisfies_linear_tolerance && satisfies_angular_tolerance;
      
      // Dynamicznie przesuwaj cel w kierunku terminal_pose
      // Robot będzie próbował nadążyć za tym ruchomym celem
      if (!satisfies_linear_tolerance)
        target_pose.pose.translate(linear_step_size);
      if (!satisfies_angular_tolerance)
        target_pose.pose.rotate(angular_step_size);
    }

    command_rate.sleep();
  }

  // ===== KROK 10: Zakończenie =====
  RCLCPP_INFO_STREAM(LOGGER, "REACHED : " << stop_tracking);
  stop_tracking = true;

  // Poczekaj na zakończenie wątku śledzącego
  if (tracker_thread.joinable())
    tracker_thread.join();

  RCLCPP_INFO(LOGGER, "Exiting demo.");
  rclcpp::shutdown();
}
