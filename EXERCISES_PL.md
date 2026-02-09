# Ćwiczenia praktyczne MoveIt 2 - dla studentów PRz

## Spis treści
1. [Ćwiczenie 1: Pierwsze kroki](#ćwiczenie-1-pierwsze-kroki)
2. [Ćwiczenie 2: Manipulacja obiektami](#ćwiczenie-2-manipulacja-obiektami)
3. [Ćwiczenie 3: Planowanie trajektorii](#ćwiczenie-3-planowanie-trajektorii)
4. [Ćwiczenie 4: Sterowanie w czasie rzeczywistym](#ćwiczenie-4-sterowanie-w-czasie-rzeczywistym)
5. [Projekt końcowy: Pick-and-Place](#projekt-końcowy-pick-and-place)

---

## Przygotowanie środowiska

Przed rozpoczęciem ćwiczeń upewnij się, że:

```bash
# 1. ROS 2 jest zainstalowany i skonfigurowany
source /opt/ros/humble/setup.bash

# 2. Workspace jest zbudowany
cd ~/moveit2_ws
colcon build
source install/setup.bash

# 3. MoveIt działa
ros2 launch moveit2_tutorials demo.launch.py
```

---

## Ćwiczenie 1: Pierwsze kroki

### Cele dydaktyczne:
- Zrozumienie podstaw MoveIt API
- Nauka planowania do nazwanych pozycji
- Praktyka z interfejsem RViz

### Zadanie 1.1: Uruchomienie demo i eksploracja

**Czas: 15 minut**

1. **Uruchom demonstrację:**
   ```bash
   # Terminal 1
   ros2 launch moveit2_tutorials demo.launch.py
   ```

2. **Zbadaj interfejs RViz:**
   - Zaznacz "Planning" w lewym panelu
   - Znajdź interaktywne markery (kolorowe strzałki/kółka)
   - Spróbuj przeciągnąć marker - robot pokaże podgląd pozycji
   - Kliknij "Plan" - zobaczysz zaplanowaną trajektorię
   - Kliknij "Execute" - robot wykona ruch

3. **Zadania do wykonania:**
   - [ ] Przesuń end-effector w 5 różnych miejsc
   - [ ] Zaobserwuj jak planner unika samokoli
   - [ ] Zaplanuj ruch "niemożliwy" (poza zasięgiem) - co się dzieje?

4. **Pytania kontrolne:**
   - Czym różni się "Plan" od "Execute"?
   - Co oznacza komunikat "Planning failed"?
   - Ile stopni swobody ma robot Panda?

### Zadanie 1.2: Pierwszy program Python

**Czas: 30 minut**

Uruchom dostarczony skrypt demonstracyjny:

```bash
# Terminal 2 (przy działającym demo.launch.py)
cd ~/moveit2_ws
python3 demo_student_tutorial.py
```

**Zadania:**
1. Przejdź przez wszystkie demonstracje w skrypcie
2. Przeczytaj komentarze w kodzie - zrozum każdy krok
3. Zanotuj pytania jeśli coś jest niejasne

**Modyfikacje do wykonania:**

Skopiuj `demo_student_tutorial.py` jako `my_first_program.py` i wprowadź zmiany:

```python
# Zadanie: Zmień pozycję docelową w demo_2_cartesian_target()
# Obecna: x=0.4, y=0.1, z=0.5
# Twoja nowa pozycja:
target_pose.position = Point(
    x=0.3,  # 30cm przed robotem (zmniejszyliśmy)
    y=-0.1, # 10cm w lewo (zmieniliśmy znak)
    z=0.6   # 60cm w górę (zwiększyliśmy)
)
```

- [ ] Uruchom zmodyfikowany program
- [ ] Czy robot osiągnął nową pozycję?
- [ ] Spróbuj 3 różnych pozycji

**Punkty do przemyślenia:**
- Jak daleko może sięgnąć robot?
- Co się dzieje gdy cel jest poza zasięgiem?
- Jak orientacja wpływa na osiągalność pozycji?

---

## Ćwiczenie 2: Manipulacja obiektami

### Cele dydaktyczne:
- Praca z Planning Scene
- Dodawanie i usuwanie przeszkód
- Planowanie w zagraconym środowisku

### Zadanie 2.1: Dodawanie przeszkód

**Czas: 30 minut**

Utwórz nowy plik `exercise_2_obstacles.py`:

```python
#!/usr/bin/env python3

import rclpy
from moveit.planning import MoveItPy, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import time

def add_table_and_objects():
    """Dodaje stół i obiekty do sceny."""
    rclpy.init()
    
    # Inicjalizacja
    moveit = MoveItPy(node_name="obstacle_demo")
    planning_scene = PlanningSceneInterface()
    arm = moveit.get_planning_component("panda_arm")
    
    time.sleep(2)
    
    # TODO 1: Dodaj stół
    # Wskazówka: Użyj planning_scene.add_box()
    # Parametry: name, pose, size
    table_pose = PoseStamped()
    table_pose.header.frame_id = "world"
    table_pose.pose.position = Point(x=0.6, y=0.0, z=0.25)
    table_pose.pose.orientation = Quaternion(w=1.0)
    
    planning_scene.add_box(
        name="table",
        pose=table_pose,
        size=(0.8, 1.2, 0.02)  # Szerokość x Głębokość x Wysokość
    )
    
    print("✓ Stół dodany")
    time.sleep(1)
    
    # TODO 2: Dodaj 3 różne obiekty na stole
    # Eksperymentuj z różnymi kształtami i pozycjami
    
    # Obiekt 1: Cylinder (butelka)
    bottle_pose = PoseStamped()
    bottle_pose.header.frame_id = "world"
    bottle_pose.pose.position = Point(x=0.6, y=0.2, z=0.35)
    bottle_pose.pose.orientation = Quaternion(w=1.0)
    
    planning_scene.add_cylinder(
        name="bottle",
        pose=bottle_pose,
        height=0.15,
        radius=0.03
    )
    print("✓ Butelka dodana")
    time.sleep(1)
    
    # TODO 3: Zaplanuj ruch który omija przeszkody
    # Spróbuj osiągnąć pozycję blisko butelki
    
    # TODO 4: Dołącz obiekt do chwytaka
    # Symulacja chwycenia (attach_object)
    
    # TODO 5: Przenieś obiekt w inne miejsce
    
    # TODO 6: Odłącz obiekt (detach_object)
    
    rclpy.shutdown()

if __name__ == '__main__':
    add_table_and_objects()
```

**Zadania do wykonania:**

1. **Dodaj 3 różne obiekty:**
   - [ ] Pudełko (box) - 10x10x10 cm
   - [ ] Cylinder (cylinder) - wysokość 15cm, promień 3cm  
   - [ ] Drugi cylinder w innej pozycji

2. **Zaplanuj ruch omijający przeszkody:**
   ```python
   # Cel: pozycja między obiektami
   target = Point(x=0.6, y=0.15, z=0.4)
   ```

3. **Eksperymentuj z pozycjami:**
   - Umieść obiekty tak, że robot musi je ominąć
   - Sprawdź co się dzieje gdy obiekty blokują dostęp do celu

**Pytania kontrolne:**
- Jak MoveIt wykrywa kolizje?
- Co to jest Allowed Collision Matrix (ACM)?
- Kiedy używamy `attach_object` vs `add_object`?

### Zadanie 2.2: Dołączanie obiektów

**Czas: 30 minut**

Rozszerz poprzedni program o symulację chwytania:

```python
# Fragment kodu do dodania

# Krok 1: Zbliż się do obiektu
approach_pose = Point(x=0.6, y=0.2, z=0.45)  # Nad butelką
# ... zaplanuj i wykonaj ...

# Krok 2: Symuluj zamknięcie chwytaka i dołączenie obiektu
touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
planning_scene.attach_box(
    link_name="panda_hand",
    name="bottle",
    touch_links=touch_links
)
print("✓ Butelka schwytana - jest teraz częścią robota!")

# Krok 3: Przenieś w nowe miejsce
place_pose = Point(x=0.6, y=-0.3, z=0.45)
# ... zaplanuj i wykonaj ...

# Krok 4: Odłóż obiekt
planning_scene.detach_object(link_name="panda_hand", name="bottle")
print("✓ Butelka odłożona")
```

**Zadania:**
- [ ] Zaimplementuj pełny cykl: podejście → chwyć → przenieś → odłóż
- [ ] Sprawdź w RViz jak obiekt "przykleja się" do chwytaka
- [ ] Zaplanuj trajektorię z dołączonym obiektem

---

## Ćwiczenie 3: Planowanie trajektorii

### Cele dydaktyczne:
- Trajektorie kartezjańskie
- Optymalizacja ścieżki
- Smooth motion planning

### Zadanie 3.1: Rysowanie kształtów

**Czas: 45 minut**

Napisz program rysujący kwadrat w przestrzeni:

```python
#!/usr/bin/env python3

import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose, Point, Quaternion
import time

def draw_square(side_length=0.1):
    """
    Rysuje kwadrat w przestrzeni używając trajektorii kartezjańskich.
    
    Args:
        side_length: Długość boku kwadratu w metrach
    """
    rclpy.init()
    
    moveit = MoveItPy(node_name="square_drawer")
    arm = moveit.get_planning_component("panda_arm")
    
    time.sleep(2)
    
    # Punkt startowy (lewy dolny róg kwadratu)
    start_x, start_y, start_z = 0.4, 0.2, 0.4
    
    # TODO 1: Zdefiniuj 4 punkty narożników kwadratu
    corners = [
        Point(x=start_x, y=start_y, z=start_z),                      # Lewy dolny
        Point(x=start_x, y=start_y + side_length, z=start_z),       # Prawy dolny
        Point(x=start_x, y=start_y + side_length, z=start_z + side_length),  # Prawy górny
        Point(x=start_x, y=start_y, z=start_z + side_length),       # Lewy górny
        Point(x=start_x, y=start_y, z=start_z)                      # Powrót do początku
    ]
    
    # TODO 2: Dla każdego punktu:
    # - Utwórz PoseStamped z punktem i stałą orientacją
    # - Zaplanuj ruch
    # - Wykonaj trajektorię
    
    orientation = Quaternion(x=0.707, y=0.0, z=0.0, w=0.707)
    
    for i, corner in enumerate(corners):
        print(f"Ruch do narożnika {i+1}...")
        
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = "panda_link0"
        target_pose_stamped.header.stamp = moveit.get_node().get_clock().now().to_msg()
        target_pose_stamped.pose.position = corner
        target_pose_stamped.pose.orientation = orientation
        
        # Planowanie trajektorii kartezjańskiej
        # compute_cartesian_path generuje gładką ścieżkę
        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=target_pose_stamped, pose_link="panda_link8")
        
        plan_result = arm.plan()
        if plan_result and plan_result.trajectory:
            trajectory = plan_result.trajectory
            moveit.execute(trajectory, controllers=[])
            print(f"✓ Osiągnięto narożnik {i+1}")
            time.sleep(0.5)
        else:
            print(f"✗ Nie udało się zaplanować do narożnika {i+1}")
            break
    
    print("✓ Kwadrat narysowany!")
    
    rclpy.shutdown()

if __name__ == '__main__':
    draw_square(side_length=0.1)  # Kwadrat 10cm x 10cm
```

**Zadania:**

1. **Uruchom program i narysuj kwadrat:**
   - [ ] Kwadrat 10cm x 10cm
   - [ ] Kwadrat 20cm x 20cm
   - [ ] Zaobserwuj różnicę w wykonaniu

2. **Modyfikacje:**
   - [ ] Zmień na trójkąt (3 punkty)
   - [ ] Narysuj okrąg (użyj wielu punktów w okręgu)
   - [ ] Zmień orientację chwytaka podczas rysowania

3. **Eksperyment z prędkością:**
   ```python
   # Dodaj scaling prędkości
   trajectory.joint_trajectory.points[i].time_from_start *= 2.0  # Wolniej
   ```

**Pytania kontrolne:**
- Czym różni się trajektoria kartezjańska od joint-space?
- Kiedy `compute_cartesian_path` może zawieść?
- Jak wpływa liczba punktów na gładkość ruchu?

### Zadanie 3.2: Optymalizacja trajektorii

**Czas: 30 minut**

Porównaj różne plannery:

```python
# Testuj różne plannery w ompl_planning.yaml
planners_to_test = [
    "RRTConnect",     # Szybki
    "RRTstar",        # Optymalizujący
    "PRM",            # Probabilistic Roadmap
    "BKPIECE"         # Kinodynamic
]

# TODO: Dla tego samego zadania:
# 1. Zmierz czas planowania dla każdego
# 2. Porównaj długość trajektorii
# 3. Oceń płynność ruchu
```

**Zadania:**
- [ ] Zmierz czas planowania każdego plannera
- [ ] Zapisz wyniki w tabeli
- [ ] Zidentyfikuj który jest najlepszy dla danego zadania

---

## Ćwiczenie 4: Sterowanie w czasie rzeczywistym

### Cele dydaktyczne:
- MoveIt Servo
- Sterowanie prędkością
- Teleoperacja

### Zadanie 4.1: Sterowanie klawiaturą

**Czas: 45 minut**

Napisz prosty sterownik klawiszowy:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
import tty
import termios
from geometry_msgs.msg import TwistStamped

class KeyboardTeleop(Node):
    """
    Prosta teleoperacja klawiaturą dla MoveIt Servo.
    
    Sterowanie:
    - W/S: przód/tył (oś X)
    - A/D: lewo/prawo (oś Y)
    - Q/E: góra/dół (oś Z)
    - I/K: obrót wokół X
    - J/L: obrót wokół Y
    - U/O: obrót wokół Z
    - Spacja: STOP
    - ESC: Wyjście
    """
    
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publisher komend twist
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        
        # Prędkość bazowa
        self.linear_speed = 0.05  # 5 cm/s
        self.angular_speed = 0.3  # ~17 deg/s
        
        self.get_logger().info("Keyboard Teleop gotowy!")
        self.print_instructions()
    
    def print_instructions(self):
        """Wyświetla instrukcje sterowania."""
        print("\n" + "="*50)
        print("  STEROWANIE KLAWIATURĄ")
        print("="*50)
        print("  W/S: Przód/Tył")
        print("  A/D: Lewo/Prawo")
        print("  Q/E: Góra/Dół")
        print("  I/K: Obrót wokół X")
        print("  J/L: Obrót wokół Y")
        print("  U/O: Obrót wokół Z")
        print("  Spacja: STOP")
        print("  ESC: Wyjście")
        print("="*50 + "\n")
    
    def get_key(self):
        """Odczytuje pojedynczy znak z klawiatury."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key
    
    def create_twist_msg(self, vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
        """Tworzy wiadomość Twist."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "panda_link0"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        return msg
    
    def run(self):
        """Główna pętla odczytująca klawisze."""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                # Mapowanie klawiszy na komendy
                twist_msg = None
                
                if key == 'w':
                    twist_msg = self.create_twist_msg(vx=self.linear_speed)
                elif key == 's':
                    twist_msg = self.create_twist_msg(vx=-self.linear_speed)
                elif key == 'a':
                    twist_msg = self.create_twist_msg(vy=self.linear_speed)
                elif key == 'd':
                    twist_msg = self.create_twist_msg(vy=-self.linear_speed)
                elif key == 'q':
                    twist_msg = self.create_twist_msg(vz=self.linear_speed)
                elif key == 'e':
                    twist_msg = self.create_twist_msg(vz=-self.linear_speed)
                # TODO: Dodaj obsługę obrotów (i, k, j, l, u, o)
                elif key == ' ':
                    twist_msg = self.create_twist_msg()  # STOP
                    self.get_logger().info("STOP")
                elif key == '\x1b':  # ESC
                    self.get_logger().info("Wyjście...")
                    break
                
                # Publikuj komendę
                if twist_msg:
                    self.twist_pub.publish(twist_msg)
                    
        except Exception as e:
            self.get_logger().error(f"Błąd: {e}")

def main():
    rclpy.init()
    teleop = KeyboardTeleop()
    teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Zadania:**

1. **Uruchom Servo i teleoperację:**
   ```bash
   # Terminal 1: Uruchom Servo demo
   ros2 launch moveit2_tutorials demo_servo.launch.py
   
   # Terminal 2: Uruchom teleoperację
   python3 keyboard_teleop.py
   ```

2. **Uzupełnij kod:**
   - [ ] Dodaj obsługę obrotów (i, k, j, l, u, o)
   - [ ] Dodaj przycisk zwiększający/zmniejszający prędkość
   - [ ] Dodaj wyświetlanie aktualnej pozycji

3. **Eksperymenty:**
   - [ ] Steruj robotem aby narysował kwadrat
   - [ ] Sprawdź reakcję na szybkie zmiany kierunku
   - [ ] Przetestuj granice workspace

---

## Projekt końcowy: Pick-and-Place

### Cel:
Zintegruj wszystkie poznane umiejętności w kompletnym systemie pick-and-place.

### Wymagania:

**Funkcjonalne:**
1. Wykryj obiekt na stole (symulowana pozycja)
2. Zaplanuj bezkolizyjne podejście
3. Chwyć obiekt (symulacja)
4. Przenieś do miejsca docelowego
5. Odłóż obiekt
6. Powróć do pozycji home

**Techniczne:**
- Użyj Planning Scene do reprezentacji otoczenia
- Obsłuż błędy (brak IK, kolizje)
- Dodaj logging wszystkich operacji
- Wizualizuj kroki w RViz

### Szkielet implementacji:

```python
class PickAndPlaceSystem:
    """Kompletny system pick-and-place."""
    
    def __init__(self):
        # TODO: Inicjalizacja
        pass
    
    def detect_object(self):
        """Wykrywa obiekt (symulacja)."""
        # W prawdziwej aplikacji: integracja z kamerą
        return Point(x=0.6, y=0.2, z=0.27)
    
    def plan_approach(self, object_pos):
        """Planuje podejście do obiektu."""
        # TODO: Zaplanuj ruch nad obiekt
        pass
    
    def grasp(self, object_pos):
        """Chwyta obiekt."""
        # TODO: 
        # 1. Otwórz chwytaka
        # 2. Zbliż się do obiektu
        # 3. Zamknij chwytaka
        # 4. Attach object
        pass
    
    def place(self, target_pos):
        """Odkłada obiekt."""
        # TODO:
        # 1. Przenieś nad cel
        # 2. Opuść do celu
        # 3. Otwórz chwytaka
        # 4. Detach object
        pass
    
    def execute_pick_and_place(self):
        """Główna sekwencja."""
        # TODO: Zintegruj wszystkie kroki
        pass
```

### Ocena projektu:

**Podstawowa (60%):**
- [ ] Wykrywa obiekt
- [ ] Planuje ruch
- [ ] Symuluje chwytanie
- [ ] Przenosi obiekt

**Rozszerzona (80%):**
- [ ] Dodaje przeszkody do sceny
- [ ] Obsługuje błędy planowania
- [ ] Logging operacji
- [ ] Wizualizacja w RViz

**Zaawansowana (100%):**
- [ ] Integracja z prawdziwym czujnikiem (kamera)
- [ ] Sterowanie prawdziwym chwytakiem
- [ ] Optymalizacja trajektorii
- [ ] Dokumentacja kodu

---

## Podsumowanie

Po ukończeniu tych ćwiczeń powinieneś potrafić:

✅ Planować ruchy robota w MoveIt 2
✅ Pracować z Planning Scene
✅ Używać różnych trybów sterowania (named, cartesian, servo)
✅ Integrować komponenty w kompletny system
✅ Debugować i rozwiązywać problemy

## Dalsze kroki

1. Przeczytaj [oficjalne tutoriale MoveIt](https://moveit.picknik.ai/)
2. Eksperymentuj z różnymi robotami
3. Dołącz do społeczności ROS Discourse
4. Pracuj nad własnym projektem!

---

**Powodzenia! 🤖**
