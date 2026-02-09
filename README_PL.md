# MoveIt 2 - Framework do Planowania Ruchu Robotów

<img src="https://moveit.ai/assets/logo/moveit_logo-black.png" alt="MoveIt Logo" width="200"/>

## O projekcie MoveIt 2

**MoveIt 2** to zaawansowany framework open-source do planowania ruchu robotów w środowisku ROS 2 (Robot Operating System 2). Jest to potężne narzędzie umożliwiające programowanie i sterowanie manipulatorami robotycznymi oraz robotami mobilnymi.

### Czym jest MoveIt 2?

MoveIt 2 to kompleksowa platforma, która:
- **Planuje trajektorię ruchu** - oblicza bezpieczne ścieżki przemieszczania się efektora końcowego robota
- **Wykrywa kolizje** - zapobiega zderzeniom z przeszkodami w środowisku pracy
- **Rozwiązuje kinematykę** - przekształca docelowe pozycje w przestrzeni na kąty stawów robota
- **Integruje czujniki** - wykorzystuje kamery i skanery 3D do percepcji otoczenia
- **Steruje robotami** - komunikuje się z kontrolerami sprzętowymi robotów

## Dlaczego używamy MoveIt 2?

### Główne zalety:

1. **Łatwość użycia** - wysokopoziomowe API ukrywa złożoność algorytmów planowania
2. **Modularność** - można wybierać różne algorytmy planowania (OMPL, STOMP, Pilz)
3. **Wizualizacja** - integracja z RViz pozwala na podgląd planowanych ruchów
4. **Społeczność** - aktywna społeczność ROS i bogata dokumentacja
5. **Kompatybilność** - działa z wieloma modelami robotów przemysłowych i badawczych

## Struktura repozytorium

```
moveit2/
├── moveit_core/              # Podstawowe algorytmy (kinematyka, planowanie, kolizje)
├── moveit_ros/               # Integracja z ROS 2
│   ├── moveit_servo/        # Sterowanie w czasie rzeczywistym (Servo)
│   ├── planning/            # Komponenty planowania ruchu
│   └── visualization/       # Narzędzia wizualizacji w RViz
├── moveit_planners/         # Algorytmy planowania (OMPL, STOMP, Pilz)
├── moveit_py/               # Biblioteka Python do MoveIt 2
├── moveit_setup_assistant/  # Narzędzie konfiguracji robota
└── moveit_plugins/          # Wtyczki rozszerzające funkcjonalność
```

## Kluczowe koncepcje

### Planning Scene
**Planning Scene** to wewnętrzna reprezentacja środowiska robota, zawierająca:
- Model kinematyczny robota (URDF)
- Aktualne położenie stawów
- Przeszkody w otoczeniu
- Dozwolone kolizje

### Move Group
**Move Group** to zestaw stawów robota, które są sterowane razem jako jedna grupa (np. lewe ramię, prawa ręka). W robocie Unitree G1 mamy kilka grup: ramiona, nogi, tułów.

### Planning Request
**Planning Request** to żądanie obliczenia trajektorii z punktu A do punktu B, zawierające:
- Pozycję początkową
- Pozycję docelową (lub pozę end-effectora)
- Ograniczenia ruchu

## Instalacja

### Instalacja binarna (zalecana dla początkujących):
```bash
sudo apt update
sudo apt install ros-humble-moveit
```

### Kompilacja ze źródeł:
Szczegółowe instrukcje: [Source Build](https://moveit.ai/install-moveit2/source/)

## Pierwsze kroki

### 1. Uruchomienie demo
```bash
# Uruchom demo z robotem Panda
ros2 launch moveit2_tutorials demo.launch.py

# Otwórz RViz i zaplanuj ruch używając interaktywnych markerów
```

### 2. Podstawowy kod Python
```python
import rclpy
from moveit.planning import MoveItPy

# Inicjalizacja node ROS 2
rclpy.init()

# Utworzenie instancji MoveItPy
moveit = MoveItPy(node_name="moveit_py_demo")

# Pobranie planning component dla grupy ramienia
arm = moveit.get_planning_component("arm")

# Zaplanowanie ruchu do pozycji domowej
arm.set_start_state_to_current_state()
arm.set_goal_state(configuration_name="home")
plan_result = arm.plan()

# Wykonanie ruchu
if plan_result:
    robot_trajectory = plan_result.trajectory
    moveit.execute(robot_trajectory, controllers=[])
```

## Przydatne zasoby

### Dokumentacja
- [Oficjalne tutoriale MoveIt 2](https://moveit.picknik.ai/)
- [Dokumentacja API](https://moveit.picknik.ai/main/api/html/)
- [Przewodnik migracji z MoveIt 1](./MIGRATION.md)

### Dla studentów PRz
- [Przewodnik studenta](./STUDENT_GUIDE_PL.md) - szczegółowy przewodnik krok po kroku
- [Przewodnik Unitree G1 EDU](./UNITREE_G1_GUIDE_PL.md) - zastosowanie w projekcie z robotem humanoidalnym
- [Przykłady z komentarzami](./moveit_ros/moveit_servo/demos/) - kod demonstracyjny z polskimi wyjaśnieniami

### Społeczność
- [Forum dyskusyjne ROS](https://discourse.ros.org/)
- [GitHub Issues](https://github.com/ros-planning/moveit2/issues)
- [Discord MoveIt](https://discord.gg/moveit)

## Najczęstsze problemy początkujących

### Problem: Robot nie planuje ruchu
**Rozwiązanie**: Sprawdź czy:
- Planning Scene jest załadowany (`planning_scene_monitor`)
- Robot ma poprawnie skonfigurowane grupy (`move_group`)
- Żądana pozycja jest osiągalna (w workspace robota)

### Problem: Kolizje w planowaniu
**Rozwiązanie**: 
- Dodaj przeszkody do Planning Scene
- Sprawdź `allowed_collision_matrix` (ACM)
- Zwiększ `planning_time` dla trudniejszych scenariuszy

### Problem: Servo nie reaguje na komendy
**Rozwiązanie**:
- Sprawdź poprawność konfiguracji `servo_params.yaml`
- Upewnij się, że `planning_scene_monitor` działa
- Sprawdź topic'i komunikacji (`ros2 topic list`)

## Następne kroki

1. Przeczytaj [Przewodnik studenta](./STUDENT_GUIDE_PL.md)
2. Zapoznaj się z przykładami w `moveit_ros/moveit_servo/demos/`
3. Przejdź przez [tutoriale Unitree G1](./UNITREE_G1_GUIDE_PL.md)
4. Eksperymentuj z kodem - najlepsza nauka przez praktykę!

## Status kompilacji

Wszystkie komponenty MoveIt 2 są regularnie testowane i kompilowane dla dystrybucji ROS 2 Humble, Iron i Rolling. Status kompilacji dostępny w [głównym README](./README.md).

## Licencja

MoveIt 2 jest oprogramowaniem open-source na licencji BSD 3-Clause. Zobacz [LICENSE.txt](./LICENSE.txt) dla szczegółów.

## Podziękowania

Ten przewodnik został przygotowany dla studentów Politechniki Rzeszowskiej w ramach projektu edukacyjnego z robotem humanoidalnym Unitree G1 EDU. Dziękujemy społeczności MoveIt za stworzenie tak potężnego narzędzia edukacyjnego.
