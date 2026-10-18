```markdown
# ROS 2 Hardware Interface for JGA25 Motor (Arduino + libserial)

Ce dépôt contient un package ROS 2 (Humble/Jazzy) implémentant une interface matérielle (`ros2_control` SystemInterface) pour piloter un moteur à courant continu **JGA25-370** (avec encodeur) via un microcontrôleur Arduino et un driver L298N.

Ce projet a été réalisé dans le cadre de mon apprentissage de l'interfaçage matériel réel sous ROS 2 (PFE 2027).

## 🏗️ Architecture du système

Le contrôle du moteur repose sur une boucle fermée (Closed-Loop) en deux parties :

1. **Côté Microcontrôleur (Arduino)** :
   - Lecture des impulsions de l'encodeur via des interruptions matérielles (`attachInterrupt`).
   - Contrôle local du moteur avec un régulateur PID (`PID_v1.h`) pour garantir une vitesse stable.
   - Communication Série (USB) à 115200 bauds.
   - Reçoit une consigne de vitesse (`V<tick_vel>\n`) et renvoie l'état actuel (`P<position_ticks> V<current_vel>\n`).

2. **Côté ROS 2 (PC)** :
   - Interface matérielle C++ (`jga25_hardware_interface.cpp`) héritant de `hardware_interface::SystemInterface`.
   - Utilisation de la bibliothèque **`libserial`** pour une communication série robuste.
   - Conversion des unités : Ticks encodeur ➡️ Radians (et Rad/s).
   - Intégration avec `velocity_controller` de `ros2_control`.

## 📦 Dépendances

Assurez-vous d'avoir ROS 2 installé (testé sous Humble et Jazzy) ainsi que les packages suivants :

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-controller-manager \
                 ros-$ROS_DISTRO-velocity-controllers \
                 ros-$ROS_DISTRO-joint-state-broadcaster \
                 libserial-dev
```

## 🚀 Compilation

Clonez le dépôt dans le dossier `src` de votre workspace ROS 2 :

```bash
cd ~/votre_workspace/src
git clone https://github.com/lacinasidibe114-sudo/jga25_hardware.git
cd ..
colcon build --packages-select jga25_hardware
source install/setup.bash
```

## 🎮 Utilisation

**1. Lancer le contrôleur (et charger les contrôleurs automatiquement) :**
```bash
ros2 launch jga25_hardware test.launch.py
```

**2. Envoyer une commande de vitesse (dans un autre terminal) :**
```bash
source install/setup.bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0]}"
```

**3. Vérifier l'état du moteur (position et vitesse) :**
```bash
ros2 topic echo /joint_states
```

## ⚙️ Configuration

Les paramètres matériels se trouvent dans le fichier `urdf/ros2_control.xacro` :
- `device` : Le port série de l'Arduino (ex: `/dev/ttyACM0` ou `/dev/ttyUSB0`).
- `baud_rate` : La vitesse de communication (115200).
- `counts_per_rev` : Le nombre de ticks de l'encodeur par tour (actuellement 1320.0).

## 👨‍💻 Auteur

**Lacina Sidibe** 
Étudiant en Robotique (PFE 2027).
- GitHub: [@lacinasidibe114-sudo](https://github.com/lacinasidibe114-sudo)
