#include <PID_v1.h>

// ==========================================
// BROCHES (À adapter à votre câblage)
// ==========================================
#define L298N_enA 9   // PWM
#define L298N_in1 12  // Dir Motor A
#define L298N_in2 13  // Dir Motor A
#define encoder_phaseA 2 // Interruption (obligatoire sur Uno)
#define encoder_phaseB 4

// ==========================================
// VARIABLES ENCODEUR & PID
// ==========================================
unsigned int encoder_counter = 0;
String wheel_sign = "p"; 
unsigned long last_millis = 0;
const unsigned long interval = 100; // 100ms (10Hz) pour le calcul de vitesse

// PID Variables
double cmd_vel = 0.0;     // Consigne envoyée par ROS 2 (rad/s)
double meas_vel = 0.0;    // Vitesse mesurée (rad/s)
double motor_pwm = 0.0;   // Sortie PID (0-255)

// Tuning PID (Vous devrez peut-être ajuster ces valeurs pour votre JGA25)
double Kp = 11.5;
double Ki = 7.5;
double Kd = 0.1;
PID motorPID(&meas_vel, &motor_pwm, &cmd_vel, Kp, Ki, Kd, DIRECT);

// ==========================================
// VARIABLES COMMUNICATION SÉRIE
// ==========================================
String input_string = "";

void setup() {
  // Init L298N
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  
  // Direction initiale
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  motorPID.SetMode(AUTOMATIC);
  Serial.begin(115200);

  // Init Encodeur
  pinMode(encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_phaseA), encoderCallback, RISING);
}

void loop() {
  // ==========================================
  // 1. LIRE LES COMMANDES DU PC (Format: "V12.5\n")
  // ==========================================
  while (Serial.available() > 0) {
    char chr = Serial.read();
    if (chr == '\n') {
      // Commande complète reçue
      input_string.trim();
      if (input_string.startsWith("V")) {
        cmd_vel = input_string.substring(1).toFloat(); // Extraire la vitesse
      }
      input_string = "";
    } else {
      input_string += chr;
    }
  }

  // ==========================================
  // 2. BOUCLE DE CONTRÔLE (Toutes les 100ms)
  // ==========================================
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    
    // --- A. Calcul de la vitesse mesurée (rad/s) ---
    // Basé sur la formule : (Ticks / intervalle) * (60 / CPR) * 2PI
    // Note : Assurez-vous que 385.0 correspond à votre CPR réel !
    meas_vel = (10 * encoder_counter * (60.0 / 385.0)) * 0.10472;
    
    // Ajouter le signe (si le moteur recule)
    if (wheel_sign == "n") {
      meas_vel = -meas_vel;
    }

    // --- B. Calcul du PID ---
    motorPID.Compute();

    // Sécurité : Si la consigne est 0, on force le PWM à 0 (évite que le moteur tremble)
    if (cmd_vel == 0.0) {
      motor_pwm = 0.0;
    }

    // Appliquer le PWM (Limité à 255 max)
    analogWrite(L298N_enA, constrain((int)motor_pwm, 0, 255));

    // --- C. Envoyer l'état au PC (Format: "P1234 V5.67\n") ---
    // L'interface ROS 2 attend P (position ticks) et V (vitesse rad/s)
    // Note : On n'a pas de position absolue ici, on envoie le compteur cumulé
    // (Vous pouvez utiliser un long pour la position si besoin)
    Serial.print("P");
    Serial.print(encoder_counter); 
    Serial.print(" V");
    Serial.println(meas_vel, 4); // 4 décimales pour plus de précision

    // Reset pour le prochain cycle
    last_millis = current_millis;
    encoder_counter = 0; // On remet le compteur à zéro pour la vitesse
  }
}

// ==========================================
// CALLBACK ENCODEUR (S'exécute à chaque impulsion)
// ==========================================
void encoderCallback() {
  if (digitalRead(encoder_phaseB) == HIGH) {
    wheel_sign = "p";
  } else {
    wheel_sign = "n";
  }
  encoder_counter++;
}