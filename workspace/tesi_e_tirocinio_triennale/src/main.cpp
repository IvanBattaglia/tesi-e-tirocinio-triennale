// alimentazione scheda di potenza: 12V ; limite assorbimento di corrente: 1A
#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Adafruit_MotorShield.h>
#include <rmw_microros/rmw_microros.h>
#include <PID_v1.h> // Libreria PID di Brett Beauregard

#include "encoder_qei.h"

// =========================================================================
// 1. PARAMETRI FISICI DEL ROBOT (INSERISCI I VALORI REALI DEL TUO ROBOT!)
// =========================================================================
const double TICKS_PER_REV   = 86000.0; // Tick per 1 giro completo ruota (in quadratura 4x) = 500 CPR * 4 (QEI) * 43 (riduttore) = 86.000
const double WHEEL_RADIUS_M  = 0.0485; // Raggio ruota in metri
const double WHEEL_BASE_M    = 0.40;  // Distanza tra i centri delle due ruote in metri

// Limite fisico di sicurezza per la singola ruota in rad/s
const double MAX_WHEEL_RAD_S = 5.5;

// Fattore di conversione precalcolato: (2 * PI) / TICKS_PER_REV
const double TICKS_TO_RAD = (2.0 * PI) / TICKS_PER_REV;

// =========================================================================
// 2. PARAMETRI DEL CONTROLLORE PID (IN UNITÀ RAD/S -> PWM)
// =========================================================================
const double PID_SAMPLE_TIME_MS = 20.0; // 50 Hz

// Guadagni PID tarati per ingresso in rad/s e uscita PWM (-255..255) e calcolati sul guadagno statico K ≈ 0.0255 dei Faulhaber a 12V
double Kp = 26.0, Ki = 180.0, Kd = 0.0;

// Variabili PID (devono essere tutte double per la libreria PID_v1)
double setpoint1 = 0, input1 = 0, output1 = 0; // Motore 1 (Sinistro / Enc 0)
double setpoint2 = 0, input2 = 0, output2 = 0; // Motore 2 (Destro / Enc 1)

PID pid1(&input1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID pid2(&input2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);

// Variabili per il calcolo della velocità dagli encoder
int32_t prev_ticks0 = 0;
int32_t prev_ticks1 = 0;
unsigned long prev_vel_time = 0;

// --- Oggetti ROS2 ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Publisher 1: Tick cumulativi encoder (per odometria)
rcl_publisher_t enc_publisher;
std_msgs__msg__Int32MultiArray enc_msg;
int32_t enc_data[2] = {0, 0};

// Publisher 2: Velocità ruote in rad/s [vel_sx, vel_dx]
rcl_publisher_t vel_publisher;
geometry_msgs__msg__Twist vel_msg;

rcl_timer_t timer;

// --- Oggetti Motore (M1 = Sinistro/Enc0, M2 = Destro/Enc1) ---
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// --- Variabili di controllo condivise ---
volatile int16_t target_pwm1 = 0;
volatile int16_t target_pwm2 = 0;
volatile bool new_cmd = false;

int16_t last_pwm1 = 999; // Valore fittizio per forzare il primo aggiornamento
int16_t last_pwm2 = 999;

static unsigned long last_ping_time = 0;
static unsigned long last_cmd_time = 0;

// Watchdog di sicurezza CMD_TIMEOUT_MS = 500 ms: se usi teleop_twist_keyboard per testare il robot, ricorda che la tastiera invia comandi solo alla pressione dei tasti (quindi il robot si fermerà dopo mezzo secondo dal rilascio, evitando che scappi via se cade la connessione). Se invece invii comandi singoli da terminale con ros2 topic pub --once, aumenta temporaneamente CMD_TIMEOUT_MS o commenta il blocco else if.
const unsigned long CMD_TIMEOUT_MS = 500; // Watchdog: ferma i motori se /cmd_vel sparisce

enum States {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void set_single_motor(Adafruit_DCMotor *motor, int16_t pwm) {
  if (pwm > 0) {
    motor->run(FORWARD);
    motor->setSpeed(pwm > 255 ? 255 : pwm);
  } else if (pwm < 0) {
    motor->run(BACKWARD);
    motor->setSpeed(-pwm > 255 ? 255 : -pwm);
  } else {
    motor->setSpeed(0);
    motor->run(RELEASE);
  }
}

void stop_motors() {
  setpoint1 = 0;
  setpoint2 = 0;
  output1 = 0;
  output2 = 0;
  // Mettendo in MANUAL azzeriamo la memoria dell'integrale (anti-windup)
  pid1.SetMode(MANUAL);
  pid2.SetMode(MANUAL);
  set_single_motor(motor1, 0);
  set_single_motor(motor2, 0);
  last_pwm1 = 0;
  last_pwm2 = 0;
}

// Timer callback a 50Hz (20 ms): legge encoder, calcola velocità e aggiorna PID
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer == NULL) return;

    unsigned long now = millis();
    double dt = (now - prev_vel_time) / 1000.0; // Tempo trascorso in secondi (~0.020 s)
    prev_vel_time = now;

    // 1. Lettura tick attuali
    int32_t curr_ticks0 = getEncoder0Ticks();
    int32_t curr_ticks1 = getEncoder1Ticks();

    // 2. Calcolo velocità reale delle ruote in RADIANTI AL SECONDO (rad/s)
    // NOTA: verifica il segno! Se con PWM > 0 la velocità risulta negativa, inverti il segno qui
    if (dt > 0.0) {
        double delta_ticks0 = (double)(curr_ticks0 - prev_ticks0);
        double delta_ticks1 = (double)(curr_ticks1 - prev_ticks1);

        input1 = (delta_ticks0 * TICKS_TO_RAD) / dt;
        input2 = (delta_ticks1 * TICKS_TO_RAD) / dt;
    }
    prev_ticks0 = curr_ticks0;
    prev_ticks1 = curr_ticks1;

    // 3. Calcolo PID (confronta setpoint in rad/s con input in rad/s)
    pid1.Compute();
    pid2.Compute();

    // 4. invio dei PWM calcolati al loop principale (che gestisce l'I2C)
    if (pid1.GetMode() == AUTOMATIC) {
        target_pwm1 = (int16_t)output1;
        target_pwm2 = (int16_t)output2;
        new_cmd = true;
    }

    // 5. Pubblicazione tick encoder
    enc_msg.data.data[0] = curr_ticks0;
    enc_msg.data.data[1] = curr_ticks1;
    rcl_publish(&enc_publisher, &enc_msg, NULL);

    // 6. Pubblicazione velocità ruote in rad/s
    vel_msg.linear.x = input1;
    vel_msg.linear.y = input2;
    rcl_publish(&vel_publisher, &vel_msg, NULL);
}

// Callback /cmd_vel: converte v (m/s) e omega (rad/s) in velocità ruote (rad/s)
void twist_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  last_cmd_time = millis();

  double v = msg->linear.x;      // Velocità lineare richiesta [m/s]
  double omega = msg->angular.z; // Velocità angolare richiesta [rad/s]

  // Cinematica inversa differential drive -> velocità angolare ruote [rad/s]
  // Se ruotando a sinistra/destra il robot gira al contrario, inverti i segni - e +
  double w_left  = (v - (omega * WHEEL_BASE_M / 2.0)) / WHEEL_RADIUS_M; // M1
  double w_right = (v + (omega * WHEEL_BASE_M / 2.0)) / WHEEL_RADIUS_M; // M2

  // Saturazione proporzionale solo se si supera il limite fisico MAX_WHEEL_RAD_S
  // In questo modo i tasti q/z/w/x/e/c di teleop_twist_keyboard modulano liberamente la velocità
  double max_w = max(fabs(w_left), fabs(w_right));
  if (max_w > MAX_WHEEL_RAD_S) {
    double scale = MAX_WHEEL_RAD_S / max_w;
    w_left  *= scale;
    w_right *= scale;
  }

  setpoint1 = w_left;
  setpoint2 = w_right;

  // Soglia di arresto (es. sotto 0.05 rad/s ferma i motori)
  if (fabs(setpoint1) < 0.05 && fabs(setpoint2) < 0.05) {
      stop_motors();
  } else {
      pid1.SetMode(AUTOMATIC);
      pid2.SetMode(AUTOMATIC);
  }
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  geometry_msgs__msg__Twist__fini(&msg_sub);
  rcl_publisher_fini(&enc_publisher, &node);
  geometry_msgs__msg__Twist__fini(&vel_msg);
  rcl_publisher_fini(&vel_publisher, &node);
  rcl_timer_fini(&timer);
  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "arduino_diff_drive_node", "", &support) != RCL_RET_OK) return false;

  // Subscriber /cmd_vel
  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) return false;

  geometry_msgs__msg__Twist__init(&msg_sub);

  // Inizializzazione memoria statica per /encoder_ticks
  enc_msg.data.capacity = 2;
  enc_msg.data.size = 2;
  enc_msg.data.data = enc_data;
  enc_msg.layout.dim.capacity = 0;
  enc_msg.layout.dim.size = 0;
  enc_msg.layout.dim.data = NULL;
  enc_msg.layout.data_offset = 0;

  // Publisher /encoder_ticks
  if (rclc_publisher_init_default(
        &enc_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_ticks") != RCL_RET_OK) return false;

  // Inizializzazione messaggio Twist per /wheel_vel (rad/s)
  geometry_msgs__msg__Twist__init(&vel_msg);

  // Publisher /wheel_vel
  if (rclc_publisher_init_default(
        &vel_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "wheel_vel") != RCL_RET_OK) return false;

  // Timer a 50Hz (20 ms)
  const unsigned int timer_timeout = 20;
  if (rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback) != RCL_RET_OK) return false;

  // Executor con 2 handles
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &twist_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}

void setup() {
    set_microros_transports();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // 1. Prima inizializza I2C e la Motor Shield
    Wire.begin();
    Wire.setClock(100000);
    AFMS.begin();

    // 2. Poi inizializza i due timer GPT a 32 bit
    setupGPT_QEI();

    // Configurazione PID
    pid1.SetOutputLimits(-255, 255);
    pid2.SetOutputLimits(-255, 255);
    pid1.SetSampleTime(PID_SAMPLE_TIME_MS);
    pid2.SetSampleTime(PID_SAMPLE_TIME_MS);
    pid1.SetMode(MANUAL); // Parte spento finché non arriva un comando
    pid2.SetMode(MANUAL);

    prev_vel_time = millis();
    state = WAITING_AGENT;
}

void loop() {
  switch (state) {
    case WAITING_AGENT:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        state = AGENT_AVAILABLE;
      } else {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
      }
      break;

    case AGENT_AVAILABLE:
      if (create_entities()) {
        state = AGENT_CONNECTED;
        digitalWrite(LED_BUILTIN, HIGH);
        last_cmd_time = millis();
      } else {
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

      // Aggiorna i motori via I2C solo fuori dalla callback e solo se il valore è cambiato
      if (new_cmd) {
        new_cmd = false;

        int16_t p1 = target_pwm1;
        int16_t p2 = target_pwm2;

        if (p1 != last_pwm1) {
          set_single_motor(motor1, p1);
          last_pwm1 = p1;
        }
        if (p2 != last_pwm2) {
          set_single_motor(motor2, p2);
          last_pwm2 = p2;
        }
    }

    // Sicurezza: ferma i motori se non arrivano più comandi /cmd_vel per oltre CMD_TIMEOUT_MS
    if (millis() - last_cmd_time > CMD_TIMEOUT_MS && (pid1.GetMode() == AUTOMATIC || last_pwm1 != 0 || last_pwm2 != 0)) {
        stop_motors();
      }

      // Ping di verifica connessione ogni 1000 ms
      if (millis() - last_ping_time > 1000) {
        last_ping_time = millis();
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          state = AGENT_DISCONNECTED;
        }
      }
      break;

    case AGENT_DISCONNECTED:
      digitalWrite(LED_BUILTIN, LOW);
      stop_motors();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}