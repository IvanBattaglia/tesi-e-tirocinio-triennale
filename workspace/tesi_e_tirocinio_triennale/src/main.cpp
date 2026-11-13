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

#include "encoder_qei.h"

// --- Oggetti ROS2 ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray enc_msg;
int32_t enc_data[2] = {0, 0}; // Buffer statico per l'array micro-ROS
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
  target_pwm1 = 0;
  target_pwm2 = 0;
  set_single_motor(motor1, 0);
  set_single_motor(motor2, 0);
  last_pwm1 = 0;
  last_pwm2 = 0;
}

// Timer callback a 50Hz (20 ms) per pubblicare [encoder0, encoder1]
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        enc_msg.data.data[0] = getEncoder0Ticks();
        enc_msg.data.data[1] = getEncoder1Ticks();
        rcl_publish(&publisher, &enc_msg, NULL);
    }
}

// Callback /cmd_vel: calcola i PWM differenziali senza chiamare I2C
void twist_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  // conferma visiva di ricesione del comando di moto
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  float linear = msg->linear.x;   // Avanti/Indietro (-1.0 a 1.0)
  float angular = msg->angular.z; // Rotazione (-1.0 a 1.0)

  // Miscelazione differenziale semplice (linear>0 = avanti; linear<0 = indietro; angular<0 = rotazione antioraria; angular>0 = rotazione oraria)
  float left = linear + angular; // motore M2
  float right = linear - angular; // motore M1

  // Normalizzazione se supera [-1.0, 1.0]
  float max_val = max(fabs(left), fabs(right));
  if (max_val > 1.0f) {
    left /= max_val;
    right /= max_val;
  } // 1.0 e -1.0 corrispondono a 255 e -255 PWM, cioè alla masima velocità avanti e indietro del motore, quando usi un robot a cinematica differenziale (differential drive), calcoli le velocità delle due ruote sommando e sottraendo la velocità lineare v e quella angolare omega, se tale somma restituisce un valore > 1.0 o < -1.0, allora normalizzi i valori per non superare il range [-1.0, 1.0], così facendo, il robot, riduce leggermente la velocità di entrambi i motori ma esegue esattamente la curva comandata poichè il rapporto tra la velocità della ruota destra e sinistra rimane identico

  target_pwm1 = (int16_t)(left * 255.0f);
  target_pwm2 = (int16_t)(right * 255.0f);
  new_cmd = true;
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  geometry_msgs__msg__Twist__fini(&msg_sub);
  rcl_publisher_fini(&publisher, &node);
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

  // Inizializzazione memoria statica per std_msgs/msg/Int32MultiArray
  enc_msg.data.capacity = 2;
  enc_msg.data.size = 2;
  enc_msg.data.data = enc_data;
  enc_msg.layout.dim.capacity = 0;
  enc_msg.layout.dim.size = 0;
  enc_msg.layout.dim.data = NULL;
  enc_msg.layout.data_offset = 0;

  // Publisher /encoder_ticks
  if (rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_ticks") != RCL_RET_OK) return false;

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
        last_cmd_time = millis();

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
      } else if (millis() - last_cmd_time > CMD_TIMEOUT_MS && (last_pwm1 != 0 || last_pwm2 != 0)) {
        // Sicurezza: ferma i motori se non arrivano più comandi /cmd_vel
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