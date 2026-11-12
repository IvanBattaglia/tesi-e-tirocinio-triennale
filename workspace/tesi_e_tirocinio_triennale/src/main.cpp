#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <Adafruit_MotorShield.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32_multi_array.h>

// Inclusione dell'header per la gestione dei log micro-ROS
#include <rcl/logging.h>

// Inclusione dell'header personalizzato per i timer GPT
#include "encoder_qei.h"

// --- Oggetti ROS2 ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__Int32MultiArray msg;
rcl_timer_t timer;

// --- Oggetti Motore ---
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);

// Array statico contenente i passi dei due encoder
static int32_t encoder_data[2];

// Timer statico per la scansione periodica del ping micro-ROS (evita saturazione bus seriale)
static unsigned long last_ping_time = 0;

// Macchina a Stati Finiti micro-ROS
enum States {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// Routine di sicurezza arresto motori
void stop_motors() {
  motor1->setSpeed(0);
  motor1->run(RELEASE);
  motor2->setSpeed(0);
  motor2->run(RELEASE);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
    {
        (void)last_call_time;
        if (timer != NULL)
        {
            // Lettura istantanea diretta dai registri contatore hardware a 32-bit (senza overhead CPU)
            msg.data.data[0] = getEncoder0Ticks();
            msg.data.data[1] = getEncoder1Ticks();

            // Pubblicazione sul topic ROS2
            rcl_publish(&publisher, &msg, NULL);
        }
    }

// Callback per ricevere i comandi e pilotare i motori
void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    RCUTILS_LOG_INFO("ricevuto input movimentomotori"\n);

    // Usiamo il valore lineare x per simulare il test avanti/indietro
    float v = msg->linear.x;

    // Convertiamo in PWM (assumendo -1.0 a 1.0 mappato su -255 a 255)
    int32_t pwm = (int32_t)(v * 255.0f);

    if (pwm > 0) {
        RCUTILS_LOG_INFO("pwm>0"\n);
        motor1->run(FORWARD);
        motor2->run(FORWARD);
        motor1->setSpeed(pwm > 255 ? 255 : pwm);
        motor2->setSpeed(pwm > 255 ? 255 : pwm);
    } else if (pwm < 0) {
        RCUTILS_LOG_INFO("pwm  <  0"\n);
        motor1->run(BACKWARD);
        motor2->run(BACKWARD);
        motor1->setSpeed(-pwm > 255 ? 255 : -pwm);
        motor2->setSpeed(-pwm > 255 ? 255 : -pwm);
    } else {
        RCUTILS_LOG_INFO("pwm = 0"\n);
        stop_motors();
    }
    RCUTILS_LOG_INFO("fine funzione twist_callback"\n);
}

void destroy_entities() {
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  geometry_msgs__msg__Twist__fini(&msg_sub); // Pulizia della struttura Twist
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
  if (rclc_node_init_default(&node, "arduino_robot_node", "", &support) != RCL_RET_OK) return false;

  // 1. Inizializzazione Subscriber /cmd_vel
  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) return false;

  // Inizializzazione della struttura del messaggio Twist in ricezione
  geometry_msgs__msg__Twist__init(&msg_sub);

  // 2. Inizializzazione Publisher /encoder_ticks
  if (rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_ticks") != RCL_RET_OK) return false;

  // Assegnazione buffer statico dei dati
  msg.data.capacity = 2;
  msg.data.size = 2;
  msg.data.data = encoder_data;

  // 3. Inizializzazione Timer a 0,001 Hz (1000 ms = 1s)
  const unsigned int timer_timeout = 1000;
  if (rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback) != RCL_RET_OK) return false;

  // 4. Inizializzazione Executor con 2 handles (1 timer + 1 subscriber)
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) return false;
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &twist_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer);

  return true;
}

void setup() {

    // Inizializzazione del trasporto micro-ROS (porta USB Seriale nativa dell'Arduino Uno R4)
    set_microros_transports();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // Configurazione hardware dei timer GPT320 e GPT321 in Phase Counting Mode 4 (4x)
    setupGPT_QEI();
    Wire.begin();

    // Inizializza l'I2C della shield senza bloccare l'esecuzione in caso di errore
    AFMS.begin();

    state = WAITING_AGENT;
}

void loop() {

    RCUTILS_LOG_INFO("sei in void loop()"\n);

  switch (state) {
    case WAITING_AGENT:

      RCUTILS_LOG_INFO("sei in case WAITING_AGENT"\n);

      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        state = AGENT_AVAILABLE;
      } else {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
      }
      break;

    case AGENT_AVAILABLE:

      RCUTILS_LOG_INFO("sei in case AGENT_AVAILABLE"\n);

      if (create_entities()) {
        state = AGENT_CONNECTED;
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        state = WAITING_AGENT;
      }
      break;

    /* case AGENT_CONNECTED:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        delay(10);
      } else {
        state = AGENT_DISCONNECTED;
      }
      break; */


    case AGENT_CONNECTED:

      RCUTILS_LOG_INFO("sei in case AGENT_CONNECTED"\n);

      // Esegue lo spin ogni 10ms per elaborare i comandi cmd_vel in ingresso
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

      // Controlla il ping solo una volta al secondo per lasciare il canale seriale libero per cmd_vel
      if (millis() - last_ping_time > 1000) {
        last_ping_time = millis();
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          state = AGENT_DISCONNECTED;
        }
      }
      break;

    case AGENT_DISCONNECTED:

      RCUTILS_LOG_INFO("sei in case AGENT_DISCONNECTED"\n);

      digitalWrite(LED_BUILTIN, LOW);
      stop_motors();
      destroy_entities();
      state = WAITING_AGENT;
      break;
  }
}