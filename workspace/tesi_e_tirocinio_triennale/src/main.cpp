#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
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
std_msgs__msg__Int32 enc_msg;
rcl_timer_t timer;

// --- Oggetto Motore (Solo Motore 1) ---
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);

static unsigned long last_ping_time = 0;

enum States {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void stop_motors() {
  motor1->setSpeed(0);
  motor1->run(RELEASE);
}

// Timer callback eseguito ogni 10 secondi per inviare i tick dell'encoder
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        enc_msg.data = getEncoderTicks();
        rcl_publish(&publisher, &enc_msg, NULL);
    }
}

void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    // Toggle LED per conferma visiva immediata della ricezione del comando
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    float v = msg->linear.x;
    int32_t pwm = (int32_t)(v * 255.0f);

    if (pwm > 0) {
        motor1->run(FORWARD);
        motor1->setSpeed(pwm > 255 ? 255 : pwm);
    } else if (pwm < 0) {
        motor1->run(BACKWARD);
        motor1->setSpeed(-pwm > 255 ? 255 : -pwm);
    } else {
        stop_motors();
    }
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
  if (rclc_node_init_default(&node, "arduino_single_motor_node", "", &support) != RCL_RET_OK) return false;

  // Subscriber /cmd_vel
  if (rclc_subscription_init_default(
        &subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) return false;

  geometry_msgs__msg__Twist__init(&msg_sub);

  // Publisher /encoder_ticks (Tipo std_msgs/msg/Int32)
  if (rclc_publisher_init_default(
        &publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "encoder_ticks") != RCL_RET_OK) return false;

  // Timer a 10 secondi (10000 ms) per la lettura dell'encoder
  const unsigned int timer_timeout = 10000;
  if (rclc_timer_init_default(
        &timer, &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback) != RCL_RET_OK) return false;

  // Executor con 2 handles (1 subscriber + 1 timer)
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
      } else {
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

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