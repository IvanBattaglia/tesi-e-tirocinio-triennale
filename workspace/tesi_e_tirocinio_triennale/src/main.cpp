#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h> // Header per i comandi di velocità ROS2
#include <Adafruit_MotorShield.h>    // Libreria Shield Adafruit Motor Shield v3

// Inclusione dell'header personalizzato per i timer GPT
#include "encoder_qei.h"

// --- Oggetti ROS2 ---
rcl_publisher_t publisher;
rcl_subscription_t subscriber; // Subscriber per ricevere i comandi
std_msgs__msg__Int32MultiArray msg_pub;
geometry_msgs__msg__Twist msg_sub; // Messaggio Twist in ricezione
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// array statici
static int32_t encoder_data[2]; // Array statico contenente i passi dei due encoder

// --- Oggetti Motore ---
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(1);  // M1 sulla shield
Adafruit_DCMotor *motorRight = AFMS.getMotor(2); // M2 sulla shield

// Parametri cinematici del robot
const float WHEEL_BASE = 0.40;   // Distanza tra le ruote (in metri)
const float MAX_SPEED_MS = 0.40; // Velocità massima stimata a 10V (in m/s) per mappare a PWM 255

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        // Lettura istantanea diretta dai registri contatore hardware a 32-bit (senza overhead CPU)
        msg_pub.data.data[0] = getEncoder0Ticks();
        msg_pub.data.data[1] = getEncoder1Ticks();

        // Pubblicazione sul topic ROS2
        rcl_publish(&publisher, &msg_pub, NULL);
    }
}

// Callback Subscriber Twist: Calcola la cinematica differenziale
void twist_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    float v = msg->linear.x;      // m/s
    float omega = msg->angular.z; // rad/s

    // Modello cinematico a guida differenziale
    float v_left = v - (omega * WHEEL_BASE / 2.0f);
    float v_right = v + (omega * WHEEL_BASE / 2.0f);

    // Mappatura lineare in scala PWM (-255 a 255)
    int32_t pwm_left = (int32_t)((v_left / MAX_SPEED_MS) * 255.0f);
    int32_t pwm_right = (int32_t)((v_right / MAX_SPEED_MS) * 255.0f);

    // Controllo Motore Sinistro (Limita tra -255 e 255)
    if (pwm_left >= 0)
    {
        motorLeft->run(FORWARD);
        motorLeft->setSpeed(pwm_left > 255 ? 255 : pwm_left);
    }
    else
    {
        motorLeft->run(BACKWARD);
        motorLeft->setSpeed(-pwm_left > 255 ? 255 : -pwm_left);
    }

    // Controllo Motore Destro (Limita tra -255 e 255)
    if (pwm_right >= 0)
    {
        motorRight->run(FORWARD);
        motorRight->setSpeed(pwm_right > 255 ? 255 : pwm_right);
    }
    else
    {
        motorRight->run(BACKWARD);
        motorRight->setSpeed(-pwm_right > 255 ? 255 : -pwm_right);
    }
}

void setup()
{
    // Inizializzazione del trasporto micro-ROS (porta USB Seriale nativa dell'Arduino Uno R4)
    set_microros_transports();

    // Configurazione hardware dei timer GPT320 e GPT321 in Phase Counting Mode 4 (4x)
    setupGPT_QEI();

    // Inizializza la Motor Shield con frequenza PWM default (1.6KHz)
    AFMS.begin();

    allocator = rcl_get_default_allocator();

    // Inizializzazione della struttura di supporto micro-ROS
    rclc_support_init(&support, 0, NULL, &allocator);

    // Creazione del nodo ROS2
    rclc_node_init_default(&node, "arduino_encoder_node", "", &support);

    // Inizializzazione del Publisher sul topic "encoder_ticks"
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_ticks");

    // Subscriber per i comandi di velocità "cmd_vel"
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    // Buffer Publisher
    msg_pub.data.capacity = 2;
    msg_pub.data.size = 2;
    msg_pub.data.data = encoder_data;

    // Configurazione del timer di pubblicazione micro-ROS a 50 Hz (periodo = 20 ms)
    const unsigned int timer_timeout = 20;
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback);

    // Inizializzazione dell'executor micro-ROS, l'executor ora gestisce 2 processi (1 timer + 1 subscriber)
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    // Aggiunta del subscriber all'executor
    rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &twist_callback, ON_NEW_DATA);
}

void loop()
{
    // Esecuzione periodica dei task micro-ROS e dispatching delle callback
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}