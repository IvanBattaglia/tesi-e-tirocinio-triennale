#include <Arduino.h>
        #include <micro_ros_arduino.h>
        #include <stdio.h>
        #include <rcl/rcl.h>
        #include <rcl/error_handling.h>
        #include <rclc/rclc.h>
        #include <rclc/executor.h>
        #include <std_msgs/msg/int32_multi_array.h>

        // Inclusione dell'header personalizzato per i timer GPT
        #include "encoder_qei.h"

        rcl_publisher_t publisher;
        std_msgs__msg__Int32MultiArray msg;
        rclc_executor_t executor;
        rclc_support_t support;
        rcl_allocator_t allocator;
        rcl_node_t node;
        rcl_timer_t timer;

        // Array statico contenente i passi dei due encoder
        static int32_t encoder_data[2];

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

        void setup()
        {
            // Inizializzazione del trasporto micro-ROS (porta USB Seriale nativa dell'Arduino Uno R4)
            set_microros_transports();

            // Configurazione hardware dei timer GPT320 e GPT321 in Phase Counting Mode 4 (4x)
            setupGPT_QEI();

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

            // Collegamento del buffer statico all'array dinamico della struttura del messaggio ROS2
            msg.data.capacity = 2;
            msg.data.size = 2;
            msg.data.data = encoder_data;

            // Configurazione del timer di pubblicazione micro-ROS a 50 Hz (periodo = 20 ms)
            const unsigned int timer_timeout = 20;
            rclc_timer_init_default(
                &timer,
                &support,
                RCL_MS_TO_NS(timer_timeout),
                timer_callback);

            // Inizializzazione dell'executor micro-ROS
            rclc_executor_init(&executor, &support.context, 1, &allocator);
            rclc_executor_add_timer(&executor, &timer);
        }

        void loop()
        {
            // Esecuzione periodica dei task micro-ROS e dispatching delle callback
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        }