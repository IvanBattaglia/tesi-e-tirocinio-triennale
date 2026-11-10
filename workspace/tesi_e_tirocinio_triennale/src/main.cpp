#include <Arduino.h>
        #include <micro_ros_arduino.h>

        #include <rcl/rcl.h>
        #include <rcl/error_handling.h>
        #include <rclc/rclc.h>
        #include <std_msgs/msg/string.h>

        /* Header C per il ping e la gestione dell'agente micro-ROS */
        #include <rmw_microros/rmw_microros.h>

        static rcl_publisher_t publisher;
        static std_msgs__msg__String msg;
        static rclc_support_t support;
        static rcl_allocator_t allocator;
        static rcl_node_t node;

        /* Buffer di memoria per memorizzare la stringa */
        static char msg_buffer[50];

        // Stato della connessione micro-ROS
        enum States
        {
            WAITING_AGENT,
            AGENT_AVAILABLE,
            AGENT_CONNECTED,
            AGENT_DISCONNECTED
        } state;

        /* Distrugge le entità create in precedenza per consentire un nuovo init pulito */
        void destroy_entities(void)
        {
            rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
            (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

            rcl_publisher_fini(&publisher, &node);
            rcl_node_fini(&node);
            rclc_support_fini(&support);
        }

        /* Inizializza le entità ROS 2 (Allocatore, Nodo, Publisher, Messaggio) */
        bool create_entities(void)
        {
            allocator = rcl_get_default_allocator();

            if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
            {
                return false;
            }

            if (rclc_node_init_default(&node, "arduino_hello_node", "", &support) != RCL_RET_OK)
            {
                return false;
            }

            if (rclc_publisher_init_default(
                    &publisher,
                    &node,
                    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                    "micro_ros_string") != RCL_RET_OK)
            {
                return false;
            }

            msg.data.data = msg_buffer;
            msg.data.capacity = sizeof(msg_buffer);
            snprintf(msg.data.data, msg.data.capacity, "Hello World from Arduino UNO R4!");
            msg.data.size = strlen(msg.data.data);

            return true;
        }

        void setup(void)
        {
            /* Configura la seriale USB per il trasporto micro-ROS */
            set_microros_transports();

            pinMode(LED_BUILTIN, OUTPUT);
            (LED_BUILTIN, LOW);

            state = WAITING_AGENT;
        }

        void loop(void)
        {
            // implemento una Macchina a Stati Finiti (FSM - Finite State Machine)
            switch (state)
            {
            case WAITING_AGENT:
                // Verifica se l'agente è attivo (ping rapido ogni 100ms)
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                {
                    state = AGENT_AVAILABLE;
                }
                else
                {
                    // Fa lampeggiare il LED arancione finché l'agente non c'è
                    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
                    delay(100);
                }
                break;

            case AGENT_AVAILABLE:
                // Tenta di creare le entità ROS 2
                if (create_entities())
                {
                    state = AGENT_CONNECTED;
                    digitalWrite(LED_BUILTIN, HIGH); // LED fisso: connesso e operativo
                }
                else
                {
                    state = WAITING_AGENT;
                }
                break;

            case AGENT_CONNECTED:
                // Controlla se l'agente è ancora vivo prima di pubblicare
                if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                {
                    rcl_publish(&publisher, &msg, NULL);
                    delay(1000); // Frequenza di pubblicazione 1Hz
                }
                else
                {
                    state = AGENT_DISCONNECTED;
                }
                break;

            case AGENT_DISCONNECTED:
                // Spegne il LED, pulisce la sessione fallita e torna in attesa
                digitalWrite(LED_BUILTIN, LOW);
                destroy_entities();
                state = WAITING_AGENT;
                break;
            }
        }