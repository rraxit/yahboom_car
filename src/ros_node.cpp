#include "credential.h"
#include "ros_node.h"

namespace airlab{

    Node::Node()
    {

    }

    void Node::init(const char* nodeName )
    {
        set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);

          allocator = rcl_get_default_allocator();

          //create init_options
          RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

          // create node
          RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

          // create publisher
          RCCHECK(rclc_publisher_init_default(
              &publisher,
              &node,
              pub_type_support,
              pub_topic_name));

          
      
          // create subscriber a reliable subscriber
          RCCHECK(rclc_subscription_init_default(
          &subscriber, &node,
          sub_type_support, sub_topic_name));

          // create timer,
          RCCHECK(rclc_timer_init_default(
              &timer,
              &support,
              RCL_MS_TO_NS(LOOP_DELAY),
              timer_callback));
          

          // create executor
          RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
          RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

            // Add subscription to the executor
          RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
          RCCHECK(rclc_executor_add_subscription(
          &executor_sub, &subscriber, &cmd_msg,
          &subscription_callback, ON_NEW_DATA));

          // config rplidar msg 
          msg.range_min = 0.0;
          msg.range_max = 50000.0;
          msg.angle_min = 0.0;
          msg.angle_max = 2 * M_PI;




         
    }

    void Node::loop()
    {
        rplidar.loop();
        RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(LOOP_DELAY)));
        RCSOFTCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(LOOP_DELAY)));
    }

    void Node::timer_callback(rcl_timer_t * timer, int64_t last_call_time)
    {
        RCLC_UNUSED(last_call_time);
        if (timer == NULL || !rplidar.is_measurement_ready()) 
            return;
        
        msg.scan_time = 1.0 / rplidar.get_fq();
        // delete [] msg.ranges.data;
        // msg.ranges.data = new float[360];
        // for (int i = 0; i < 360; i++)
        // {
        //     float val = (float) rplidar.get_measurement(i);
        //     msg.ranges.data[i] = val; 
        // }
        
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    }

    void Node::subscription_callback(const void * msgin)
    {
        // Cast received message to used type
        const geometry_msgs__msg__Twist * cmd_msg_ptr = (const geometry_msgs__msg__Twist *)msgin;

    }



}