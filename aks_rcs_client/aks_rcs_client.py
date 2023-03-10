#usr/bin/env python3
import rclpy
import paho.mqtt.client as mqtt
from rclpy.node import Node
from std_msgs.msg import String


class RCSclient(Node):

    def __init__(self):
        super().__init__("RCSclient")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('mqtt_host', "127.0.0.1"),  # Address to mqtt broker
                ('mqtt_port', 1883),  # Port to mqtt broker
                ('team_id',
                 "team_id"),  # Used for client name and mqtt kart state topic
                ('mqtt_track_state_topic', "track_state"),
                ('mqtt_kart_state_topic', "kart_state"),
                ('ros_track_state_topic',
                 "track_state"),  # Topic to publish track state within ROS
                ('ros_kart_state_topic', "kart_state"
                 )  # Topic to get kart state within ROS
            ])

        # Set Parameter values to variables
        client_name = self.get_parameter('team_id').value
        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value
        mqtt_track_topic = self.get_parameter('mqtt_track_state_topic').value
        # String not parameter object like previous variables
        self.mqtt_kart_topic = str(
            self.get_parameter('mqtt_kart_state_topic').value) + "/" + str(
                self.get_parameter('team_id').value)
        ros_track_topic = self.get_parameter('ros_track_state_topic').value
        ros_kart_topic = self.get_parameter('ros_kart_state_topic').value

        # Initialize ros2 publisher and subscriber
        self.race_state_pub = self.create_publisher(String, ros_track_topic,
                                                    10)
        # Change data type if needed. not sure what data will be sent from topic.
        self.race_kart_sub = self.create_subscription(String, ros_kart_topic,
                                                      self.listener_callback,
                                                      10)

        # MQTT functions for async actions
        def on_message(client, userdata, message):
            msg = String()
            msg.data = str(message.payload.decode("utf-8"))
            self.get_logger().info(msg.data)
            self.race_state_pub.publish(msg)

        def on_disconnect(client, userdata, rc):
            client.loop_stop()
            client.disconnect()


# Initialize mqtt publishers and subscribers

        self.mqttclient = mqtt.Client(client_name)
        self.mqttclient.on_disconnect = on_disconnect
        self.mqttclient.on_message = on_message
        self.mqttclient.connect(mqtt_host, port=mqtt_port)
        self.mqttclient.loop_start()
        self.mqttclient.subscribe(mqtt_track_topic)

    # ROS2 kart_topic message publish to mqtt kart_topic
    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        self.mqttclient.publish(self.mqtt_kart_topic, str(msg.data))


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RCSclient()
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
