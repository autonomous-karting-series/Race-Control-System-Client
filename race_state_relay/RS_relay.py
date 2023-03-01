#usr/bin/env python3
import rclpy
import paho.mqtt.client as mqtt
from rclpy.node import Node
from std_msgs.msg import String

class RS_relay(Node):
    def __init__(self):
        super().__init__("RS_relay")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('mqtt_host', "127.0.0.1"),                     # ADDRESS TO MQTT BROKER
                ('mqtt_port', 1883),                            # PORT TO MQTT BROKER
                ('team_id', "AMP"),                             # USED FOR CLIENT NAME AND MQTT KART STATE TOPIC
                ('mqtt_track_state_topic', "track_state"),
                ('mqtt_kart_state_topic', "kart_state/"),
                ('ros_track_state_topic', "race_state"),
                ('ros_kart_state_topic', "kart_state")          # TOPIC TO PUBLISH STATE DATA WITHIN ROS
            ]
        )

        client_name = self.get_parameter('team_id').value
        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value
        mqtt_track_topic = self.get_parameter('mqtt_track_state_topic').value
        global mqtt_kart_topic ; str(self.get_parameter('mqtt_kart_state_topic').value) + str(self.get_parameter('team_id').value) # STRING NOT PARAMETER OBJECT LIKE OTHERS
        ros_track_topic = self.get_parameter('ros_track_state_topic').value
        ros_kart_topic = self.get_parameter('ros_kart_state_topic').value


        self.race_state_pub = self.create_publisher(String, ros_track_topic, 10)
        self.race_kart_sub = self.create_subscription(String, ros_kart_topic, self.listener_callback, 10) # CHANGE DATA TYPE IF NEEDED. NOT SURE WHAT DATA WILL BE SENT FROM TOPIC

        def on_message(client, userdata, message):
            msg = String()
            msg.data = str(message.payload.decode("utf-8"))
            self.get_logger().info(msg.data)
            self.race_state_pub.publish(msg)


        def on_disconnect(client, userdata, rc):
            client.loop_stop()
            client.disconnect()

        self.mqttclient = mqtt.Client(client_name)
        self.mqttclient.on_disconnect = on_disconnect
        self.mqttclient.on_message = on_message
        self.mqttclient.connect(mqtt_host, port = mqtt_port)
        self.mqttclient.loop_start()
        self.mqttclient.subscribe(mqtt_track_topic)
        
    def listener_callback(self, msg):
        self.get_logger().info(msg.data)
        self.mqttclient.publish(mqtt_kart_topic, str(msg.data))

def main(args=None):
    rclpy.init(args=args)

    try:
        node = RS_relay()
        rclpy.spin(node)
    except rclpy.exceptions.ROSInterruptException:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()