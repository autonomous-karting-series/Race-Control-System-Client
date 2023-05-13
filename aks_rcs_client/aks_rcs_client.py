"""Contains main runner for aks_rcs_client."""

import aks_rcs_client
import rclpy
import paho.mqtt.client as mqtt
from rclpy.node import Node
from std_msgs.msg import String

# Create high-reliability QoS with guranteed delivery
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy
qos_reliable = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, # TRANSIENT_LOCAL set, not point in history
    depth=1, # See above
    reliability=ReliabilityPolicy.RELIABLE,  # Guaranteed delivery
    durability=DurabilityPolicy.
    TRANSIENT_LOCAL  # Re-send msg to late-joining subscribers
)

class AKSRCSClient(Node):
    """Class handling the ASK Client node process."""

    def __init__(self):
        """Initialize AKSRCSClient class."""
        super().__init__('aks_rcs_client')
        self.declare_parameters(
            namespace='',
            parameters=[  #Team ID, unique identifier for each team
                ('team_id', 'TEAM_ID'),
                #Address of mqtt broker
                ('mqtt_host', '127.0.0.1'),
                #Port of mqtt broker
                ('mqtt_port', 1883),
                ('mqtt_track_state_topic', 'track_state'),
                ('mqtt_kart_state_topic', 'kart_state'),
                #ROS topic to publish track state to
                ('ros_track_state_topic', 'track_state'),
                #ROS topic to subscribe to for kart state
                ('ros_kart_state_topic', 'kart_state')
            ])

        # Get params
        team_id = self.get_parameter('team_id').value
        mqtt_host = self.get_parameter('mqtt_host').value
        mqtt_port = self.get_parameter('mqtt_port').value
        mqtt_track_topic = self.get_parameter('mqtt_track_state_topic').value
        mqtt_kart_topic = self.get_parameter('mqtt_kart_state_topic').value
        ros_track_topic = self.get_parameter('ros_track_state_topic').value
        ros_kart_topic = self.get_parameter('ros_kart_state_topic').value

        # full mqtt topic to publish kart state
        self.mqtt_kart_topic = f"{mqtt_kart_topic}/{team_id}"

        # Initialize ros2 publisher and subscriber
        self.race_state_pub = self.create_publisher(String, ros_track_topic,
                                                    qos_reliable)
        self.race_kart_sub = self.create_subscription(String, ros_kart_topic,
                                                      self.listener_callback,
                                                      qos_reliable)

        # MQTT functions for async actions
        def on_message(client, userdata, message):
            msg = String()
            msg.data = str(message.payload.decode('utf-8'))
            self.race_state_pub.publish(msg)

        def on_disconnect(client, userdata, rc):
            client.loop_stop()
            client.disconnect()

        # Initialize mqtt publishers and subscribers
        self.mqttclient = mqtt.Client(team_id)
        self.mqttclient.on_disconnect = on_disconnect
        self.mqttclient.on_message = on_message
        self.mqttclient.connect(mqtt_host, port=mqtt_port)
        self.mqttclient.loop_start()
        self.mqttclient.subscribe(mqtt_track_topic)

    def listener_callback(self, msg):
        """ROS2 kart_topic message publish to mqtt kart_topic."""
        self.mqttclient.publish(self.mqtt_kart_topic, str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    aks_rcs_client = AKSRCSClient()
    rclpy.spin(aks_rcs_client)
    aks_rcs_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
