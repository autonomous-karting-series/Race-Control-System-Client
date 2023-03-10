from setuptools import setup

package_name = 'aks_rcs_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Nick Berlier',
    maintainer_email='berlier3@gmail.com',
    description='ROS2-MQTT bridge for track and kart state communication',
    license='GNU General Public License 3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aks_rcs_client_node = aks_rcs_client.aks_rcs_client:main"
        ],
    },
)
