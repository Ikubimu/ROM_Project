from setuptools import setup
import os
from glob import glob

package_name = 'gestos_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'Model'), glob('Model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu_correo@example.com',
    description='Paquete para detectar gestos y publicar órdenes',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gestos_node = gestos_robot.run_gestos:main',
            'orden_subscriber = gestos_robot.subscriber_node:main',
        ],
    },
)
