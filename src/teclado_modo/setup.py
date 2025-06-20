from setuptools import find_packages, setup

package_name = 'teclado_modo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mbenarm@alumno.upv.es',
    maintainer_email='mbenarm@alumno.upv.es@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'teclado_modo_node = teclado_modo.teclado_modo_node:main',
        ],
    },
)
