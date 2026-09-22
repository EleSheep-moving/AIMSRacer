from glob import glob
from setuptools import find_packages, setup


setup(
    name='aims_gazebo_sim',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/aims_gazebo_sim']),
        ('share/aims_gazebo_sim', ['package.xml']),
        ('share/aims_gazebo_sim/config', glob('config/*.yaml')),
        ('share/aims_gazebo_sim/launch', glob('launch/*.py')),
        ('share/aims_gazebo_sim/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={'console_scripts': [
        'vesc_gazebo_bridge = aims_gazebo_sim.node:main',
        'run_acceptance = aims_gazebo_sim.acceptance:main',
    ]},
)
