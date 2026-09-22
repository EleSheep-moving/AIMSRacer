from glob import glob
from setuptools import find_packages, setup


setup(
    name='aims_mpcc_sim',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/aims_mpcc_sim']),
        ('share/aims_mpcc_sim', ['package.xml']),
        ('share/aims_mpcc_sim/launch', glob('launch/*.py')),
        ('share/aims_mpcc_sim/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={'console_scripts': [
        'numerical_plant = aims_mpcc_sim.node:main',
        'prepare_circle = aims_mpcc_sim.fixture:main',
        'run_acceptance = aims_mpcc_sim.acceptance:main',
    ]},
)
