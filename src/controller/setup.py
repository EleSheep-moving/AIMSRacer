from glob import glob
from setuptools import find_packages, setup

setup(
    name='aims_mpcc', version='0.1.0', packages=find_packages(),
    package_data={'aims_mpcc.vendor': ['*LICENSE']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/aims_mpcc']),
        ('share/aims_mpcc', ['package.xml', 'LICENSE', 'NOTICE.md']),
        ('share/aims_mpcc/config', glob('config/*.yaml')),
        ('share/aims_mpcc/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'casadi==3.7.2', 'PyYAML'],
    zip_safe=False, maintainer='AIMSRacer', maintainer_email='ZhizhaoZhang@gmail.com',
    description='Recorded-lap MPCC with bounded ROS execution and speed-mode control.',
    license='BSD-3-Clause AND Apache-2.0 AND LGPL-3.0-only',
    entry_points={'console_scripts': [
        'record_path = aims_mpcc.recorder:main',
        'prepare_path = aims_mpcc.prepare:main',
        'prepare_solver = aims_mpcc.prepare_solver:main',
        'mpcc_node = aims_mpcc.node:main',
        'closed_loop_test = aims_mpcc.integration:main',
    ]},
)
