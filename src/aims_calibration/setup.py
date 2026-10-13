from glob import glob
from setuptools import find_packages, setup

setup(name='aims_calibration',version='0.1.0',packages=find_packages(exclude=['tests']),
      data_files=[('share/ament_index/resource_index/packages',['resource/aims_calibration']),
                  ('share/aims_calibration',['package.xml','README.md']),
                  ('share/aims_calibration/config',glob('config/*.yaml')),
                  ('share/aims_calibration/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools','PyYAML'],zip_safe=True,
      maintainer='AIMSRacer maintainers',maintainer_email='maintainers@aimsracer.local',
      description='Selector-controlled longitudinal calibration',license='Apache-2.0',
      tests_require=['pytest'],test_suite='tests',entry_points={'console_scripts':[
          'longitudinal_calibration = aims_calibration.node:main']})
