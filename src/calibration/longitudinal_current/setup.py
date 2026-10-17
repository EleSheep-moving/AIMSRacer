from glob import glob
from setuptools import find_packages, setup

setup(name='aims_calibration',version='0.1.0',packages=find_packages(exclude=['tests']),
      data_files=[('share/ament_index/resource_index/packages',['resource/aims_calibration']),
                  ('share/aims_calibration',['package.xml','README.md']),
                  ('share/aims_calibration/config',glob('config/*.yaml')+glob('config/*.json')),
                  ('share/aims_calibration/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools','PyYAML'],
      extras_require={'wheel-model-fit':['numpy>=1.23.5','scipy>=1.12','matplotlib>=3.5']},zip_safe=True,
      maintainer='AIMSRacer maintainers',maintainer_email='maintainers@aimsracer.local',
      description='Selector-controlled longitudinal calibration',license='Apache-2.0',
      tests_require=['pytest'],test_suite='tests',entry_points={'console_scripts':[
          'baseline = aims_calibration.cli:baseline_main',
          'plan = aims_calibration.cli:plan_main',
          'results = aims_calibration.cli:results_main',
          'wheel-current-fit = aims_calibration.wheel_current_cli:fit_main',
          'wheel-current-query = aims_calibration.wheel_current_cli:query_main']})
