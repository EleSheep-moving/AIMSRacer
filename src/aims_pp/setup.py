from glob import glob
from setuptools import find_packages, setup

setup(name='aims_pp',version='0.1.0',packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/aims_pp']),('share/aims_pp',['package.xml','README.md','README.zh-CN.md','LICENSE']),
                  ('share/aims_pp/config',glob('config/*.yaml')),('share/aims_pp/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools','numpy','PyYAML'],zip_safe=False,maintainer='AIMSRacer',maintainer_email='ZhizhaoZhang@gmail.com',
      tests_require=['pytest'],
      description='Independent enhanced pure pursuit with trusted localization and explicit run ownership',license='Apache-2.0',
      entry_points={})
