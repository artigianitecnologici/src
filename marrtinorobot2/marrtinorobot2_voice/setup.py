# Copyright 2025 robotics-3d.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Ferrarini Fabio
# Email : ferrarini09@gmail.com

from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'marrtinorobot2_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
       
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ferrarini fabio',
    maintainer_email='ferrarini09@gmail.com',
    description='module tts',
    license='',
    entry_points={
        'console_scripts': [
          'tts_node = marrtinorobot2_voice.tts_node:main',
        ],
    },
)


