from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'marrtinorobot2_vision'

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
    maintainer='Ferrarini Fabio',
    maintainer_email='ferrarini09@gmail.com',
    description='Vision Module',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
      'img_publisher = marrtinorobot2_vision.webcam_pub:main',
      'img_subscriber = marrtinorobot2_vision.webcam_sub:main',
      'face_recognition_node = marrtinorobot2_vision.face_recognition_node:main'
	  'face_tracker_controller = marrtinorobot2_vision.face_tracker_controller:main'
	

      ],
    },
)
