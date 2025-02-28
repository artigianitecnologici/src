from setuptools import find_packages, setup

package_name = 'marrtinorobot2_voice'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (f'share/{package_name}/launch', ['launch/tts_node.launch.py']), 
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


