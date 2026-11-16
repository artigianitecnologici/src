from setuptools import setup

package_name = 'marrtinorobot2_recognition'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/face_recognizer.launch.py',
            'launch/person_follower.launch.py',
        ]),
        ('share/' + package_name + '/params', [
            'params/face_recognizer.yaml',
            'params/person_follower.yaml',
        ]),
        ('share/' + package_name + '/models', [
            'models/face_detection_yunet_2026may.onnx',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS2 face recognition and lightweight person follower nodes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'face_recognizer = marrtinorobot2_recognition.face_recognizer_node:main',
            'person_follower = marrtinorobot2_recognition.person_follower_node:main',
        ],
    },
)
