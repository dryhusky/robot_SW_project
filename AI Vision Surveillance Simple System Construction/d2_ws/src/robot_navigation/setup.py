from setuptools import setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'robot_navigation.init_pose',
        'robot_navigation.send_goal_stop',
        'robot_navigation.send_waypoint'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Robot navigation package for initial pose, goal setting, and waypoint following',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_pose = robot_navigation.init_pose:main',
            'send_goal_stop = robot_navigation.send_goal_stop:main',
            'send_waypoint = robot_navigation.send_waypoint:main',
        ],
    },
)

