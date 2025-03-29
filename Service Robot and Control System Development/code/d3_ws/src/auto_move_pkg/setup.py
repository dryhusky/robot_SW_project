from setuptools import setup

package_name = 'auto_move_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Auto move package for TurtleBot3',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_move = auto_move_pkg.auto_move:main',
        ],
    },
)

