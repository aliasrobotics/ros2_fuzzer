from setuptools import setup

setup(
    name='ros2_fuzzer',
    version='1.0.0',
    packages=['ros2_fuzzer'],
    url='https://www.aliasrobotics.com',
    license='GPLv3',
    author='Alias Robotics',
    author_email='contact@aliasrobotics.com',
    description='A ROS2 fuzzing tool for ROS2 systems',
    keywords=['network', 'fuzzing', 'ros', 'ros2'],
    entry_points={
        'console_scripts': ['ros2_fuzzer=ros2_fuzzer.ros_fuzzer:main'],
    },
    install_requires=[
        'hypothesis==3.82',
        'attrs==19.1.0',
        'numpy==1.16.3',
    ],
    include_package_data=True,
    python_requires='>=3'
)
