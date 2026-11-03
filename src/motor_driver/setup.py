from setuptools import setup
import glob

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    # >>> ONLY INCLUDE THE PYTHON PACKAGE <<<
    packages=[package_name],  # <<==== ONLY THIS LINE
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lyes',
    maintainer_email='lyes@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_driver_node = motor_driver.motor_driver_node:main",
            "odometry_node = motor_driver.odometry_node:main",
            'pid_circular_node = motor_driver.pid_circular_node:main',
            'carry_pid_controller = motor_driver.carry_pid_node:main',
            "mpc_circulaire = motor_driver.mpc_circulaire:main",
            "smc_infinity_node = motor_driver.smc_infinity_node:main",
            "SMC_circular = motor_driver.SMC_circular:main",
            "pid_infiny = motor_driver.pid_infiny:main",
            "plot = motor_driver.plot:main",
            "plot1 = motor_driver.plot1:main",
            "pid_plot = motor_driver.pid_plot:main",
            "pid_plot_infiny = motor_driver.pid_plot_infiny:main"
        ],
    },
)
