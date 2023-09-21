from setuptools import setup

package_name = 'husky_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alexqixuan@gmail.com',
    description='MPC Node for Husky Obstacle Avoidance',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mpc_node = husky_mpc.mpc_node:main"
        ],
    },
)
