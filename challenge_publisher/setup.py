from setuptools import find_packages, setup

package_name = 'challenge_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bryan Luco',
    maintainer_email='bluco55@gmail.com',
    description='Publishes Magnetometer/Angular data from PX4 flight log',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher = challenge_publisher.mag_ang_publisher:main'
        ],
    },
)
