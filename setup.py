from setuptools import find_packages, setup

package_name = 'ros2_vehicle_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mizdrak',
    maintainer_email='markomizdrak@gmail.com',
    description='simple car model,
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'vehicle_model = ros2_vehicle_mmodel.simulator:main',
        ],
    },
    
)
