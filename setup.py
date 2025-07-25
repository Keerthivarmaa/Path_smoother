from setuptools import find_packages, setup

package_name = 'smooth_path_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',['launch/final_launch.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keerthi',
    maintainer_email='keerthi003keerthi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'path_smoother = smooth_path_controller.path_smoother:main',
        	'trajectory = smooth_path_controller.trajectory_generator:main',
        	'controller = smooth_path_controller.controller:main',
        	'waypoints = smooth_path_controller.waypoints:main',
        
        ],
    },
)
