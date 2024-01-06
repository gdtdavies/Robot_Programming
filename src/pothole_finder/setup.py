from setuptools import find_packages, setup

package_name = 'pothole_finder'

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
    maintainer='george',
    maintainer_email='27421138@students.lincoln.ac.uk',
    description='In this pakage you will find code for the assignment of the UOL CMP9767M module in conjuction with the limo_ros2 repository. \
        It contains code for the navigation of the limo robot and the detection of potholes. \
        The goal is to navigate the limo robot through the environment and detect potholes. \
        The potholes are detected using a color blob detection algorithm for the easy detection of the potholes \
        and yolo for the detection of the potholes on the regular version of the map. \
        With the navigator, the robot is able to drive around the map \
            ',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'count_potholes = pothole_finder.count_potholes:main',
            'map_potholes = pothole_finder.map_potholes:main',
            'mask_potholes = pothole_finder.mask_potholes:main',
            'navigate = pothole_finder.navigate:main',
            'nav_to_pose = pothole_finder.nav_to_pose:main',
        ],
    },
)
