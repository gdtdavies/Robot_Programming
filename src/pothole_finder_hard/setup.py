from setuptools import find_packages, setup

package_name = 'pothole_finder_hard'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate = pothole_finder_hard.navigate:main',
            'map_potholes = pothole_finder_hard.map_potholes:main',
            'collect_data = pothole_finder_hard.collect_data:main',
        ],
    },
)
