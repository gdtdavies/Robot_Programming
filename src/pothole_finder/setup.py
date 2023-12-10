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
    maintainer_email='gdtdavies@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_pothole = pothole_finder.find_pothole:main',
            'waypoint_follower = pothole_finder.waypoint_follower:main',
            'navigator = pothole_finder.navigator:main',
            'image_projection = pothole_finder.image_projection:main',
            'pothole_counter = pothole_finder.pothole_counter:main',
        ],
    },
)
