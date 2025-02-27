from setuptools import find_packages, setup

package_name = 'martian_mines_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=open('requirements.txt').read().splitlines(),
    zip_safe=True,
    maintainer='highflyers',
    maintainer_email='franektrz4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bbox_publisher = martian_mines_ros2.bbox_publisher:main'
        ],
    },
)
