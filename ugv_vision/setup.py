from setuptools import find_packages, setup

package_name = 'ugv_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['ugv_vision/vis_models/hand_landmarker.task']),
        ('share/' + package_name, ['ugv_vision/vis_models/vbh_dataset.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shreyansh',
    maintainer_email='shreyansh.sh99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pc_viewer_node = ugv_vision.pc_viewer_node:main",
            "ugv_cam_node = ugv_vision.ugv_cam_node:main",
            
        ],
    },
)


