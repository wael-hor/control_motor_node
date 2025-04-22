from setuptools import setup

package_name = 'pi_usb_camera'

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
    maintainer='Wael',
    maintainer_email='wael@example.com',
    description='Publishes USB camera feed from Raspberry Pi using OpenCV',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = pi_usb_camera.camera_node:main',
        ],
    },
)
