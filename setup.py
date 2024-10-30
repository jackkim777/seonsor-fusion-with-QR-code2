from setuptools import setup
import os
import glob

package_name = 'qr_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch','*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yjchng',
    maintainer_email='yjchng@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_camera=qr_localization.qr_camera:main',
            'z_coord=qr_localization.z_coord_estimation:main',
            'QR_Position=qr_localization.QR_Position:main',
            'QrToRobot=qr_localization.QrToRobot:main'
        ],
    },
)
