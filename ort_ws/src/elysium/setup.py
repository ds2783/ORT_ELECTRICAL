from setuptools import find_packages, setup

package_name = 'elysium'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'qreader', 'picamera2'],
    zip_safe=True,
    maintainer='william',
    maintainer_email='williamwarrenmeeks@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_cam = elysium.qr_cam:main',
            'teleop = elysium.teleop:main'
        ],
    },
)
