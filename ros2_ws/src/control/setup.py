from setuptools import find_packages, setup

package_name = 'control'

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
    maintainer='Cook131',
    maintainer_email='cloudkid075@gmail.com',
    description='TODO: Esto es un PID bonito',
    license='TODO: licencia',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                    'control_servoing = control.control_servoing:main',
        ],
    },
)
