from setuptools import find_packages, setup

package_name = 'status_panel'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/status_panel']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pitosalas',
    maintainer_email='pitosalas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'status_panel_node = status_panel.status_panel_node:main',
            'display_ros = status_panel.display_ros:main',
            'display2 = status_panel.display2:main',
            'demo = status_panel.demo:main',
        ],
    },
)
