from setuptools import setup

package_name = 'savo_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[('share/' + package_name + '/config', ['config/safety.yaml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO_NAME',
    maintainer_email='todo@example.com',
    description='Navigation helpers: safety, kinematics, Nav2 interface',
    license='MIT',
)
