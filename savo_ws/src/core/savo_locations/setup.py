from setuptools import setup

package_name = 'savo_locations'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[('share/' + package_name, ['locations.yaml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO_NAME',
    maintainer_email='todo@example.com',
    description='Static known locations for Robot Savo navigation',
    license='MIT',
)
