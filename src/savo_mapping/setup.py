from setuptools import setup

package_name = 'savo_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[('share/' + package_name + '/config', ['config/slam_toolbox.yaml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO_NAME',
    maintainer_email='todo@example.com',
    description='SLAM, map saving, exploration setup',
    license='MIT',
)
