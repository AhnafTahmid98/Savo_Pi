from setuptools import setup

package_name = 'savo_intent'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO_NAME',
    maintainer_email='todo@example.com',
    description='Bridge between llm_server and ROS topics/intents',
    license='MIT',
)
