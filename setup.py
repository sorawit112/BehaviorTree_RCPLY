from setuptools import find_packages, setup

package_name = 'behaviour_tree_rclpy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['resource/py_tree_nodes.xml']),
        ('share/' + package_name, ['resource/default_tree.xml']),
        ('share/' + package_name, ['resource/test_trigger_service.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sorawit-inprom',
    maintainer_email='sorawit.112@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_node = behaviour_tree_rclpy.bt_node:main',
            'mockup_tigger = behaviour_tree_rclpy.mockup_server_node.mockup_trigger_server:main',
        ],
    },
)
