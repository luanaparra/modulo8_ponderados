from setuptools import find_packages, setup

package_name = 'andar'

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
    maintainer='elisa',
    maintainer_email='elisaflemer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "chatbot=andar.chatbot:main",
            "vallet=andar.vallet:main",
            "init_pose=andar.initialize_pose:main",
            "queue=andar.poseQueue:main",
            "coordinatesChatbot=andar.coordinateChatbot:main"
        ],
    },
)
