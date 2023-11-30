from setuptools import find_packages, setup

package_name = 'chat'

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
    maintainer='luana',
    maintainer_email='luana@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "chat=chat.chat:main",
            "input=chat.input:main",
            "llm=chat.llm:main",
            "output=chat.output:main",
        ],
    },
)
