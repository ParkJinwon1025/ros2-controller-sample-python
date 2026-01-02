from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test_nodes_py' # 패키지 이름 설정

# 패키지 정보 설정
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), # 설치할 Python 패키지 목록 자동 탐색
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], # 의존성 설정(pip가 자동으로 설치)
    zip_safe=True,  # 압축 가능 여부
    maintainer='ubisam',
    maintainer_email='ubisam@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_a = test_nodes_py.node_a:main',
            'node_b = test_nodes_py.node_b:main',
        ],
    },
)