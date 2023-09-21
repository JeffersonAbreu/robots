from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_package'

# Coleta os arquivos usando glob e constrói o caminho de destino usando os.path.join
def collect_files(source_directory, pattern):
    source_files = glob(os.path.join(source_directory, pattern))
    dest_directory = os.path.join('share', package_name, source_directory)
    return [(dest_directory, source_files)]

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml'])
] 

data_files.extend(collect_files('launch', '*.launch.py'))
data_files.extend(collect_files('worlds', '*.world'))
data_files.extend(collect_files('models', '**/*.*'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jehffersson@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = my_package.controller:main'
        ],
    },
)
