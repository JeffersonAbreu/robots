from setuptools import find_packages, setup
import glob
import os
package_name = 'harm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Copie todos os arquivos da pasta 'worlds' para o diretório de instalação
        (os.path.join('share', package_name, 'worlds'), glob.glob(os.path.join('worlds', '*'))),
        # Copie todos os arquivos da pasta 'launch' para o diretório de instalação
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*')))
        # Copie todos os arquivos e subdiretórios da pasta 'models' para o diretório de instalação
        (os.path.join('share', package_name, 'models'), glob.glob(os.path.join('models', '**', '*'), recursive=True)),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jehffersson@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = harm.controller:main',
        ],
        'launch.frontend.launch_extension': ['launch = launch.frontend.launch_extension:LaunchExtension'],
    },
)
