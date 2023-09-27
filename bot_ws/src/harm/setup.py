from setuptools import setup
import os

package_name = 'harm'

# Função para coletar todos os arquivos em um diretório e seus subdiretórios
def collect_subdir_files(directory):
    data = []
    for (path, directories, filenames) in os.walk(directory):
        if filenames:
            files = [os.path.join(path, filename) for filename in filenames]
            data.append((path, files))
    return data

# Coletando os subdiretórios e arquivos em 'models'
model_data = collect_subdir_files('models')
world_data = collect_subdir_files('worlds')
launch_data = collect_subdir_files('launch')

# Adicionando esses subdiretórios e arquivos à lista data_files
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]
for subdir, files in model_data:
    data_files.append((os.path.join('share', package_name, subdir), files))
for subdir, files in world_data:
    data_files.append((os.path.join('share', package_name, subdir), files))
for subdir, files in launch_data:
    data_files.append((os.path.join('share', package_name, subdir), files))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seu Nome',
    maintainer_email='seu_email@example.com',
    description='Descrição do seu pacote',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = harm.controller:main',
        ],
    },
)
