import os # Operating system library
from glob import glob # Handles file path names
from setuptools import setup # Facilitates the building of packages

package_name = 'my_camera_calibration'

data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:
        
        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                    
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'config'],
    data_files=package_files(data_files, ['config/', 'launch/', 'my_camera_calibration/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jehffersson@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_aruco = my_camera_calibration.move_aruco:main',
            'capture_images = my_camera_calibration.capture_images:main',
        ],
    },
)