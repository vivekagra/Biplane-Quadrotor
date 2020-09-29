from setuptools import setup
from setuptools import find_packages

setup(	
    name='biplane_quad',
    version='0.0.0',
    description='RL with trajectory control',
    url='',
    license='MIT',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    zip_safe=False,
)