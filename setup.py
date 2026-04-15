from setuptools import setup

package_name = 'rcomponent'

setup(
	name=package_name,
	version='0.0.1',
	packages=[package_name],
	package_dir={'': '.'},
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],

	install_requires=['setuptools'],

	zip_safe=True,
)