from setuptools import find_packages, setup

package_name = 'rasptank_alumno'
ejemplos = ['ej01_avanzar', 'ej02_cuadrado', 'ej03_evitar_obstaculos', 'ej04_seguir_linea',
            'ej05_ruta', 'ej06_brazo', 'ej07_coger_cubo', 'ej08_luces_sonido', 'plantilla_nodo']

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gbellido',
    maintainer_email='gbellido@mondragon.edu',
    description='Simple API and examples to program the RaspTank.',
    license='MIT',
    entry_points={
        'console_scripts': [f'{e} = rasptank_alumno.ejemplos.{e}:main' for e in ejemplos],
    },
)
