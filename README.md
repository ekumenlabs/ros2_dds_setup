# ekuabc

## ROS2 Foxy guidelines
Projecto base que contiene una imagen Docker para construir un contenedor con Ubuntu 20.04 como sistema operativo principay y la distribucion de ROS 2 Foxy (Version completa) con algunos paquetes de demostracion.

## Integracion continua (CI)
La integracion continua proporcionada en este repositorio se basa en las acciones de Github, estas acciones se encargan de configurar un entorno con ROS 2 Foxy
para compilar (Buildear) y testear dos paquetes basicos. Si alguna dependencia adicional es requerida, esta no podra ser gestionada por `rosdep`, por lo tanto
debe ejecutar los pasos de instalacion de dicha dependencia antes de ejecutar `action-ros-ci`.

## Docker
Esta seccion proporciona pasos para la configuracion del contenedor Docker.

- Soporte de NVIDIA GPU: *Omita los siguientes dos pasos si no cuenta con una tarjeta de video NVIDIA.*
  - Verifique que cuenta con los drives NVIDIA instalados, para ello abra una nueva terminal y ejecute el siguiente comando:
    ```sh
    nvidia-smi
    ```

  - Si cuenta con los drivers y una tarjeta de video NVIDIA, instale `nvidia-container-toolkit` en el ordenador (host machine):
    ```sh
    sudo apt-get install -y nvidia-container-toolkit
    ```

- Construya la imagen de Docker cuyo nombre es `ros2_foxy`:

  ```sh
  ./docker/build.sh
  ```

- Puede renombrar la imagen si es necesario (Opcional):

  ```sh
  ./docker/build.sh -i my_fancy_image_name
  ```

- Corra el contenedor Docker a partir `ros2_foxy` con el nombre `ros2_foxy_container`:

  ```sh
  ./docker/run.sh
  ```

- Tambien puede intentar especificar el nombre de la imagen y el contendor:

  ```sh
  ./docker/run.sh -i my_fancy_image_name -c my_fancy_container_name
  ```

## Preparacion del espacio de trabajo (Workspace), compilacion y testeo.
La preparacion del espacio de trabajo debe ejecutarse dentro del contenedor.

- Para la instalacion de las dependencias del espacio de trabajo a través de `rosdep`:
  ```sh
  rosdep install -i -y --rosdistro foxy --from-paths src
  ```

- Ahora para la compilacion de nuestros paquetes (Publicadores y Subscriptores basicos en C++ y Python) ejecute el siguiente comando:
  ```sh
  colcon build
  ```

- Para obtener detalles adicionales de la compilacion, puede agregar la siguiente opcion al comando anterior:
  ```sh
  colcon build --event-handlers console_direct+
  ```

- Finalmente para poder ejecutar los test implementados para cada uno de los paquetes:
  ```sh
  colcon test --event-handlers console_direct+
  colcon test-result
  ```

## Probemos el codigo!
En este caso la prueba de nuestro codigo la ejecutaremos con dos diferentes Middlewares, para ello acceda a la pagina [RMW_seleccion](RMW_seleccion.md)

