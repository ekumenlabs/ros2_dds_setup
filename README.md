# Tutorial de configuración del RMW para ROS 2
Este proyecto busca proporcionar un tutorial practico y sencillo para entender y dominar la modificación del RMW para ROS 2. Dentro de este proyecto un contenedor es entregado con todas las herramientas necesarias para poder ejecutar los ejemplos explicados en esta guía.

## Información del proyecto
Proyecto base que contiene una imagen Docker para construir un contenedor con Ubuntu 20.04 como sistema operativo principal y la distribución de ROS 2 Foxy (Version completa) con algunos paquetes de demostración.
## Integración continua (CI)
La integración continua proporcionada en este repositorio se basa en las acciones de Github, estas acciones se encargan de configurar un entorno con ROS 2 Foxy para compilar (Build) y testear dos paquetes básicos. Si alguna dependencia adicional es requerida, esta no podra ser gestionada por `rosdep` y por lo tanto debe ejecutar los pasos de instalación de dicha dependencia antes de ejecutar `action-ros-ci`.

## Docker
Esta sección proporciona pasos para la configuración del contenedor Docker.

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
  ./docker/build.sh -i mi_imagen
  ```

- Inicie el contenedor Docker basado en `ros2_foxy` con el nuevo nombre `ros2_foxy_container`:

  ```sh
  ./docker/run.sh
  ```

- También puede intentar especificar el nombre de la imagen y el contendor:

  ```sh
  ./docker/run.sh -i mi_imagen -c my_fancy_container_name
  ```

## Preparación del espacio de trabajo (Workspace), compilación y testeo.
La preparación del espacio de trabajo debe ejecutarse dentro del contenedor.

- Para la instalación de las dependencias del espacio de trabajo a través de `rosdep`:
  ```sh
  rosdep install -i -y --rosdistro foxy --from-paths src
  ```

- Ahora para la compilación de nuestros paquetes (Publicadores y Subscriptores basicos en C++ y Python) ejecute el siguiente comando:
  ```sh
  colcon build
  ```

- Para obtener detalles adicionales de la compilación, puede agregar la siguiente opción al comando anterior:
  ```sh
  colcon build --event-handlers console_direct+
  ```

- Finalmente para poder ejecutar los test implementados para cada uno de los paquetes:
  ```sh
  colcon test --event-handlers console_direct+
  colcon test-result
  ```

## Probemos el código!
En este caso la prueba de nuestro código la ejecutaremos con dos diferentes Middlewares, para ello acceda a la página [RMW_seleccion](RMW_seleccion.md)
