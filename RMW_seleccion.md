
# **Introduccion**
Esta guia proporciona instrucciones sobre como configurar y usar el Middleware de ROS2 deseado.

## **Seleccion de ROS2 Middleware**
ROS2 fue construido sobre el protocolo de conexion DDS/RTPS (RTPS tambien conocido como DDSI-RTPS es un protocolo de cable usado por DDS para comunicarse atraves de la red) el cual proporciona descubrimiento, serializacion y transporte. Esta guia busca proporcionar las herramientas para poder configurar adecuadamente las diferentes implementaciones de DDS y el protocolo de conexion RTPS del DDS.

En la actualidad DDS es un estandar en la industria que ha sido implementado por diferentes proveedores, entre ellos RTI's Connext DDS, eProsima's Fast DDS,  Eclipse’s Cyclone DDS, y GurumNetworks’s GurumDDS. La seleccion de la implementacion del DDS para ROS2 dependera de diferentes factores, desde consideraciones logisticas como el tipo de licencia (Uso libre o uso comercial) hasta consideraciones de tipo tecnico como el tipo de plataforma para el que esta disponible y el impacto en recurso computacional de cada implementation. Cada uno de los proveedores proporciona una solucion orientada a diferentes necesidades, por eso la seleccion de este para una aplicacion especifica no debe tomarse de manera apresurada.

Esta guia unicamente explicara las dos implementaciones de uso libre que se encuentran disponibles en la actualidad: eProsima Fast DDS y Eclipse Cyclone DDS. Para mas informacion del tipo de licencia, revise la siguiente tabla.

| Nombre del producto | Licencia | Implementacion RMW | Estado de soporte |
|----|----|----|----|
| eProsima Fast DDS | Apache 2 | `rmw_fastrtps_cpp` | Soporte completo. RMW usado por defecto. Entregado en versiones binarias|
| Eclipse Cyclone DDS | Eclipse Public License v2.0 | `rmw_cyclonedds_cpp` | Soporte completo. Entregado en versiones binarias|
| RTI Connext | Comercial, Investigacion | `rmw_connext_cpp` | Soporte completo. Soporte incluido en versiones binarias, pero Connext instalado individualmente|
| GurumNetworks GurumDDS | Comercial | `rmw_gurumdds_cpp` | Soporte de comunidad. Soporte incluido en versiones binarias, pero GurumDDS instalado individualmente|

### **eProsima Fast DDS**
Para configurar FastRTPS necesitamos proporcionar un archivo de configuracion, este archivo es importante ya que nos permite especificar la interfaz o las interfaces de red que van a ser usadas durante la comunicacion. En caso de que el archivo de configuracion no sea proporcionado, FastRTPS utilizara todas las interfaces de red que se encuentres disponibles.

Encuentre el archivo de configuracion para FastRTPS en [configuracion_red/DEFAULT_FASTRTPS_PROFILES.xml](./configuracion_red/DEFAULT_FASTRTPS_PROFILES.xml). El archivo de configuracion proporcionado contiene una IP de ejemplo que es usada para comunicarse con otro dispositivo por medio de la interfaz elegida; sin embargo, esta IP debe ser modificada en cada caso. Recuerde que para esta implementacion diferentes interfaces de red pueden ser usadas simultaneamente, esto proporciona una gran ventaja cuando una de las interfaces de red presenta mayor latencia u otro tipo de inconvenientes que otra.

Para usar FastRTPS como middleware es necesario exportar y configurar dos variables de entorno, para hacerlo use los siguientes comandos:

- Exporte el archivo de configuracion de los perfiles por defecto:
  ```bash
  cd src/ros2_dds_setup/configuracion_red/
  export FASTRTPS_DEFAULT_PROFILES_FILE=/$PWD/DEFAULT_FASTRTPS_PROFILES.xml
  ```

- Exporte la implementacion RMW (FastRTPS) para ROS 2:
  ```bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ```

- **Nota:** En este punto puede exportar la distribucion de ROS 2 y el espacio de trabajo que contiene los paquetes ROS 2. Esta etapa de exportacion puede ser ejecutada antes o despues de exportar las variables de entorno mencionadas arriba.


### **Eclipse Cyclone DDS**
La configuracion de Eclipse Cyclone DDS es similar a la de eProsima Fast DDS, la unica diferencia toma lugar en el archivo de configuracion, el cual puede ser encontrado en [configuracion_red/cyclonedds_config.xml](./configuracion_red/cyclonedds_config.xml). El archivo de configuracion para este DDS nos permite seleccionar una sola interfaz de red que va a ser usada por nuestro sistema de ROS 2 (Note que esto es una diferencia con respecto a Fast DDS ya que en este ultimo era posible agregar mas de una interfaz de red para la comunicacion).

Para usar Cyclone DDS como middleware ese necesario exportar y configurar dos variables de entorno, para hacerlo use los siguientes comandos:

- Exporte el archivo de configuracion de los perfiles por defecto:
  ```bash
  cd src/ros2_dds_setup/configuracion_red/
  export CYCLONEDDS_URI=file://$PWD/cyclonedds_config.xml
  ```

- Exporte la implementacion RMW (Cyclone DDS) para ROS2:
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

- **Nota:** En este punto puede exportar la distribucion de ROS 2 y el espacio de trabajo que contiene los paquetes ROS 2. Esta etapa de exportacion puede ser ejecutada antes o despues de exportar las variables de entorno mencionadas arriba.

## Probemos nuestro codigo con diferentes RMW
Nuestro codigo de ejemplo esta basado en el [Tutorial Publicador-Subscriptor de Python](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html) y en el [Tutorial Publicador-Subscriptor de C++](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html), en caso de necesitar mayor informacion, dirijase a todos las paginas proporcionadas.
Agregar instrucciones entregadas en el README principal

Una vez haya completado la compilacion y el test en el espacio de trabajo, podemos correr nuestros nodos con diferentes RMW y verificar que todo corra de la forma correcta.

  - Ejecute tmux:
  ```sh
  tmux
  ```

  - Puede abrir dos terminales simultanemoanete al presionar `Ctrl-b` en inmediatamente despues `"`.

  - Exporte la distribucion de ROS 2 y el espacio de trabajo que contiene nuestros paquetes.
  ```sh
  source /opr/ros/foxy/setup.bash
  source install/setup.bash
  ```

### Seleccionando **eProsima Fast DDS**
Este paso es necesario ejecutarlo en cada una de las terminales donde vaya a lanzar un nodo en ROS 2 (Que tengan relacion entre si)

  - Exporte el archivo de configuracion de los perfiles por defecto:
  ```bash
  cd src/ros2_dds_setup/configuracion_red/
  export FASTRTPS_DEFAULT_PROFILES_FILE=/$PWD/DEFAULT_FASTRTPS_PROFILES.xml
  ```

  - Exporte la implementacion RMW (FastRTPS) para ROS 2:
  ```bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ```

  - Y ahora lance el nodo `talker` de ejemplo en su implementacion en Python:
  ```sh
  ros2 run py_pubsub talker
  ```

  - De forma alternativa, tambien podria usar el nodo `talker` en su implementacion en C++:
  ```sh
  ros2 run cpp_pubsub talker
  ```

En la otra ventana de la terminal (Puede cambiar entre estas divisiones al usar de forma sequencial los comandos `Ctrl-b` y las teclas `arriba` o `abajo`), no olvide ejecutar el mismo proceso de exportar los espacios de trabajo y especificar el RMW a utilizar:

  - Y ahora lance el nodo `lister` de ejemplo en su implementacion en Python:
  ```sh
  ros2 run py_pubsub listener
  ```

  - De forma alternativa, tambien podria usar el nodo `talker` en su implementacion en C++:
  ```sh
  ros2 run cpp_pubsub listener
  ```

### Seleccionando **Eclipse Cyclone DDS**
Este paso es necesario ejecutarlo en cada una de las terminales donde vaya a lanzar un nodo en ROS 2 (Que tengan relacion entre si)

  - Exporte el archivo de configuracion de los perfiles por defecto:
  ```bash
  cd src/ros2_dds_setup/configuracion_red/
  export CYCLONEDDS_URI=file://$PWD/cyclonedds_config.xml
  ```

  - Exporte la implementacion RMW (Cyclone DDS) para ROS2:
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ```

  - Y ahora lance el nodo `talker` de ejemplo en su implementacion en Python:
  ```sh
  ros2 run py_pubsub talker
  ```

  - De forma alternativa, tambien podria usar el nodo `talker` en su implementacion en C++:
  ```sh
  ros2 run cpp_pubsub talker
  ```

En la otra ventana de la terminal (Puede cambiar entre estas divisiones al usar de forma sequencial los comandos `Ctrl-b` y las teclas `arriba` o `abajo`), no olvide ejecutar el mismo proceso de exportar los espacios de trabajo y especificar el RMW a utilizar:

  - Y ahora lance el nodo `lister` de ejemplo en su implementacion en Python:
  ```sh
  ros2 run py_pubsub listener
  ```

  - De forma alternativa, tambien podria usar el nodo `talker` en su implementacion en C++:
  ```sh
  ros2 run cpp_pubsub listener
  ```


Por traducir!

You should see that in the `talker` pane you get logs every time a message is
sent with an increasing counter and in the `listener` pane you get the sent
message right after it was logged in the other one.

You can stop each node by pressing `Ctrl-c` and then exit each tmux pane by
`exit`ing the terminal session. You should return to the initial bash session
in the container.




For more information visit [ROS 2 DDS/RTPS vendors](https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html) and [ROS 2 middleware implementations](https://docs.ros.org/en/galactic/Concepts/About-Middleware-Implementations.html)