
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
Para configurar Fast DDS (Tambien conocido como FastRTPS como se puede observar en sus variables de entorno) necesitamos proporcionar un archivo de configuracion, este archivo es importante ya que nos permite especificar la interfaz o las interfaces de red que van a ser usadas durante la comunicacion. En caso de que el archivo de configuracion no sea proporcionado, Fast DDS utilizara todas las interfaces de red que se encuentres disponibles.

Encuentre el archivo de configuracion para Fast DDS en [configuracion_red/fastdds_profiles.xml](./configuracion_red/fastdds_profiles.xml). El archivo de configuracion proporcionado contiene una IP de ejemplo que es usada para comunicarse con otro dispositivo por medio de la interfaz elegida; sin embargo, esta IP debe ser modificada en cada caso. Recuerde que para esta implementacion diferentes interfaces de red pueden ser usadas simultaneamente, esto proporciona una gran ventaja cuando una de las interfaces de red presenta mayor latencia u otro tipo de inconvenientes que otra.

Para usar Fast DDS como middleware es necesario exportar y configurar dos variables de entorno, para hacerlo use los siguientes comandos:

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
  source /opt/ros/foxy/setup.bash
  source install/setup.bash
  ```

### Seleccionando **eProsima Fast DDS**
Este paso es necesario ejecutarlo en cada una de las terminales donde vaya a lanzar un nodo en ROS 2 (Que tengan relacion entre si)

  - Exporte el archivo de configuracion de los perfiles por defecto:
  ```bash
  cd src/ros2_dds_setup/configuracion_red/
  export FASTRTPS_DEFAULT_PROFILES_FILE=/$PWD/fastdds_profiles.xml
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

  - Y ahora lance el nodo `listener` de ejemplo en su implementacion en Python:
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

  - Y ahora lance el nodo `listener` de ejemplo en su implementacion en Python:
  ```sh
  ros2 run py_pubsub listener
  ```

  - De forma alternativa, tambien podria usar el nodo `talker` en su implementacion en C++:
  ```sh
  ros2 run cpp_pubsub listener
  ```

Una vez seleccionada nuestra implementacion de DDS, y los nodos de prueba `talker` y `listener` estan corriendo, en la consola de nuestro nodo `talker` deberiamos ver un mensaje "Hello World!" con un contador que se incrementa una vez por segundo, y en la consola de nuestro nodo `listener` deberiamos ver el mismo mensaje justamente despues de que fue mostrado en la otra consola.

- Consola del nodo Talker:
  ```sh
  [INFO] [1648569283.600949234] [minimal_publisher]: Publishing: "Hello World: 9"
  [INFO] [1648569284.101093229] [minimal_publisher]: Publishing: "Hello World: 10"
  [INFO] [1648569284.600692160] [minimal_publisher]: Publishing: "Hello World: 11"
  [INFO] [1648569285.100826438] [minimal_publisher]: Publishing: "Hello World: 12"
  [INFO] [1648569285.601028037] [minimal_publisher]: Publishing: "Hello World: 13"
  [INFO] [1648569286.100849896] [minimal_publisher]: Publishing: "Hello World: 14"
  [INFO] [1648569286.600548057] [minimal_publisher]: Publishing: "Hello World: 15"
  [INFO] [1648569287.100791272] [minimal_publisher]: Publishing: "Hello World: 16"
  ```

- Consola del nodo Listener:
  ```sh
  [INFO] [1648569283.116940841] [minimal_subscriber]: I heard: "Hello World: 8"
  [INFO] [1648569283.601321219] [minimal_subscriber]: I heard: "Hello World: 9"
  [INFO] [1648569284.101660286] [minimal_subscriber]: I heard: "Hello World: 10"
  [INFO] [1648569284.601298749] [minimal_subscriber]: I heard: "Hello World: 11"
  [INFO] [1648569285.101591780] [minimal_subscriber]: I heard: "Hello World: 12"
  [INFO] [1648569285.601753987] [minimal_subscriber]: I heard: "Hello World: 13"
  [INFO] [1648569286.101463539] [minimal_subscriber]: I heard: "Hello World: 14"
  [INFO] [1648569286.601341266] [minimal_subscriber]: I heard: "Hello World: 15"
  [INFO] [1648569287.101485094] [minimal_subscriber]: I heard: "Hello World: 16"
  ```

- **Nota:** En este caso el resultado es el mismo para ambas implementaciones.

Puede detener cualquiera de los nodos al presionar `Ctrl-c` y para terminar la sesion en cada terminal puede ejecutar `exit` o presiona `Ctrl-d`.

## Informacion adicional
En esta seccion puede encontrar informacion adicional sobre los diferentes proveedores de DDS para ROS 2. Aunque la mayoria de la informacion esta ingles, este es un buen complemento para entender en mayor profundidad los conceptos compartidos en esta guia. Para mayor informacion visits [Proveedores de DDS/RTPS para ROS 2](https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html) e [Implementaciones de Middleware para ROS 2](https://docs.ros.org/en/foxy/Concepts/About-Middleware-Implementations.html)
