# Tutorial Room & Pillar
El objetivo de este documento es instruir al lector en cómo utilizar y replicar los resultados más relevantes obtenidos durante la práctica en simulación de ambientes Room and Pillar. Este tutorial se divide en dos partes: la primera consiste en una explicación de cómo correr la simulación con el modelo final logrado por el practicante mientras que la segunda es una guía que explica el proceso de crear un ambiente propio. 
## 1. Cómo correr la simulación
Como se mencionó anteriormente, esta sección explicará cómo se puede correr el ambiente de simulación logrado al final de la práctica, que en este caso corresponde al modelo de una mina Room and Pillar. Para comenzar lo primero que necesitamos es tener los archivos de los resultados.
### 1.1 Obtener los archivos:
Todos los archivos necesarios para poder correr la simulación se encuentran en el siguiente Gitlab https://gitlab.com/finostro/3d-slam/-/tree/master/amtc-subt-testbed/amtc-packages, de donde se deben descargar todos los contenidos de la carpeta Modelo_Room_Pillar. Lo primero es una carpeta llamada models, esta contiene los archivos de todos los modelos que fueron modificados de alguna u otra manera, los cuales tendremos que reemplazar una vez creado un container nuevo. Lo segundo es un archivo sdf que corresponde al mundo, es decir el modelo de la mina por el que se moverá el vehículo. En este archivo se indica la posición tridimensional de las distintas piezas utilizadas y tiene un nombre particular, esto se debe a que en vez de crear un mundo nuevo reemplazaremos la información de uno existente. Correr la simulación desde un mundo completamente custom puede llevar a ciertos problemas que podemos evitar fácilmente aplicando este truco. Por último tenemos un pequeño script que nos ayuda a reemplazar los archivos, pero de todos modos se explicará como reemplazar modelos y mundos manualmente en la segunda parte del tutorial.
### 1.2 Iniciar un contenedor:
En este tutorial se asume que el usuario ya ha instalado Docker. Si este no es el caso seguir el siguiente tutorial: https://github.com/osrf/subt/wiki/Docker%20Install. Ocuparemos una versión modificada de los paquetes de Darpa Subterranean Challange que contiene el nuevo vehículo y configuraciones de sensores, entre otras cosas. Para esto se debe descargar la carpeta amtc-subt-testbed del siguiente git: https://gitlab.com/finostro/3d-slam. En este caso se realizó el pull desde la carpeta personal, ya que esto nos da un acceso más directo a amtc-subt-testbed.

Luego para iniciar un contenedor basta correr las siguientes líneas de código:

    cd amtc-subt-testbed/
    bash script/build
    bash script/run
    catkin_make install

Estas se deben correr cada vez que uno quiera iniciar un nuevo contenedor a excepción de la segunda, que sólo es necesaria la primera vez. Con esto tenemos nuestro container listo para empezar a simular con los mundos y modelos predeterminados, pero lo que queremos es lograr correr los mundos y modelos custom así que veremos como hacer eso a continuación.

Si se desea se puede comprobar que todo haya funcionado abriendo una simulación con uno de los mapas predeterminados con la siguiente línea en la terminal del contenedor:

    ign launch -v 4 cave_circuit.ign    worldName:=cave_circuit_practice_01    robotName1:=X1    robotConfig1:=AMTC_LHD_SENSOR_CONFIG_1 localModel:=true enableGroundTruth:=true
    
Debiese verse algo como la siguiente imagen. Más adelante se muestra como manejar el camión.
![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/Captura%20de%20pantalla%20de%202021-02-03%2000-10-19.png)
### 1.3 Correr el script:
Antes de correr el script debemos asegurarnos de que la carpeta que contiene los archivos se encuentre en el directorio de preferencia (en este caso Downloads) y modificar algunos aspectos del script. A continuación se muestra el código contenido en el script:

```shell
#!/bin/sh

container_id=""
echo $container_id
host_path="/home/idassori/Downloads/Modelo_Room_Pillar"
echo $host_path
container_path="/home/developer/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models"

# Mundo
docker cp $host_path/cave_circuit_practice_01.sdf $container_id:/home/developer/subt_ws/install/share/subt_ign/worlds/cave_circuit_practice_01.sdf

# Paquetes
docker cp $host_path'/models/cave straight 01 type b'                    $container_id:$container_path
docker cp $host_path'/models/cave starting area type b'                  $container_id:$container_path
docker cp $host_path'/models/cave straight 02 type b'                    $container_id:$container_path
docker cp $host_path'/models/cave straight 04 type b'                    $container_id:$container_path
docker cp $host_path'/models/cave straight 05 type b'                    $container_id:$container_path
docker cp $host_path'/models/cave cap type b'                            $container_id:$container_path
docker cp $host_path'/models/rough tunnel tile 4-way intersection'       $container_id:$container_path
docker cp $host_path'/models/cave 3 way 01 type b'                       $container_id:$container_path
```
Lo primero es asegurarse que el path del host sea correcto, ya que la carpeta Modelo_Room_Pillar puede encontrarse en otro directorio. Notamos que la variable container_id esta vacia, aqui debemos rellenar con el id de nuestro contenedor (container_id debe quedar como un string!). Para saber cual es la id corremos en un nuevo terminal la siguiente línea:

    docker ps
  
Por ejemplo en el caso de la imagen el id es 634c607d0d1c.
![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/Captura%20de%20pantalla%20de%202021-02-03%2009-22-24.png)

Una vez hecho esto solo queda correr el script, para esto nos dirigimos al directorio donde esté ubicado y lo ejecutamos.

    cd ~/Downloads/Modelo_Room_Pillar
    bash copiar_mina_sal.sh
    
Probemos ahora correr nuevamente la simulación y deberíamos ver una mina distinta con los modelos personalizados como se muestra en la siguiente imagen:
![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/Captura%20de%20pantalla%20de%202021-02-03%2010-24-13.png)

### 1.4 Controlar el vehículo:
Para manejar el camión usamos un comando teleop personalizado que viene incluido en los paquetes del amtc previamente instalados. En una nueva terminal abrimos un contenedor, pero es importante que el contenedor original siga abierto, ya que necesitamos su id.

    docker exec -it container_id /bin/bash
    roslaunch amtc_lhd_teleop teleop.launch
    
Se debe conectar un joystick ya sea por usb o bluetooth, de lo contrario aparecerá un error al correr teleop que nos indicará que no se ha hayado un joystick y que seguirá buscando cada segundo. En controles de Xbox el camión se maneja manteniendo presionado el botón A y virando con el stick izquierdo. En controles de PS es el botón X el que se debe mantener presionado.

### 1.5 Visualizar data de sensores:
Ahora que tenemos corriendo tanto la simulación como teleop y podemos manejar el vehículo por la mina veamos cómo podemos visualizar la información registrada por los sensores. Para esto usamos Rviz, en una nueva terminal lo corremos con la siguiente línea:

    rosrun rviz rviz
    
Debiese aparecer una ventana como la que se muestra a continuación:
![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/rviz.png)

Cambiamos el Fix Frame a uno que nos acomode más, esta será la perspectiva que muestra Rviz. Podemos elegir X1 para seguir al vehículo mientras este recorre el túnel. Abajo a la izquierda seleccionamos Add para añadir un método de visualización, escogamos PointCloud2 que fue el que usamos durante la práctica para visualizar las paredes de la mina mediante el sensor láser. Luego se debe seleccionar uno de los tópicos del camión, en este caso /X1/front_laser_velodyne/points con lo que deberíamos poder ver la data registrada por el sensor. Se pueden variar parámetros como el grosor e intensidad de las líneas para lograr una mejor visualización.
![Alt Text]()
### 1.6 Grabar rosbag:

### 1.7 Reproducir rosbag:
