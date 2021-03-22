# Tutorial Room & Pillar
El objetivo de este documento es instruir al lector en cómo utilizar y replicar los resultados más relevantes obtenidos durante la práctica en simulación de ambientes Room and Pillar. Este tutorial se divide en dos partes: la primera consiste en una explicación de cómo correr la simulación con el modelo final logrado por el practicante mientras que la segunda es una guía que explica el proceso de crear un ambiente propio. 
## 1. Cómo correr la simulación
Como se mencionó anteriormente, esta sección explicará cómo se puede correr el ambiente de simulación logrado al final de la práctica, que en este caso corresponde al modelo de una mina Room and Pillar. Para comenzar lo primero que necesitamos es tener los archivos del simulador.
### 1.1 Obtener los archivos:
Todos los archivos necesarios para poder correr la simulación se encuentran en el siguiente Gitlab https://gitlab.com/finostro/3d-slam/-/tree/master/amtc-subt-testbed/amtc-packages, de donde se deben descargar todos los contenidos de la carpeta Modelo_Room_Pillar. Lo primero es una carpeta llamada models, esta contiene los archivos de todos los modelos que fueron modificados de alguna u otra manera, los cuales tendremos que reemplazar una vez creado un container nuevo. Lo segundo es un archivo sdf que corresponde al mundo, es decir el modelo de la mina por el que se moverá el vehículo. En este archivo se indica la posición tridimensional de las distintas piezas utilizadas y tiene un nombre particular, esto se debe a que en vez de crear un mundo nuevo reemplazaremos la información de uno existente. Correr la simulación desde un mundo completamente custom puede llevar a ciertos problemas que podemos evitar fácilmente aplicando este truco. Por último tenemos un pequeño script que nos ayuda a reemplazar los archivos, pero de todos modos se explicará como reemplazar modelos y mundos manualmente en la segunda parte del tutorial.
### 1.2 Iniciar un contenedor:
En este tutorial se asume que el usuario ya ha instalado Docker. Si este no es el caso seguir el siguiente tutorial: https://github.com/osrf/subt/wiki/Docker%20Install. Ocuparemos una versión modificada de los paquetes de Darpa Subterranean Challange que contiene el nuevo vehículo y configuraciones de sensores, entre otras cosas. Para esto se debe descargar la carpeta amtc-subt-testbed del siguiente git: https://gitlab.com/finostro/3d-slam. En este caso se realizó el pull desde la carpeta personal, ya que esto nos da un acceso más directo a amtc-subt-testbed.

Luego para iniciar un contenedor basta correr las siguientes líneas de código:

    git clone https://gitlab.com/finostro/3d-slam.git
    cd ~/3d-slam/amtc-subt-testbed/
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
host_path="/home/Usuario/Downloads/Modelo_Room_Pillar"
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
Lo primero es asegurarse que el path del host sea correcto, ya que la carpeta Modelo_Room_Pillar puede encontrarse en otro directorio. Además, el nombre del usuario debe cambiarse según el que se tenga en la computadora. Notamos que la variable container_id esta vacia, aqui debemos rellenar con el id de nuestro contenedor (container_id debe quedar como un string!). Para saber cual es la id corremos en un nuevo terminal la siguiente línea:

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

Cambiamos el Fixed Frame a uno que nos acomode más, esta será la perspectiva que muestra Rviz. Podemos elegir X1 para seguir al vehículo mientras este recorre el túnel. Abajo a la izquierda seleccionamos Add para añadir un método de visualización, escogamos PointCloud2 que fue el que usamos durante la práctica para visualizar las paredes de la mina mediante el sensor láser. Luego se debe seleccionar uno de los tópicos del camión, en este caso /X1/front_laser_velodyne/points con lo que deberíamos poder ver la data registrada por el sensor. Se pueden variar parámetros como el grosor e intensidad de las líneas para lograr una mejor visualización.
![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/rviz_cloudpoint2.png)

### 1.6 Grabar y reproducir con rosbag:
Por último se explicará brevemente cómo grabar información de los tópicos en un bagfile usando rosbag, y cómo reproducir esta información después. Cabe mencionar que durante la práctica se grabaron 16 bagfiles de recorridos por la mina con diferentes configuraciones de sensores, sin embargo estos archivos no se encuentran en un git debido a su gran peso. Para grabar un bagfile primero se deben tener corriendo los contenedores de la simulación y teleop de modo que tengamos tópicos siendo publicados. Podemos ver información sobre los tópicos con rostopic list -v. Para grabar ejecutamos las siguientes líneas:

    mkdir ~/bagfiles
    cd ~/bagfiles
    rosbag record -a
    
Con esto grabaremos todos los tópicos publicados, sin embargo quizás no querramos grabar algunos tópicos, como la imagen registrada por la cámara del camión, que puede llegar a pesar bastante dependiendo del largo del recorrido. En este caso podemos cambiar -a por un listado de los tópicos que nos interesa grabar, como en el siguiente ejemplo que ḿuestra los tópicos grabados para los bagfiles de la práctica:

    rosbag record /X1/FL_wheel/command /X1/FR_wheel/command /X1/RL_wheel/command /X1/RR_wheel/command /X1/base_link_to_front_base_link_joint/cmd_vel /X1/bucket_boom_joint/command /X1/bucket_bucket_joint/command /X1/cmd_vel /X1/front/optical/depth /X1/front_laser_hokuyo_sensor/points /X1/front_laser_velodyne/points /X1/imu/data /X1/odom /X1/plan_cmd_vel /X1/pose /X1/pose_static /X1/rear_laser_hokuyo_sensor/points /X1/rear_laser_velodyne/points /clock /diagnostics /joy /joy/set_feedback /rosout /rosout_agg /subt/run_clock /subt/score /tf /tf_static

Para reproducir el bagfile debemos tener instalada una versión modificada de loam_velodyne que encontramos en el siguiente link que contiene instrucciones: https://gitlab.com/finostro/loam_velodyne. Con esto podemos pasar a reproducir, notar que no es necesario tener ningún contenedor corriendo para esto. Corremos loam_velodyne, lo que debería abrir una ventana similar a la vista al correr Rviz:

    roslaunch loam_velodyne loam_velodyne.launch
    
En una nueva terminal reproducimos el bagfile que hayamos grabado:

    rosbag play ~/bagfiles/file.bag

## 2. Crear un ambiente de simulación
Para crear un ambiente de simulación hay dos elementos principales con los que hay que trabajar, mundos y modelos. Para esto debemos saber modificar archivos .sdf y .dae, que será lo que veremos a continuación.

### 2.1 Modificar mundo
Primero clonamos el repositorio, con esto podremos acceder a los archivos de los mundos con mayor facilidad y después de modificarlos bastará con reemplazarlos en el contenedor:

    cd ~
    git clone https://github.com/osrf/subt.git
    
Ahora tenemos una carpeta subt en nuestra carpeta personal (u otro directorio si así se escogió). Para llegar a los mundos seguimos el siguiente path: subt/subt_ign/worlds/. En esta última carpeta tenemos los archivos .sdf de los distintos mundos que vienen con el repositorio, como se mencionó anteriormente en vez de crear nuestro propio archivo de mundo modificaremos uno ya existente, en el caso de prueba elegimos cave_circuit_practice_01.sdf. Si abrimos el archivo con un editor de texto notaremos que tienen varios tags, pero a nosotros sólo nos interesa <include>, ya que este llama a los modelos y les da su posición en el espacio. Muchas veces el resto de los elementos que no sean de este tipo se pueden simplemente borrar con algunas excepciones. Se puede empezar borrando algunos elementos o mejor aún reemplazar todo por lo siguiente, que es un mundo que solamente contiene la entrada a la mina:
    
    <?xml version="1.0" ?>
    <sdf version="1.6">
      <world name="cave_circuit_practice_01">

        <physics name="1ms" type="ode">
          <max_step_size>0.004</max_step_size>
          <real_time_factor>1.0</real_time_factor>
        </physics>

        <scene>
          <ambient>0.1 0.1 0.1 1.0</ambient>
          <background>0 0 0 1.0</background>
          <grid>false</grid>
          <origin_visual>false</origin_visual>
        </scene>

        <!-- The staging area -->

        <include>
          <static>true</static>
          <name>staging_area</name>
          <pose>0 6 0 0 0 0</pose>
          <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Cave Starting Area Type B</uri>
        </include>

        <include>
          <name>artifact_origin</name>
          <pose>10 0.0 0.0 0 0 0</pose>
          <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Fiducial</uri>
        </include>

      </world>
    </sdf>
    
A partir de esta base se pueden ir agregando piezas ocupando el tag <include>, este requiere de ciertos parámetros. Por defecto siempre mantenemos <static> como true para los modelos, y es importante que todos los nombres sean distintos, de lo contrario la simulación no cargará. <pose> determina la posición en el espacio del modelo, siendo los tres primeros números las coordenadas en los ejes x, y, z y las últimas tres los ángulos de rotación con respecto a los ejes. Por último, <uri> llama al modelo o pieza, para esto se debe entregar un enlace como el que se ve en el ejemplo. En https://docs.google.com/spreadsheets/d/1P-aFQXw79qmT6hsnNeHionBNCPDuD1D6a7Qvi7W_NoU/edit?usp=sharing se elaboró una tabla que muestra el nombre de todos las piezas del repositorio, y en la pestaña cave se listan todas aquellas que corresponden a pedazos de cueva incluyendo fotos y sus uri. Esto es todo lo que se necesita para crear un mundo personalizado, ahora veremos como reemplazar este archivo .sdf en el contenedor para poder verlo en la simulación.
    
Para reemplazar nuestro archivo de mundo modificado corremos lo siguiente desde una terminal fuera del contenedor. Notar que se debe reemplazar Usuario por el nombre de usuario del pc y se necesita la id del contenedor, por lo que si no se tiene se deberá correr antes docker ps.

    cd ~/subt/docker
    docker cp /home/Usuario/subt/subt_ign/worlds/cave_circuit_practice_01.sdf container_id:/home/developer/subt_ws/install/share/subt_ign/worlds/cave_circuit_practice_01.sdf
    
Ahora al correr la simulación se verá algo similar a lo siguiente, donde la única pieza es la entrada del túnel:
![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/Captura%20de%20pantalla%20de%202021-02-03%2014-21-44.png)

### 2.2 Modificar modelos
Hay muchas formas de modificar los modelos que vienen en el repositorio subt. Se pueden escalar sus ejes para aumentar o disminuir sus tamaños, eliminar rocas o escombros presentes en la pieza, o entrar en procesos más complicados como alterar la forma de la pieza mediante el uso de softwares como meshlab y blender. En este tutorial se explicará como hacer las dos primeras modificaciones y se darán algunas recomendaciones para poder modificar los modelos con blender, ya que no hay una forma única de hacerlo y muchas veces termina siendo un proceso de prueba y error.

Para empezar debemos hacer una copia de la carpeta que contiene los archivos de los modelos, a la cual aún no tenemos acceso pero se encuentra dentro del contenedor. Para copiarla desde el contenedor a nuestra carpeta personal ejecutamos lo siguiente, tomando nuevamente en consideración usar el path correcto y la id del contenedor:

    docker cp container_id:/home/developer/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models /home/Usuario/models

En la carpeta models veremos que cada modelo tiene su propia carpeta, y dentro de esta una subcarpeta con un número. Dentro de esta última encontramos los archivos de interés, model.sdf y la carpeta meshes. Primero veamos el archivo sdf. Cada modelo tiene dos partes que lo conforman: visuales y colisión. En general en model.sdf veremos que hay un solo elemento <collision> y un <visual> para cada nodo del modelo. Los nodos son partes que juntas hacen el pedazo de tunel, estos pueden ser paredes, rocas y escombros. En el tag <mesh> de cada elemento vemos que se hace referencia a un uri que corresponde a un archivo .dae. Como se mencionó anteriormente, estos son los otros archivos de interés y son los que se encuentran en la carpeta meshes. Los veremos en detalle más adelante. Lo importante es que en el mismo nivel del tag <uri> podemos agregar un nuevo parámetro si no esta ya presente. Este es <scale>, que nos permite escalar el modelo en los tres ejes. Para modificar la escala del modelo ed manera correcta se debe poner la misma escala en cada uno de los <mesh> de model.sdf, ya que sino podemos terminar con un túnel con colisión y visuales de tamaños diferentes.
    
    <!-- escala predeterminada -->
    <scale>1.0 1.0 1.0</scale>
    
Como se mencionó antes, cada nodo tiene su propia visual, así que si se desea eliminar cualquier escombro o rocas que tapen el camino de un túnel, podemos empezar por eliminar sus visuales en model.sdf. Sin embargo, aunque las rocas ya no se vean en la simulación, con esto no hemos quitado su colisión. Para modificar las colisiones de un modelo debemos modificar los archivos .dae que se encuentran dentro de la carpeta meshes. Dentro de esta carpeta usualmente sólo habrá un archivo .dae pero en piezas más complicadas ocasionalmente puede haber más de uno. Para saber cuál se debe modificar basta con ir a model.sdf y ver el uri que referencia el tag <collision>. Si abrimos un dae con un editor de texto veremos que se tratan de archivos muy extensos, superando muchas veces las 100000 lineas. La mayor parte del archivo se concentra en <library_geometries>, que es además la parte que nos interesa modificar. Aquí se encuentran los array que definen las formas tridimensionales de cada uno de los nodos del modelo, y cada nodo cuenta con 4 arrays: POSITION-array, Normal0-array, UV0-array y finalmente <triangles>. Si se quiere eliminar la colisión de un nodo, por ejemplo rocas, se deberán eliminar del .dae todos los arrays de ese nodo. Si haciendo esto el simulador deja de funcionar o hay algún error, se puede probar también el <effect> del nodo (que aparece al principio del archivo) y el nodo mismo <node> (que aparece al final del archivo).

Por último dejo aquí consejos basados en mi experiencia durante la práctica. Si lo que se quiere es cambiar la forma del modelo la tarea se vuelve más complicada, ya que hay que modificar los 4 arrays mencionados anteriormente. Para esto recomiendo utilizar Blender, un software opensource de creación 3D. Desde Blender se deben importar los archivos .dae, de vez en cuando puede ocurrir que un modelo no abra, en estos casos recomiendo abrir el archivo primero en Meshlab, guardarlo en documentos y he imporarlo a Blender desde ahí. Si se hace esto se debe agregar al principio del archivo generado por meshlab <unit meter="0.010000" name="centimeter"/> dentro de <assest>, de lo contrario la pieza tomará un tamaño enorme dentro de Blender. Una vez se tenga el modelo en el software se puede editar como uno desee, pero en mi experiencia cambios muy radicales pueden resultar en que el simulador deje de funcionar. Cuando se haya terminado de modificar el modelo este se exporta desde Blender como .dae al escritorio o documentos. Lo que se hace a continuación es reemplazar los arrays del .dae original por los del .dae generado con Blender. Esto nos dará la forma que creamos en el simulador, pero las texturas se arruinarán y que dependen del array UV0, el cual en Blender no tiene referencia a la textura utilizada en el repositorio. Sin embargo existe una forma de agregar las texturas a un modelo custom, para esto recomiendo descargar el jpeg que usan los modelos de cueva para sus texturas, el cual se encuentra referenciado en el .dae. Luego, uno puede encontrar tutoriales en línea que enseñan a agregar texturas a modelos desde Blender. Si se logra hacer esto correctamente el array UV0 del modelo exportado desde Blender estará ordenado según la misma imagen referenciada en el .dae lo que arreglará el problema de las texturas. Como último consejo decir que siempre es mejor modificar una pieza ya existente que crear una desde 0, ya que esto último rara vez termina funcionando, y que el proceso de modificar una pieza con Blender conlleva mucha prueba y error, ya que a veces uno sigue una serie de pasos que sirvieron con un modelo y termina no funcionando con otro.
    
![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/paso1.png)

![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/paso2.png)

![Alt Text](https://github.com/IgnacioDassori/SubT_model_pictures/blob/main/Im%C3%A1genes/paso3.png)
