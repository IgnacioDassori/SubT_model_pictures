# Tutorial Room & Pillar
El objetivo de este documento es instruir al lector en cómo utilizar y replicar los resultados más relevantes obtenidos durante la práctica en simulación de ambientes Room and Pillar. Este tutorial se divide en dos partes: la primera consiste en una explicación de cómo correr la simulación con el modelo final logrado por el practicante mientras que la segunda es una guía que explica el proceso de crear un ambiente propio. 
## 1. Cómo correr la simulación
Como se mencionó anteriormente, esta sección explicará cómo se puede correr el ambiente de simulación logrado al final de la práctica, que en este caso corresponde al modelo de una mina Room and Pillar. Para comenzar lo primero que necesitamos es tener los archivos de los resultados.
### 1.1 Obtener los archivos:
Todos los archivos necesarios para poder correr la simulación se encuentran en el siguiente Gitlab https://gitlab.com/finostro/3d-slam/-/tree/master/amtc-subt-testbed/amtc-packages, de donde se deben descargar todos los contenidos de la carpeta Modelo_Room&Pillar. Lo primero es una carpeta llamada models, esta contiene los archivos de todos los modelos que fueron modificados de alguna u otra manera, los cuales tendremos que reemplazar una vez creado un container nuevo. Lo segundo es un archivo sdf que corresponde al mundo, es decir el modelo de la mina por el que se moverá el vehículo. En este archivo se indica la posición tridimensional de las distintas piezas utilizadas y tiene un nombre particular, esto se debe a que en vez de crear un mundo nuevo reemplazaremos la información de uno existente. Correr la simulación desde un mundo completamente custom puede llevar a ciertos problemas que podemos evitar fácilmente aplicando este truco. Por último tenemos un pequeño script que nos ayuda a reemplazar los archivos, pero de todos modos se explicará como reemplazar modelos y mundos manualmente en la segunda parte del tutorial.
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
host_path="/home/idassori/Downlaods/Modelo_Room&Pillar"
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
Lo primero que se debe hacer es asegurar que el path del host sea correcto, ya que la carpeta Modelo_Room&Pillar puede encontrarse en otro directorio. 
