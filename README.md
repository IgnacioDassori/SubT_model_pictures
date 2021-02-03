# Tutorial Room & Pillar
El objetivo de este documento es instruir al lector en cómo utilizar y replicar los resultados más relevantes obtenidos durante la práctica en simulación de ambientes Room and Pillar. Este tutorial se divide en dos partes: la primera consiste en una explicación de cómo correr la simulación con el modelo final logrado por el practicante mientras que la segunda es una guía que explica el proceso de crear un ambiente propio. 
## 1. Cómo correr la simulación
Como se mencionó anteriormente, esta sección explicará cómo se puede correr el ambiente de simulación logrado al final de la práctica, que en este caso corresponde al modelo de una mina Room and Pillar. Para comenzar lo primero que necesitamos es tener los archivos de los resultados.
### 1.1 Obtener los archivos:
Todos los archivos necesarios para poder correr la simulación se encuentran en el siguiente Gitlab https://gitlab.com/finostro/3d-slam/-/tree/master/amtc-subt-testbed/amtc-packages, de donde se deben descargar todos los contenidos de la carpeta Modelo_Room&Pillar. Lo primero es una carpeta llamada models, esta contiene los archivos de todos los modelos que fueron modificados de alguna u otra manera, los cuales tendremos que reemplazar una vez creado un container nuevo. Lo segundo es un archivo sdf que corresponde al mundo, es decir el modelo de la mina por el que se moverá el vehículo. En este archivo se indica la posición tridimensional de las distintas piezas utilizadas y tiene un nombre particular, esto se debe a que en vez de crear un mundo nuevo reemplazaremos la información de uno existente. Correr la simulación desde un mundo completamente custom puede llevar a ciertos problemas que podemos evitar fácilmente aplicando este truco. Por último tenemos un pequeño script que nos ayuda a reemplazar los archivos, pero de todos modos se explicará como reemplazar modelos y mundos manualmente.
### 1.2 Iniciar un contenedor:
En este tutorial se asume que el usuario ya ha instalado Docker. Si este no es el caso seguir el siguiente tutorial: https://github.com/osrf/subt/wiki/Docker%20Install. Ocuparemos una versión modificada de los paquetes de Darpa Subterranean Challange que contiene el nuevo vehículo y configuraciones de sensores, entre otras cosas. Para esto se debe descargar la carpeta amtc-subt-testbed del siguiente git: https://gitlab.com/finostro/3d-slam. En este caso se realizó el pull desde la carpeta personal, ya que esto nos da un acceso más directo a amtc-subt-testbed.

Luego para iniciar un contenedor basta correr las siguientes líneas de código:

    cd amtc-subt-testbed/
    bash script/run
    catkin_make install
    bash script/build

### 1.3 Correr el script:
Antes de correr el script debemos asegurarnos de que la carpeta que contiene los archivos se encuentre en el directorio de preferencia (en este caso Downloads) y modificar algunos aspectos del script
