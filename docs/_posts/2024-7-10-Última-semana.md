---
title: "Últimas semanas"
categories:
  - Weblog
  - TFG
tags:
  - OMPL
  - Amazon Warehouse
  - Logistic robots
  - Ackermann robot
---

Por un lado, he implementado una solución de referencia basandome en una planificación basada en controles con el robot logístico Ackermann utilizando la clase app de OMPL como se había mencionado en posts anteriores. Los waypoints de la ruta solución encontrada tiene el siguiente formato: (x, y, yaw, v, w, d), siendo (v, w) la velocidad lineal y angular respectivamente, y d el tiempo de duración de los controles. Y la ejecución de la ruta se realiza recorriendo la secuencia (v, w, d), dicha implementación tiene el inconveniente de que no es reactivo y es inevitable la generación de errores y la acumulación de ellos. 

Por otro lado, he terminado de escribir la memoria.

