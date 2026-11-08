
---

## Guía para conectar remotamente la jetson a un portatil


### 1. Obtener la dirección IP
Lo primero es conocer la IP de la Jetson. Ejecuta el siguiente comando en la terminal local de la **Jetson**:
```bash
hostname -I
```

### 2. Conexión vía SSH
Conecta el router y la Jetson (Ethernet o WiFi). Desde la terminal de tu **PC** o VS Code, ejecuta:
```bash
ssh jetson@<TU_IP_AQUÍ>
```

### 3. Autenticación
Introduce la contraseña de la Jetson cuando se te solicite (es posible que debas escribir `yes` para confirmar la autenticidad del host). Si la conexión es exitosa, verás el mensaje de bienvenida de **Ubuntu 22.04 LTS**.

### 4. Acceso al Contenedor Docker
Para trabajar dentro del entorno configurado, identifica el contenedor y accede a él:

1. **Listar contenedores activos:**
   ```bash
   docker ps
   ```
   *(Normalmente el nombre será `heuristic_johnson`)*.

2. **Entrar al contenedor:**
   ```bash
   docker exec -it heuristic_johnson bash
   ```
   > **Nota:** Si aparece el mensaje `groups: cannot find name for group ID 992`, ignóralo. Sabrás que estás dentro cuando el prompt cambie a `mirena@jetson-desktop:~$`.

### 5. Localización de paquetes (Source)
Una vez dentro del contenedor, muévete al workspace y carga las variables de entorno:
```bash
cd mirena_ws
source install/setup.bash
```

### 6. Sincronización de ROS_DOMAIN_ID
Para que los dispositivos se comuniquen entre sí, deben compartir el mismo ID de dominio.
Ejecuta este comando **tanto en la terminal SSH como en la terminal del contenedor** en VS Code:

```bash
export ROS_DOMAIN_ID=42
```

---

### 7. Ejecución y Visualización
Ya puedes lanzar tus procesos y visualizarlos de forma remota:

* **En la terminal SSH (Ejecutar proceso):**
    ```bash
    ros2 launch ./launch/launchfile.py
    ```

* **En VS Code / Contenedor (Visualizar):**
    ```bash
    rviz2
    ```