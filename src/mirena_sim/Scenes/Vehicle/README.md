# VehicleDynamics — Modelo de dinámica vehicular de MirenaSim

**Ruta:** `src/mirena_sim/Scenes/Vehicle/VehicleDynamics/`  
**Vehículo de referencia:** Tecnun Formula Student TeR26

`VehicleDynamics` es el modelo físico modular que utiliza MirenaSim para simular el comportamiento del coche. Aquí se calculan el movimiento del chasis, las cargas en las ruedas, la suspensión, los neumáticos, la dirección, el tren motriz, los frenos y la aerodinámica.

El modelo modular convive con el sistema legacy de `VehicleBody3D` que se utilizaba en `vehicleMotion.gd`. Se puede cambiar entre ambos durante la simulación con **F6**. Es importante que solo uno tenga el control de las fuerzas en cada momento: si los dos actuasen a la vez, se contarían fuerzas por duplicado.

La idea de esta carpeta es tener la física separada de la escena y de ROS. El modelo calcula fuerzas y momentos; `GodotPhysicsAdapter` se encarga de aplicarlos al `RigidBody3D`.

### Qué incluye

- Chasis rígido y dinámica del vehículo.
- Cuatro ruedas, con su cinemática y dinámica de giro.
- Contacto con el suelo mediante plano analítico o raycast.
- Suspensión, muelles, amortiguadores y barras estabilizadoras.
- Dirección, Ackermann, toe y bump steer.
- Neumáticos MF6.2, TeR Grey-Box + PINN y baseline lineal.
- Tren motriz, reparto de par y control de tracción.
- Frenos y EBS.
- Drag y carga aerodinámica.
- Integración con Godot, ROS y herramientas de telemetría y comparación.

El objetivo final es que el coche virtual responda como el TeR26 real. Para medir cuánto nos acercamos, comparamos las señales de simulación con telemetría tomada en pista. La correlación es una parte importante de esa comparación, aunque no basta por sí sola para dar el modelo por validado.

> **Nota sobre el estado del documento:** describe la implementación según la revisión de código disponible. No se ha ejecutado Godot ni la suite de tests al preparar esta versión. Los porcentajes de correlación que aparecen más adelante son resultados históricos comunicados, no una medición repetida ahora.

---

## 2. Principios de arquitectura

| Principio | Aplicación |
|---|---|
| Separación por dominio físico | Cada subsistema tiene una responsabilidad y unas entradas/salidas definidas. |
| Núcleo de cálculo desacoplado | `Core`, `Chassis`, `Contact`, `Suspension`, `Steering`, `Wheels`, `Tires`, `Powertrain`, `Brakes` y `Aerodynamics` contienen la mayor parte de la lógica física y no deberían depender de nodos de escena ni de ROS. |
| Adaptadores en los límites | La interacción con Godot y ROS se concentra en `Integration` y `Telemetry`. |
| Configuración frente a estado | Los parámetros se agrupan en `Resource`; los estados dinámicos se mantienen en objetos `RefCounted` como `VehicleState`, `WheelState`, `SuspensionState`, `TireState` y `BrakeState`. |
| Propiedad única de fuerzas | `GodotPhysicsAdapter` es el único punto autorizado para aplicar fuerzas y pares al cuerpo rígido. |
| Unidades SI | Las magnitudes internas se expresan en unidades SI; las conversiones de presentación se realizan en los límites de la interfaz. |
| Fallos explícitos | Se registran flags de fallo para estados no finitos, configuración inválida, saturaciones y pérdida de contacto. |

Que el código esté separado por módulos no significa que toda la física esté terminada. Hay partes bastante desarrolladas y otras que siguen siendo aproximaciones; las principales están recogidas en la sección de problemas pendientes.

---

## 3. Estructura del directorio

### Raíz

| Archivo | Responsabilidad |
|---|---|
| `VehicleDynamicsController.gd` | Nodo de entrada. Posee el modelo, los adaptadores y el logger; selecciona el modo de integración y expone `step_from_host()`. |
| `VehicleDynamicsModel.gd` | Orquestador del modelo. Ejecuta `step(dt)` y los subpasos físicos mediante `_step_single()`. |
| `VehicleDynamicsConfig.gd` | `Resource` compuesto que agrupa parámetros del vehículo y configuraciones de neumáticos, suspensión, dirección, tren motriz, frenos y aerodinámica. |

### `Core/` — tipos, coordenadas, unidades y estado

- `VehicleCoordinates.gd`: transformaciones entre coordenadas Godot y el sistema del vehículo basado en ISO 8855; también contiene conversiones de rotaciones y datos de odometría.
- `VehicleUnits.gd`: conversiones de unidades y comprobaciones de finitud.
- `VehicleParameters.gd`: masa, centro de gravedad, inercia, geometría, vías, alturas y propiedades rotacionales de rueda.
- `VehicleState.gd`: estado agregado del vehículo, fuerzas, estados de rueda, aerodinámica, tiempo de simulación y flags de fallo.
- `VehicleDynamicsTypes.gd`: enumeraciones de esquinas, ejes, modos de integración, arquitectura de transmisión, tipo de neumático y flags de diagnóstico.

### `Chassis/` — dinámica del cuerpo rígido

- `ChassisModel.gd`: actualiza la cinemática del cuerpo y ensambla fuerzas y momentos procedentes de ruedas y aerodinámica. Calcula aceleraciones para el estado del vehículo y señales tipo IMU.
- `RigidBodyDynamics.gd`: proyección de fuerzas y momentos y paso de integración standalone, principalmente para modo bench.
- `VehicleInertia.gd`: propiedades de inercia, cargas estáticas por rueda y términos giroscópicos.

### `Contact/` — contacto con el suelo

- `ContactQuery.gd`: consulta de suelo mediante plano analítico o `PhysicsDirectSpaceState3D`.
- `WheelContact.gd`: consulta desde el cubo, compresión de suspensión, velocidad de compresión y radio efectivo.
- `ContactState.gd`: normal, punto de contacto y escala de fricción de superficie.

### `Suspension/` — suspensión y cargas normales

- `SuspensionModel.gd`: evaluación de las cuatro esquinas, acoplamiento de barra estabilizadora y limitación de la carga normal para evitar tracción del suelo (`Fz ≥ 0`).
- `SpringModel.gd`: precarga, rigidez equivalente en rueda (`k / MR²`) y bump stop.
- `DamperModel.gd`: amortiguación bump/rebound con comportamiento digresivo.
- `AntiRollBarModel.gd`: contribución de la barra estabilizadora con signos opuestos a izquierda y derecha.
- `SuspensionConfiguration.gd` y `SuspensionState.gd`: parámetros y estado de suspensión.

### `Steering/` — dirección

- `SteeringModel.gd`: estado de cremallera, límite opcional de velocidad y offset de cero.
- `SteeringGeometry.gd`: Ackermann, toe estático y contribuciones de bump steer.
- `SteeringConfiguration.gd`: límites angulares, velocidad máxima y parámetros geométricos.

### `Wheels/` — cinemática y dinámica rotacional

- `WheelKinematics.gd`: velocidad del cubo, slip ratio (`κ`) y slip angle (`α`), con regularización y transición suave a baja velocidad.
- `WheelRotationalDynamics.gd`: dinámica de giro de la rueda, con el balance de par de tracción, fuerza longitudinal, freno e inercia rotacional.
- `WheelAssembly.gd`: coordina cinemática, entradas del neumático, evaluación de fuerzas e integración rotacional por rueda.
- `WheelState.gd`: estado dinámico de cada conjunto.

### `Tires/` — neumáticos

- `TireModel.gd`: interfaz de evaluación por rueda y gestión de relajación transitoria longitudinal y lateral. Gestiona salidas no finitas mediante flags de fallo.
- `TireModelRegistry.gd`: factoría de implementaciones a partir de `TireModelType`.
- `TireInputs.gd`, `TireForces.gd`, `TireState.gd` y `TireConfiguration.gd`: contrato de entrada, salida, estado y configuración.
- `Models/PacejkaMF62TireModel.gd`: implementación MF6.2 para fuerza lateral y momento autoalineante; la fuerza longitudinal se completa mediante un modelo compañero de elipse de fricción, no mediante una calibración longitudinal completa.
- `Models/TeRTireGreyBoxModel.gd`: extensión Grey-Box que añade una red residual PINN al modelo de base. La red utiliza siete entradas relacionadas con `α`, su derivada, camber, carga normal, presión y variables térmicas; predice correcciones residuales de `Fy` y `Mz`.
- `Models/TireModelInterface.gd`: interfaz base y baseline lineal/de pruebas.
- `TeRTireWeightsData.gd`: almacenamiento de coeficientes y pesos embebidos utilizados por las implementaciones correspondientes.

En la interfaz de simulación se contemplan tres alternativas: **TeR Grey-Box + PINN**, **Pure Pacejka MF6.2** y **Linear baseline**. La baseline lineal sirve como referencia simplificada; no debe interpretarse como un neumático calibrado.

### `Powertrain/` — tren motriz

- `PowertrainModel.gd`: calcula el par solicitado, aplica el escalado de control de tracción (`tc_scale`) y una atenuación heurística cuando la carga normal es baja.
- `MotorModel.gd`: envolvente de par y potencia, límite de régimen y política de marcha atrás.
- `DrivetrainModel.gd`: reparto de par según arquitectura RWD/AWD y diferencial viscoso trasero.
- `PowertrainConfiguration.gd`: parámetros del conjunto, incluidos los parámetros del diferencial.

### `Brakes/` — sistema de frenos

- `BrakeSystem.gd`: presupuesto de par de frenado, reparto delantero/trasero y contribución de EBS por rueda.
- `BrakeConfiguration.gd` y `BrakeState.gd`: configuración y estado del freno.

### `Aerodynamics/` — aerodinámica

- `AerodynamicModel.gd`: calcula presión dinámica, drag y downforce, reparto por ejes y momento de cabeceo.
- `AerodynamicConfiguration.gd`: densidad, área frontal, coeficientes de drag/lift y reparto aerodinámico.

**Limitación conocida:** existen parámetros de configuración para efecto suelo (`enable_ground_effect`, `k_ground_*`, `h_aero_ref_m`) que, según la revisión disponible, todavía no se aplican en `AerodynamicModel.gd`.

### `Integration/` — adaptadores Godot y ROS

- `GodotPhysicsAdapter.gd`: sincroniza estado con el cuerpo rígido, configura masa/centro de gravedad/inercia y aplica las fuerzas y momentos calculados. Es el único punto de escritura física sobre el `RigidBody3D`.
- `VehicleSceneAdapter.gd`: enlaza las cuatro ruedas visuales, actualiza dirección, giro y compresión, y gestiona efectos visuales.
- `ROSVehicleAdapter.gd`: contiene codificación y publicación de estado del vehículo y velocidades de rueda, con decimación, filtrado y cuantización.

**Limitación conocida:** la publicación activa del host todavía pasa por funciones de `vehicleMotion.gd`; `ROSVehicleAdapter.publish_step()` no está conectado al recorrido principal descrito en la revisión. No migrar la publicación sin eliminar simultáneamente el camino duplicado.

### `Telemetry/` — diagnóstico y validación

- `DynamicsDiagnostics.gd`: snapshots y contadores de fallos.
- `DynamicsLogger.gd`: buffer y exportación CSV. El formato normal contiene 30 columnas; el modo expandido añade más estado.
- `TelemetryChannel.gd` y `TelemetryRecording.gd`: series temporales con procedencia: medida, simulada, derivada o no disponible.
- `TelemetryImporter.gd`: importación de bundles de comparación y CSV.
- `CorrelationEngine.gd`: métricas de error y correlación, incluyendo RMSE, MAE, bias, Pearson `r`, NRMSE y lag, además de la agregación utilizada por el HUD.

### `Config/` — presets

| Recurso | Uso / observación |
|---|---|
| `vehicle_default.tres` | Preset general. En la revisión disponible: radio descargado de rueda de 0,20 m e `Izz = 120 kg·m²`. |
| `tire_default.tres`, `suspension_default.tres`, `powertrain_default.tres` | Subrecursos del preset general. |
| `vehicle_ter26_calibrated.tres` y subrecursos | Preset de referencia TeR26. En la revisión disponible: radio de 0,2032 m e `Izz = 135 kg·m²`. |
| `vehicle_ter26_opt.tres` y subrecursos | Variante optimizada, con parámetros específicos de relajación y neumático. |
| `baselines/modular_baseline/` | Referencia congelada para comparaciones antes/después. No modificar durante una campaña de validación. |

Que los presets tengan valores distintos no significa necesariamente que haya un error. Lo importante es saber a qué configuración pertenece cada uno y de dónde sale el valor. No conviene igualarlos sin comprobar antes medidas y cálculos.

### `Tests/` — verificación

| Prueba / script | Cobertura prevista |
|---|---|
| `test_vehicle_dynamics.gd` | Agregador de las pruebas del modelo e integración. |
| `test_tire_model.gd` | Signos, saturación, relajación, valores no finitos y paridad con referencias del modelo de neumático. |
| `test_wheel_kinematics.gd` | Transformaciones, slip, singularidad a baja velocidad y unidades. |
| `test_suspension_model.gd` | Cargas estáticas, bump stops, amortiguación, ARB y descarga de rueda. |
| `test_powertrain.gd` | Par, potencia, reparto, marcha atrás y frenos/bloqueo. |
| `test_phase_iv_model_integrity.gd` | Integridad de configuración, alias, simetría y radios. |
| `test_low_speed_stability.gd` | Estabilidad a baja velocidad y comportamiento en parado. |
| `run_validation_maneuvers.gd` | Maniobras controladas: equilibrio, aceleración, frenado, escalón, rampa, skidpad, slalom, etc. |
| `run_modular_replay.gd` | Replay en lazo abierto de las entradas registradas y generación de CSV de simulación. |
| `test_qol_01_verification.gd` | Conmutación legacy/modular, interfaz y transferencia de estado. |

Que el script exista no significa que se haya ejecutado con la última versión. Cuando se pase la batería, conviene guardar también el commit, el preset, la configuración de Godot y la salida de cada prueba.

### `UI/` — monitorización y ajuste

- `VehicleDynamicsDashboardWindow.gd`: ventana de monitorización (F7), comparación de telemetría y ajuste en caliente de parámetros seleccionados.
- `SignalPlotCanvas.gd`: trazas temporales y gráficos XY con zoom y desplazamiento.

### Puntos de integración externos

- `Scenes/Vehicle/vehicleMotion.gd` (`MirenaCar`): propietario del ciclo desde la escena, selección del motor físico, transferencia de autoridad, estado y publicación del host.
- `Scenes/Sim/UI/`: HUD, lanzador, gráficas flotantes, registro de señales y tema de interfaz.

---

## 4. Flujo de cálculo por tick

En modo modular, el host (`MirenaCar`) controla el paso:

1. `MirenaCar._physics_process()` llama a `_apply_vehicle_physics()`.
2. `dynamics_controller.step_from_host(delta, gas, steer, ebs, origin)` recibe el paso y las entradas.
3. `GodotPhysicsAdapter.sync_state_from_body()` sincroniza pose y velocidades del cuerpo rígido.
4. `VehicleDynamicsModel.step(dt)` ejecuta `physics_substeps` iteraciones de `_step_single()`.
5. En cada subpaso se evalúan, de forma ordenada:
   1. cinemática del chasis;
   2. contacto de las cuatro ruedas;
   3. aerodinámica;
   4. suspensión y barra estabilizadora;
   5. dirección y ángulos por rueda;
   6. cinemática de rueda y cálculo de `κ` y `α`;
   7. entradas al neumático y cálculo de `Fx`, `Fy`, `Mz` y demás salidas;
   8. par motriz y par de frenado;
   9. ensamblaje de fuerzas y momentos en coordenadas del vehículo y del mundo Godot;
   10. integración de la velocidad angular de cada rueda;
   11. integración standalone, únicamente cuando corresponde al modo bench.
6. `GodotPhysicsAdapter.apply_forces_to_body()` aplica la fuerza central y el par al `RigidBody3D`.
7. El adaptador de escena actualiza las ruedas visuales y el logger registra la muestra.

El orden importa. Primero se calculan las cargas de cada rueda; con ellas y con los slips se evalúa el neumático. Después, las fuerzas del neumático se utilizan tanto para mover el chasis como para actualizar el giro de las ruedas.

### Reset

`MirenaCar.reset_position()` llama a `dynamics_controller.reset_vehicle(origin)`, que reinicia el modelo y los elementos asociados de encoder y logger. Para las pruebas de correlación hay que distinguir este reset normal de los resets artificiales dentro de un replay: reiniciar el estado cada pocas muestras puede esconder el error que se va acumulando.

---

## 5. Convenciones físicas y de coordenadas

| Magnitud / convención | Definición |
|---|---|
| Godot | `+Y` arriba, `-Z` hacia delante y `-X` hacia la izquierda. |
| Vehículo | `+X` hacia delante, `+Y` hacia la izquierda y `+Z` hacia arriba. |
| Transformación vectorial | `vehicle = (-z, -x, y)` según `VehicleCoordinates`. |
| Índices de rueda | `FL = 0`, `FR = 1`, `RL = 2`, `RR = 3`. |
| Slip ratio | `κ > 0` representa tracción y debe producir `Fx > 0` bajo la convención adoptada. |
| Slip angle | La convención implementada establece `α > 0` con `Fy < 0` y `Mz > 0`; debe mantenerse coherente con los datos de neumático y sensores. |
| Dirección | El comando se interpreta como ángulo de cremallera en radianes; positivo significa giro a la izquierda. |
| Unidades | SI internamente; ángulos en radianes salvo presentación explícita en grados. |

Al comparar con telemetría hay que revisar la orientación de los sensores, los signos, los marcos de referencia y cómo se ha tratado la gravedad. Una correlación baja no siempre significa que la física esté mal: también puede venir de una transformación o de una convención de signos incorrecta.

---

## 6. Configuración y selección de modelos

1. **Preset:** asignar el recurso `.tres` correspondiente en `VehicleDynamicsController.config` o en el nodo `MirenaCar`.
2. **Validación:** `VehicleDynamicsConfig.is_valid()` comprueba coherencia de la configuración. Una configuración inválida debe impedir avanzar el modelo y quedar registrada como fallo.
3. **Modelo de neumático:** se puede seleccionar mediante `TireModelType`, la API de runtime o los selectores de interfaz.
4. **Ajuste en caliente:** la pestaña de *Live Physics Tuning* modifica parámetros de la configuración activa. Estos cambios son de sesión y no deben considerarse persistentes hasta guardarlos en el preset correspondiente.
5. **Paso temporal:** `physics_substeps` controla el subpaso interno. Debe documentarse junto a la frecuencia de física de Godot; no se debe cambiar el paso durante una comparación sin repetir la validación.

### Modelos de neumático disponibles

| Modelo | Función | Estado de calibración conocido |
|---|---|---|
| TeR Grey-Box + PINN | MF6.2 de base más corrección residual neuronal para fuerzas laterales y momento autoalineante. | El ajuste lateral/transitorio es el foco de desarrollo. La salida longitudinal no equivale a un MF6.2 longitudinal calibrado. |
| Pure Pacejka MF6.2 | Modelo paramétrico de neumático para `Fy` y `Mz`, con companion longitudinal. | No considerar `Fx` calibrado mientras dependa de la elipse de fricción sin ajuste longitudinal independiente. |
| Linear baseline | Referencia simplificada para contrastar arquitectura y estabilidad. | Baseline, no modelo predictivo final. |

La selección entre modelos debe ejecutarse con idénticas entradas, cargas, condiciones iniciales, filtros y ventanas temporales. Comparar cada modelo con distintas condiciones no permite atribuir la diferencia al neumático.

---

## 7. Comparación con el TeR26 real

La forma de saber si el modelo se parece al coche no es solo mirar si la simulación funciona o si el comportamiento parece razonable. Hay que reproducir las mismas entradas que tuvo el coche en pista y comparar las señales medidas con las que genera MirenaSim.

### 7.1 Qué resultados tenemos

La última referencia histórica que tenemos comunicada es esta:

| Comparación | Correlación reportada |
|---|---:|
| Señales filtradas a 5 Hz | **72,9 %** |
| Señales sin ese filtrado | **60,1 %** |

Son cifras útiles para seguir la evolución, pero todavía no las tomaría como el resultado definitivo del modelo actual. No tenemos en esta referencia el desglose completo por señal y maniobra, el número exacto de muestras ni todos los detalles de cómo se calculó el valor global. Tampoco se ha repetido aquí el cálculo sobre el estado actual del repositorio.

En su momento apareció también un resultado cercano al **92 %**. No lo usamos como referencia porque se había obtenido con resets frecuentes del estado de referencia (aproximadamente cada 0,5 s o 100 muestras). Al reiniciar el estado tan a menudo se evita que el error se acumule durante el replay, así que el número puede dar una impresión demasiado buena del modelo.

La corrección fue pasar a una comparación en lazo abierto y continua, con ventanas de 2 s (400 muestras), sin resets artificiales dentro de la ventana. La cifra de 72,9 % debe recalcularse con ese procedimiento y con los datos y configuración actuales antes de sustituirla por un nuevo resultado oficial.

### 7.2 Cómo interpretar el porcentaje

La correlación indica si dos señales siguen una evolución parecida, pero no dice por sí sola si sus valores son correctos. Por ejemplo, la simulación puede tener una forma muy parecida a la señal real y aun así presentar un offset, una amplitud distinta o un retardo.

Por eso no conviene mirar únicamente Pearson `r`. Para cada señal también hay que revisar:

- **RMSE y MAE:** cuánto se separa la simulación de la medida.
- **Bias:** si el modelo tiende a quedarse por encima o por debajo.
- **NRMSE:** error normalizado para poder comparar señales de distinta escala.
- **Lag:** desfase temporal entre ambas señales.
- **Trazas superpuestas:** para ver si los errores aparecen en el transitorio, en régimen o durante los cambios de maniobra.

El filtrado a 5 Hz puede ayudar a comparar la tendencia y reducir ruido, pero también puede suavizar transitorios. Por eso interesa guardar y presentar siempre los resultados filtrados y sin filtrar.

### 7.3 Lo que más interesa mejorar

La **aceleración lateral (`ay`)** es uno de los puntos que más interesa mejorar. No depende solo del neumático: también entran en juego las cargas normales, la transferencia de carga, la suspensión, la geometría de dirección, la inercia de guiñada, la aerodinámica y la propia sincronización de sensores.

Para entender de dónde viene la diferencia, interesa mirar `ay` junto con:

- velocidad angular de guiñada (`wz` o yaw rate);
- velocidad longitudinal (`vx`);
- ángulo de dirección y entradas de acelerador y freno;
- velocidades de rueda;
- cargas normales por rueda, cuando estén disponibles.

Antes de tocar parámetros del neumático conviene comprobar que las señales están en el mismo sistema de coordenadas, tienen el mismo signo y están bien alineadas en el tiempo. De lo contrario, podríamos acabar ajustando el neumático para compensar un error que está en otra parte del modelo o del pipeline de datos.

### 7.4 Cómo hacer una comparación que podamos repetir

Para que los resultados tengan sentido de una ejecución a otra, el replay debería seguir siempre el mismo procedimiento:

1. Usar el mismo ensayo y las entradas registradas del coche real.
2. Reproducir las entradas en lazo abierto, sin corregir la simulación con el estado medido durante la ventana.
3. Mantener el replay continuo. Nada de reiniciar el estado cada pocas muestras para mejorar artificialmente el resultado.
4. Alinear timestamps y frecuencias de muestreo. Si se compensa un retardo, dejar registrado cuánto y por qué.
5. Aplicar el mismo criterio de filtrado y remuestreo a todas las comparaciones. Guardar el resultado con y sin filtro.
6. Revisar marcos de referencia, signos y unidades, especialmente en las señales IMU.
7. Separar los datos usados para calibrar de los que se usan para validar. Si ajustamos parámetros y medimos el resultado sobre los mismos tramos, es fácil sobreajustar.
8. Sacar métricas por señal y por maniobra, además del valor global.
9. Guardar junto a los resultados el commit, preset, modelo de neumático, dataset, versión de Godot y parámetros del replay.

Una mejora no debería darse por buena solo porque suba el porcentaje global. También tiene que reducir el error donde importa, no introducir retardos o sesgos nuevos y mantener un comportamiento razonable en maniobras que no se hayan usado para ajustar el modelo.

## 8. Lo que falta por resolver

Estos son los puntos que siguen abiertos según la revisión disponible. Algunos afectan directamente a la correlación; otros son simplificaciones del modelo o tareas de integración que conviene cerrar.

### Lo que más afecta a la correlación

1. **Aceleración lateral (`ay`).** Es una de las señales que más interesa mejorar. Primero hay que descartar problemas de signo, coordenadas, gravedad y sincronización. Después podremos separar cuánto viene del neumático, las cargas normales, la suspensión, la dirección, la inercia o la aerodinámica.

2. **Repetir la correlación con el replay continuo.** El 72,9 % filtrado a 5 Hz sigue siendo nuestra referencia histórica, pero hay que recalcularlo con el procedimiento corregido y publicar el desglose por canal y maniobra. El resultado cercano al 92 % obtenido con resets frecuentes no debe usarse como KPI.

3. **Calibrar `Fx` de verdad.** En Pacejka y en el Grey-Box, la fuerza longitudinal se completa con un modelo compañero basado en la elipse de fricción. Eso no es lo mismo que tener un modelo longitudinal y de combined-slip calibrado con datos del neumático. La diferencia afecta a aceleración, frenada y maniobras donde el neumático trabaja a la vez en longitudinal y lateral.

4. **Comprobar la transferencia de carga.** Hay que contrastar cómo evolucionan las cuatro cargas `Fz` en aceleración, frenada y curva. Si las cargas no son correctas, el neumático recibe unas condiciones equivocadas y el error termina apareciendo en `ay`, `wz` y el comportamiento global.

5. **Reproducir bien el frenado.** El replay tiene que conservar la señal de freno y, cuando corresponda, el mando EBS por separado del acelerador. Sin esas entradas no se pueden validar correctamente las fases de frenada ni las transiciones entre acelerar, frenar y girar.

### Partes del modelo que siguen siendo aproximadas

6. **Respuesta transitoria y temperatura del neumático.** Falta contrastar la relajación longitudinal y lateral, y la sensibilidad a presión, camber y temperatura, con datos transitorios. Que la red PINN reproduzca bien sus datos de entrenamiento no demuestra por sí solo que funcione igual de bien en pista; hace falta validarla con datos que no se hayan usado para entrenarla.

7. **Cinemática de suspensión.** Hay que comprobar las curvas de motion ratio, camber, toe, bump steer y centros de balanceo a lo largo de todo el recorrido. Un muelle y un amortiguador equivalentes no sustituyen una cinemática completa si la geometría real cambia de forma importante con el movimiento.

8. **Control de tracción.** `tc_scale` y la reducción de par cuando la carga normal es baja están metidos directamente en `PowertrainModel`. Son heurísticas y pueden cambiar la aceleración y la salida de curva. Falta contrastarlas con la estrategia real de la ECU o separarlas detrás de una interfaz de control.

9. **Diferencial y reparto de par.** El reparto por arquitectura y el diferencial viscoso trasero son simplificaciones. Hay que comprobar cuánto se acercan a la transmisión real, especialmente cuando las ruedas tienen velocidades o cargas distintas.

10. **Aerodinámica y efecto suelo.** El modelo utiliza coeficientes globales y un reparto por ejes. En la configuración existen `enable_ground_effect`, `k_ground_*` y `h_aero_ref_m`, pero esos parámetros todavía no se aplican en `AerodynamicModel.gd`. Habrá que implementar el efecto suelo o quitar los parámetros para no dar a entender que ya está modelado. También falta contrastar drag, downforce y balance aero con datos o cálculos fiables.

11. **Contacto con el suelo.** Ahora se usa un plano analítico o raycast. Es suficiente para muchos escenarios, pero no representa una huella de contacto deformable ni las irregularidades de la carretera. Habrá que decidir si hace falta algo más detallado para las maniobras que queremos validar.

### Tareas de integración y mantenimiento

12. **Rutas absolutas.** Hay referencias a `/home/alexr/eRacing/mirena_ws-SIM/imu_compare` en la UI, el importador y algunos scripts. Funcionan en esa máquina, pero complican mover el proyecto a otro equipo. Lo suyo es usar rutas relativas al proyecto, `user://` o una ruta configurable.

13. **Diferencias entre presets.** `vehicle_default.tres` usa un radio de rueda de 0,20 m, mientras que la configuración del neumático usa 0,2032 m. También aparecen valores de `Izz` de 120 y 135 kg·m² en presets distintos. Puede estar justificado, pero hay que dejar claro a qué configuración corresponde cada valor y de dónde sale.

14. **Selector de neumático.** El mapeo entre el índice que muestra la UI y `TireModelType` está repetido en varios sitios. Conviene centralizarlo para que añadir un modelo no obligue a modificar varios `match` independientes.

15. **Publicación ROS.** `ROSVehicleAdapter.publish_step()` tiene su propia lógica, pero el recorrido principal sigue publicando desde `vehicleMotion.gd`, según la revisión disponible. Cuando se migre, habrá que dejar una sola ruta activa para no publicar dos veces los mismos datos.

16. **Alias del diferencial.** `diff_preload_viscous_nm_s_rad` es un alias de `diff_viscous_damping_nm_s_per_rad`. Para configuraciones nuevas, usar el nombre canónico y mantener el alias solo por compatibilidad.

17. **Procedencia de los pesos del neumático.** El código identifica TeR-Tire como fuente de los coeficientes y pesos. Falta dejar una referencia fácil de comprobar al repositorio, commit, fichero de entrenamiento/exportación y hash de los pesos usados.

18. **Documentación y tests.** Algunos documentos de fases anteriores pueden haberse quedado desfasados respecto al código actual. Este README debería ser la referencia principal de la carpeta. Cuando cambien interfaces o parámetros importantes, hay que actualizarlo y volver a ejecutar los tests relacionados. La existencia de un test no significa que se haya ejecutado en cada cambio.

---

## 9. Sustitución y extensión de subsistemas

La regla es sencilla: cambiar la implementación por detrás de su interfaz y mantener un solo sitio encargado de cada cálculo físico. No hay que añadir un segundo cálculo de fuerza de rueda ni dejar activos a la vez el motor legacy y el modular.

### 9.1 Neumáticos

Contrato principal: `TireInputs` → `TireForces`.

Entradas relevantes: `slip_ratio_kappa`, `slip_angle_alpha_rad`, `normal_load_fz_n`, `camber_gamma_rad`, presión, temperatura y escala de fricción de superficie. Salidas relevantes: `fx_n`, `fy_n`, `mz_nm` y momento de resistencia a la rodadura cuando esté implementado.

Para añadir un neumático:

1. Implementar `TireModelInterface` y validar entradas/configuración.
2. Definir explícitamente el dominio de validez y la procedencia/calibración de cada salida.
3. Registrar el modelo en `VehicleDynamicsTypes.TireModelType` y `TireModelRegistry`.
4. Conectar su selección en la UI mediante un mapeo centralizado, no mediante índices independientes.
5. Añadir tests de `Fz ≤ 0`, signos de `κ` y `α`, saturación, continuidad, valores no finitos y respuesta combinada.
6. Compararlo con los otros modelos usando exactamente el mismo replay y conjunto de validación.

### 9.2 Suspensión

Los puntos principales son `SuspensionModel.evaluate_all_corners()` y las funciones de muelle, amortiguador y barra estabilizadora. Cualquier cambio debe conservar el tratamiento de descarga de rueda y verificar simetría izquierda/derecha, cargas estáticas, respuesta heave/roll y evolución de `Fz`.

### 9.3 Tren motriz

Los puntos principales son `PowertrainModel.evaluate_drive_torques()`, `MotorModel` y `DrivetrainModel`. Un cambio de curva de motor, reparto o diferencial debe indicar si conserva o elimina el control de tracción heurístico y la atenuación por carga normal.

### 9.4 Frenos

El punto de entrada es `BrakeSystem.evaluate_brake_torques()`. Un modelo hidráulico más completo puede añadir retardos y estados, pero debe mantener coherencia con el par que consume la dinámica rotacional de rueda y con la señal EBS.

### 9.5 Aerodinámica

El punto de entrada es `AerodynamicModel.evaluate_aerodynamics()`. Una ampliación de centro de presión variable o efecto suelo debe mantener el contrato de fuerzas/momentos y validar balance aero y sensibilidad a altura y actitud.

### 9.6 Dirección

El punto de entrada es `SteeringGeometry.compute_wheel_steering_angles()`. Cualquier cambio debe aclarar si la entrada representa volante, cremallera o ángulo de rueda, y mantener la convención de signo documentada.

### 9.7 Integración física

`GodotPhysicsAdapter` debe seguir siendo el único punto que aplica fuerzas y momentos al cuerpo. Los cambios en `WheelRotationalDynamics`, `VehicleState`, el orden FL/FR/RL/RR o la sincronización con el host requieren revisar todos los consumidores y repetir los tests de integración.

---

## 10. Verificación y ejecución

Estos comandos se ejecutan desde la raíz del repositorio. Aquí se usa `godot` como ejemplo; si el ejecutable no está en el `PATH`, sustituye ese nombre por la ruta de tu instalación.

### Suite agregada

```bash
godot --headless --path src/mirena_sim \
  -s res://Scenes/Vehicle/VehicleDynamics/Tests/test_vehicle_dynamics.gd
```

### Maniobras controladas

```bash
godot --headless --path src/mirena_sim \
  -s res://Scenes/Vehicle/VehicleDynamics/Tests/run_validation_maneuvers.gd
```

### Replay de entradas reales

```bash
godot --headless --path src/mirena_sim \
  -s res://Scenes/Vehicle/VehicleDynamics/Tests/run_modular_replay.gd -- \
  --config res://Scenes/Vehicle/VehicleDynamics/Config/vehicle_ter26_calibrated.tres \
  --out-prefix ter26_cal \
  --duration 281
```

El replay debe generar resultados identificables por configuración y dataset. Si los scripts esperan una carpeta de datos concreta, esa ruta debe pasarse de forma configurable; no debe depender de una ruta absoluta personal.

### Registro mínimo de una campaña de validación

Para cada ejecución se recomienda guardar:

- hash/commit del código;
- preset y hash de los recursos `.tres`;
- versión de Godot y frecuencia de física;
- modelo de neumático seleccionado;
- dataset y maniobras incluidas;
- filtros, remuestreo y sincronización aplicados;
- condiciones iniciales y duración de replay;
- resultados por señal: `r`, RMSE, MAE, bias, NRMSE y lag;
- resultado de la suite de tests y maniobras;
- cambios de parámetros realizados y su justificación.

---

## 11. Por dónde seguir

Yo seguiría este orden, porque evita empezar a retocar parámetros físicos cuando todavía puede haber problemas en los datos o en el propio pipeline:

1. Dejar cerrado el replay continuo y recalcular la correlación con sus métricas por señal y maniobra.
2. Investigar `ay`: coordenadas, signos, gravedad, sincronización y diferencia entre aceleración específica de la IMU y aceleración cinemática.
3. Comprobar las cargas normales y la transferencia de carga en aceleración, frenada y curva.
4. Volver al neumático: mejorar la respuesta transitoria de `Fy`/`Mz` y calibrar `Fx` y combined-slip con datos.
5. Validar las entradas de freno, el control de tracción, el diferencial y el reparto de par.
6. Contrastar la cinemática de suspensión y dirección con la geometría del coche.
7. Completar la aerodinámica y resolver qué hacemos con los parámetros de efecto suelo que ahora no se usan.
8. Limpiar las rutas absolutas, centralizar el selector de neumáticos y dejar una única ruta de publicación ROS.
9. Repetir las pruebas con maniobras que no se hayan utilizado durante la calibración.

---

## 12. En qué punto estamos

La base modular ya está montada y conectada a MirenaSim. Tenemos los principales bloques de dinámica, varios modelos de neumático, telemetría, herramientas para comparar con datos reales y tests para comprobar partes del sistema.

Lo que todavía falta es demostrar que todo eso reproduce bien al coche real. Ahora mismo los dos frentes más importantes son mejorar la respuesta lateral —especialmente `ay`— y cerrar la validación con un replay continuo que podamos repetir y comparar entre versiones. También queda trabajo en la fuerza longitudinal del neumático, la transferencia de carga, el frenado y algunas simplificaciones del drivetrain y la aerodinámica.

El **72,9 % a 5 Hz** sirve como punto de partida histórico, pero no lo presentaría como la correlación definitiva del modelo actual hasta volver a calcularlo con el procedimiento corregido.

