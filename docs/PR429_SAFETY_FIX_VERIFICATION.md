# PR #429 — Verificación de las correcciones de seguridad

## Estado

La PR permanece en **Draft**. No debe fusionarse ni probarse con el vehículo apoyado en el suelo hasta completar esta lista con las cuatro ruedas elevadas, el vehículo inmovilizado y una desconexión de potencia accesible.

## Fallos corregidos

### 1. Falso `RELAY OPEN` / DTC 16

La protección antigua comparaba la corriente CH0–CH3 con un umbral de carga de 1,5–4 A y podía declarar el relé abierto aunque las ruedas estuvieran girando. Esa conclusión es físicamente contradictoria: un relé totalmente abierto no puede alimentar motores que giran bajo PWM.

La corrección deja el diagnóstico de evidencias visible por CAN/HMI, pero no permite que una medida baja o inválida genere DTC 16. Con ruedas girando y corriente realmente cercana a cero, el resultado esperado es `CURRENT SENSE INVALID`, no `RELAY OPEN`.

### 2. CH5 `PRESENT NO SHUNT` estando el volante parado

Cero amperios y aproximadamente cero microvoltios son lecturas normales cuando el motor de dirección está parado. CH5 solo puede declarar `PRESENT NO SHUNT` o `POLARITY REVERSED` cuando TIM3 confirma PWM real en PA6 o PA7.

### 3. Fallo del centrado y volante frenado

Ante fallo de búsqueda del centro:

- PWM de dirección queda a cero;
- `EN_STEER` queda desactivado por `Steering_Neutralize()`;
- PC12 corta la alimentación de 12 V del puente de dirección;
- el sistema entra en `LIMP_HOME`, con 20 % de par y límite de 5 km/h;
- la tracción puede completar su secuencia de relés sin energizar el puente de dirección no calibrado.

### 4. Diagnóstico `POWER NOT READY` durante homing

El homing se ejecuta en STANDBY y controla PC12 localmente. Su diagnóstico utiliza ahora PC12 más el estado real de barrido, no el estado del secuenciador global de tracción.

## Preparación del registro DTC

El registro de errores es persistente y sobrevive a reinicios. Por tanto, un DTC 16 creado por una versión anterior puede continuar apareciendo como **entrada histórica** aunque ya no sea el fallo activo.

Antes de repetir la prueba:

1. anotar o exportar el registro actual;
2. borrar el historial mediante la función de diagnóstico existente que invoque `ErrorLog_Clear()`, cuando esté disponible en la interfaz de ingeniería;
3. si no se borra, comparar el timestamp y distinguir claramente `DTC activo` de `DTC histórico`.

El criterio de aceptación es que la nueva ejecución no cree otra entrada DTC 16 y que el error activo permanezca distinto de `SAFETY_ERROR_RELAY_OPEN` mientras las ruedas giren bajo PWM.

## Prueba A — Arranque sin detectar el tornillo central

1. Elevar e inmovilizar el vehículo.
2. Colocar un multímetro entre PC12/salida del relé de dirección y masa.
3. Arrancar sin presentar metal al sensor PB5.
4. Esperar al timeout o al segundo fin de recorrido detectado.

Resultado obligatorio:

- estado final `LIMP_HOME`;
- PC12 = 0 V después del aborto;
- PC4 `EN_STEER` = nivel bajo;
- PA6 y PA7 sin PWM;
- el volante no debe quedar frenado eléctricamente por el firmware;
- no debe aparecer `DEGRADED 40%` por el centrado.

Si PC12, PC4, PA6 y PA7 están a cero pero el volante continúa frenado al conectar los cables del motor, la causa está en el BTS7960/IBT-2 o su cableado: `R_EN/L_EN` fijados a 5 V, entradas PWM cruzadas, puente dañado o una conexión que aplica frenado dinámico.

## Prueba B — Tracción con ruedas elevadas

1. Confirmar que la secuencia de tracción llega a `power_ready`.
2. Seleccionar avance y aplicar pedal progresivamente, empezando por 5–10 %.
3. Observar CAN 0x317, el DTC activo, CH0–CH3, PWM final y velocidad de rueda.

Resultado obligatorio:

- las ruedas pueden girar con corriente inferior al antiguo umbral de 1,5 A sin DTC 16;
- nunca debe aparecer `RELAY OPEN` como error activo cuando existe movimiento bajo PWM;
- si CH0–CH3 permanecen realmente en 0,00 A mientras las ruedas giran, el diagnóstico debe indicar `CURRENT SENSE INVALID` y la tracción no debe caer a DEGRADED L3 por ese dato;
- no debe aparecer `MOTION_INHIBIT_POWER_NOT_READY` una vez acabados los 50 ms del secuenciador.

## Prueba C — Verificación eléctrica de CH0–CH3

Con PWM bajo y una sola rueda girando:

1. Medir directamente milivoltios entre los dos bornes del shunt correspondiente.
2. Comparar el signo y magnitud con la lectura INA226.

Interpretación:

- hay mV en el shunt pero INA indica 0 A: revisar VIN+/VIN−, conexiones Kelvin, soldadura y escala;
- no hay mV y el motor gira: el shunt está puenteado o fuera del camino de corriente;
- lectura negativa con avance: VIN+ y VIN− están invertidos;
- una corriente muy pequeña con la rueda en el aire puede ser normal y ya no se interpreta como relé abierto.

Todo el consumo del motor debe atravesar el shunt. Los hilos finos VIN+/VIN− deben salir de lados opuestos del elemento resistivo, no del mismo borne.

## Prueba D — INA226 de dirección CH5

### Motor parado

Con PC12 alimentado pero PA6/PA7 sin PWM:

- CH5 puede mostrar 0,00 A;
- estado esperado: `CH5 OK`;
- no debe mostrar `PRESENT NO SHUNT`.

### Motor accionado

Durante un barrido controlado de dirección:

- con caída positiva suficiente: `CH5 OK`;
- con PWM real y caída próxima a 0 µV: `PRESENT NO SHUNT`;
- con corriente negativa significativa: `POLARITY REVERSED`.

El shunt de 1,5 mΩ usado en los motores de tracción es también el valor previsto para dirección. Trabajar a 12 V no obliga a cambiar su resistencia; deben respetarse la corriente máxima y la disipación del shunt.

## Prueba E — Sensor central PB5

PB5 está configurado como sensor activo a nivel bajo:

- sin metal: nivel alto por pull-up;
- con metal: nivel bajo;
- el paso alto→bajo debe ser detectado;
- al detectar el centro se debe neutralizar el motor, poner el encoder a cero, guardar la calibración y permitir posteriormente el relé de dirección.

## Criterios de aceptación

La PR puede salir de Draft únicamente cuando:

- compila STM32 con `make -j2` y sin warnings (`-Werror`);
- compila ESP32 con `pio run -e esp32s3`;
- pasan todos los tests host de INA226, relé, centrado, heartbeat y paridad CAN;
- pasan análisis estático e integridad;
- las pruebas A–E se completan con ruedas elevadas;
- no se reproduce un nuevo DTC 16 con ruedas girando;
- CH5 parado aparece OK;
- después de un fallo de centrado, PC12, PC4, PA6 y PA7 quedan sin mando.

## Limitación actual de la verificación remota

Durante la preparación de esta PR, GitHub Actions terminó en fallo antes de ejecutar incluso un workflow mínimo sin `checkout`. Por ello, el código y las pruebas están preparados en la rama, pero no se debe interpretar el estado rojo actual de Actions como un resultado de compilación del firmware ni declarar la PR validada hasta ejecutar la matriz en un runner operativo o localmente.
