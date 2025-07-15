# Rimappatura Assi Joystick DualShock 4

Il nodo `dualshock_ps4` ora supporta la rimappatura dei valori degli assi del joystick tramite parametri ROS2.

## Parametri Disponibili

### Rimappatura per Asse (0-5)
- `axis_X_min`: Valore minimo di input per l'asse X (default: -1.0)
- `axis_X_max`: Valore massimo di input per l'asse X (default: 1.0)

Dove X può essere:
- **0**: Stick sinistro X (Roll)
- **1**: Stick sinistro Y (Pitch)
- **2**: L2 gear (trigger sinistro)
- **3**: Stick destro X (Yaw)
- **4**: Stick destro Y (Thrust)
- **5**: R2 gear (trigger destro)

### Valori di Output
- `output_min`: Valore minimo di output dopo la rimappatura (default: -1.0)
- `output_max`: Valore massimo di output dopo la rimappatura (default: 1.0)

## Esempi di Configurazione

### Esempio 1: Rimappare stick destro Y (Thrust) da [0.0, 1.0] a [-1.0, 1.0]
```yaml
axis_4_min: 0.0      # Thrust stick minimo
axis_4_max: 1.0      # Thrust stick massimo  
output_min: -1.0     # Output minimo
output_max: 1.0      # Output massimo
```

### Esempio 2: Invertire stick sinistro Y (Pitch)
```yaml
axis_1_min: 1.0      # Valore max come min (inversione)
axis_1_max: -1.0     # Valore min come max (inversione)
output_min: -1.0     # Output minimo
output_max: 1.0      # Output massimo
```

### Esempio 3: Aumentare sensibilità stick destro X (Yaw)
```yaml
axis_3_min: -0.5     # Range di input ridotto
axis_3_max: 0.5      # Range di input ridotto
output_min: -1.0     # Output completo
output_max: 1.0      # Output completo
```

### Esempio 4: Trigger L2/R2 da analogico a digitale
```yaml
axis_2_min: -1.0     # L2 completamente rilasciato
axis_2_max: -0.5     # L2 premuto a metà
output_min: 0.0      # Output minimo 
output_max: 1.0      # Output massimo

axis_5_min: -1.0     # R2 completamente rilasciato  
axis_5_max: -0.5     # R2 premuto a metà
output_min: 0.0      # Output minimo
output_max: 1.0      # Output massimo
```

## Utilizzo nel Launch File

```python
Node(
    package='joy_manager_ros2',
    executable='dualshock_ps4',
    name='dualshock4_joystick_controller',
    parameters=[{
        # Configurazione topic
        'rpyt_topic': 'joystick_dualshock4/rpyt',
        
        # Rimappatura assi personalizzata
        'axis_0_min': -1.0,    # Roll min
        'axis_0_max': 1.0,     # Roll max
        'axis_1_min': -1.0,    # Pitch min  
        'axis_1_max': 1.0,     # Pitch max
        'axis_2_min': -1.0,    # L2 gear min
        'axis_2_max': 1.0,     # L2 gear max
        'axis_3_min': -1.0,    # Yaw min
        'axis_3_max': 1.0,     # Yaw max
        'axis_4_min': 0.0,     # Thrust min (esempio: solo valori positivi)
        'axis_4_max': 1.0,     # Thrust max
        'axis_5_min': -1.0,    # R2 gear min
        'axis_5_max': 1.0,     # R2 gear max
        
        # Range di output
        'output_min': -1.0,
        'output_max': 1.0
    }]
)
```

## Come Funziona

1. **Input Clamping**: Il valore dell'asse viene limitato al range [axis_X_min, axis_X_max]
2. **Normalizzazione**: Il valore viene normalizzato nel range [0, 1]
3. **Rimappatura**: Il valore normalizzato viene rimappato nel range [output_min, output_max]

La formula utilizzata è:
```
output = output_min + normalized * (output_max - output_min)
dove normalized = (input_clamped - axis_min) / (axis_max - axis_min)
```

## Note
- Se `axis_min == axis_max`, l'output sarà sempre `output_min`
- I valori fuori dal range [axis_min, axis_max] vengono automaticamente limitati
- La rimappatura è applicata agli assi RPYT e ai gear L2/R2
- I pulsanti non sono interessati dalla rimappatura
