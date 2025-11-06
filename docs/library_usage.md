# Guida Completa all'uso della libreria USB PD

## Panoramica

Questa libreria porta lo stack USB Power Delivery (USB PD) di NXP su piattaforme generiche. Il core gestisce la negoziazione PD e il controllo Type-C, mentre i port sotto `pd/port/` forniscono l'astrazione hardware minima per integrare il controller PTN5110 con MCU non NXP. Il documento descrive come configurare, inizializzare e utilizzare lo stack in progetti embedded basati su ESP32-S3 (ESP-IDF) o su piattaforme Arduino.

## Prerequisiti

- Conoscenza dei ruoli Type-C (Source/Sink/DRP) e dei concetti base della specifica USB PD.
- Controller TCPC NXP PTN5110 collegato via I2C e linea ALERT a un GPIO.
- Toolchain C/C++ per la piattaforma target (ESP-IDF v5.x, Arduino Core, PlatformIO o build personalizzata).
- Pull-up I2C adeguati per le linee SDA/SCL (esterni o interni laddove supportato).

## Struttura del pacchetto

- `pd/`: cuore dello stack PD (policy engine, device policy manager, gestione stato, timer, driver astratti).
- `pd/ptn5110/`: implementazione del PHY driver per PTN5110 (I2C, interrupt, registri).
- `pd/port/esp32s3/`: adattatori FreeRTOS (eventi, mutex, timer, I2C, GPIO) per ESP32-S3.
- `pd/port/arduino/`: adattatori cooperativi basati su `millis()/yield()` e `Wire` per MCU Arduino.
- `docs/`: documentazione di utilizzo (incluso questo file).
- `library.json`: manifest per l'import come libreria PlatformIO.

## Componenti principali dello stack

- **Policy Engine (`pd/usb_pd_policy.c`)**: gestisce la negoziazione PD, controlla i messaggi, implementa Source/Sink/DRP.
- **Device Policy Manager (`pd/usb_pd_interface.c`)**: crea le istanze PD, coordina i callback applicativi e i comandi DPM.
- **Driver PTN5110 (`pd/ptn5110/`)**: inizializza il TCPC, gestisce interrupt ALERT e trasferimenti I2C.
- **Adapter HAL (`pd/fsl_adapter_*.h/.c`)**: definisce l’interfaccia per I2C, GPIO, timer, mutex ed eventi; ogni port implementa queste funzioni.
- **Timer e Scheduler (`pd/usb_pd_timer.c`)**: cronometra i timeout protocollo e delega all’applicazione l’esecuzione periodica tramite `PD_TimerIsrFunction`.

## Configurazione compile-time

Le opzioni principali sono in `pd/usb_pd_config.h` (modifica o sovrascrivi via macro del compilatore):

- `PD_CONFIG_MAX_PORT`: numero massimo di istanze simultanee (default 1).
- `PD_CONFIG_PTN5110_PORT`: abilita il driver PTN5110 (impostato a 1 in questo porting).
- `PD_CONFIG_TARGET_ESP32S3` / `PD_CONFIG_TARGET_ARDUINO`: selezione del port; mantieni attivo un solo target per build.
- `PD_CONFIG_ALT_MODE_SUPPORT`, `PD_CONFIG_COMMON_TASK` e ulteriori macro consentono funzionalità opzionali (DisplayPort Alt-Mode, task condiviso, ecc.).
- Imposta i macro tramite `build_flags` (PlatformIO) o `add_compile_definitions` (CMake, ESP-IDF).

Se compili con PlatformIO puoi usare il `srcFilter` nel `library.json` o nel `platformio.ini` per includere esclusivamente i file del port richiesto.

## Workflow di inizializzazione

1. **Descrivi l’hardware PTN5110**

   - ESP32-S3: popola `pd_phy_esp32s3_config_t` con controller I2C, GPIO ALERT, timer e parametri FreeRTOS.
   - Arduino: compila `pd_phy_arduino_config_t` indicando puntatore `TwoWire`, pin SDA/SCL, pull-up, polarità ALERT.

2. **Prepara la configurazione della stack (`pd_instance_config_t`)**

   - Seleziona il ruolo Type-C (es. `kPowerConfig_SourceDefault`, `kPowerConfig_SinkOnly`, `kPowerConfig_DRPToggling`).
   - Compila le tabelle PDO (`sourceCaps`/`sinkCaps`) in formato raw a 32 bit come previsto dalla specifica.
   - Imposta `phyType = kPD_PhyPTN5110` e collega `phyConfig` alla struttura del port scelto.
   - Assegna `portConfig` a un `pd_power_port_config_t` popolato con ruoli, soglie e policy.

   ```c
   static const pd_source_pdo_t s_sourceCaps[] = {
      {
         .fixedPDO = {
            .maxCurrent = 150,            // 1.5 A (10 mA units)
            .voltage = 100,               // 5.0 V (50 mV units)
            .peakCurrent = 0,
            .unchunkedSupported = 0,
            .dualRoleData = 1,
            .usbCommunicationsCapable = 1,
            .externalPowered = 0,
            .usbSuspendSupported = 0,
            .dualRolePower = 1,
            .fixedSupply = kPDO_Fixed,
         },
      },
   };

   static pd_power_port_config_t s_portConfig = {
      .sourceCaps = (uint32_t *)s_sourceCaps,
      .sinkCaps = NULL,
      .sourceCapCount = sizeof(s_sourceCaps) / sizeof(s_sourceCaps[0]),
      .sinkCapCount = 0,
      .typecRole = kPowerConfig_SourceDefault,
      .typecSrcCurrent = kCurrent_StdUSB,
      .drpTryFunction = kTypecTry_None,
      .dataFunction = kDataConfig_DFP,
      .vconnSupported = 1,
   };

   static pd_phy_esp32s3_config_t s_phyCfg = {
      .base = {
         .i2cInstance = kInterface_i2c0,
         .slaveAddress = 0x50,
         .i2cSrcClock = 40U * 1000U * 1000U,
         .i2cReleaseBus = NULL,
         .alertPort = 0,
         .alertPin = 10,
         .alertPriority = 5,
      },
      .sdaPin = GPIO_NUM_8,
      .sclPin = GPIO_NUM_9,
      .i2cBusSpeed_Hz = 400000,
      .enableInternalPullup = true,
      .alertInterruptType = GPIO_INTR_NEGEDGE,
      .alertPullMode = kHAL_GpioPullUp,
      .alertIsrFlags = ESP_INTR_FLAG_IRAM,
   };

   static pd_instance_config_t s_pdConfig = {
      .deviceType = kDeviceType_NormalPowerPort,
      .phyType = kPD_PhyPTN5110,
      .phyConfig = &s_phyCfg.base,
      .portConfig = &s_portConfig,
   };
   ```

3. **Implementa il callback applicativo**

   - Scrivi una funzione `pd_stack_callback_t` per gestire gli eventi `pd_dpm_callback_event_t` (hard reset, request PDO, role swap...).
   - Implementa (facoltativo) le funzioni `pd_power_handle_callback_t` per controllare VBUS/VCONN e misurazioni correnti.

4. **Crea l’istanza e registra i port hook**

   ```c
   static pd_handle s_pdHandle;
   pd_status_t status = PD_InstanceInit(&s_pdHandle,
                                        App_PdCallback,
                                        &appPowerCallbacks,
                                        &appContext,
                                        &s_pdConfig);
   if (status != kStatus_PD_Success)
   {
       // Gestisci l'errore di inizializzazione
   }

   // Registra l'istanza presso il port attivo
   PD_PortEsp32S3_RegisterInstance((pd_instance_t *)s_pdHandle);
   // PD_PortArduino_RegisterInstance(...) se stai compilando per Arduino
   ```

5. **Avvia il loop di servizio**

   - ESP32-S3: crea un task FreeRTOS che chiama `PD_InstanceTask(s_pdHandle);` in ciclo e delega a `vTaskDelay` un piccolo intervallo se necessario.
   - Arduino: invoca `PD_PortArduino_TaskTick();` e `PD_InstanceTask(s_pdHandle);` nel `loop()` principale, evitando long blocking delay fuori dalla libreria.

6. **Timer**

   Assicurati che il port chiami periodicamente `PD_TimerIsrFunction(pdInstance);` (implementato all’interno degli adattatori) per mantenere attivi i timeout protocollo.

## Callback ed eventi

`App_PdCallback` riceve eventi `pd_dpm_callback_event_t`. Alcuni eventi comuni:

| Evento | Quando avviene | Azione tipica |
|--------|----------------|----------------|
| `PD_DPM_SNK_RECEIVE_PARTNER_SRC_CAP` | Il device riceve le capacità sorgente dal partner | Valuta i PDO e invia `PD_DPM_CONTROL_REQUEST_NEW_POWER` con la tua richiesta. |
| `PD_DPM_SRC_RDO_REQUEST` | Un sink chiede potenza alla tua sorgente | Valida l'RDO, configura l’alimentazione e rispondi con `PD_Command(pdHandle, PD_DPM_CONTROL_POWER_ROLE_SWAP, ...)` se necessario. |
| `PD_DPM_PR_SWAP_REQUEST` | Il partner chiede un power role swap | Decidi se accettare (ritorna `kStatus_PD_Success`) o rifiutare (`kStatus_PD_Error`). |
| `PD_DPM_SNK_HARD_RESET_REQUEST` / `PD_DPM_SRC_HARD_RESET_REQUEST` | È richiesto un hard reset lato sink/source | Spegni/riaccendi VBUS, ripristina stato applicativo e conferma l'esito. |
| `PD_DPM_STRUCTURED_VDM_REQUEST` | Arriva un VDM strutturato (es. per Alt-Mode) | Analizza comando/modalità e fornisci la risposta appropriata (ACK/NAK/BUSY). |

Il valore restituito dal callback (`kStatus_PD_Success` / `kStatus_PD_Error`) indica allo stack se l'operazione è stata gestita correttamente.

## Comandi DPM e controllo runtime

- `PD_Command(pdHandle, comando, parametri)` avvia operazioni PD attive (es. role swap, richiesta potenza, soft/hard reset).
- `PD_Control(pdHandle, controllo, buffer)` interroga o imposta proprietà (ruolo corrente, orientamento Type-C, corrente erogata ecc.).

Esempio: richiedere un nuovo PDO da sink dopo avere ricevuto le capacità partner:

```c
pd_rdo_t rdo = {.objectPosition = 1, .operationalCurrent = 60, .maxCurrent = 60};
if (PD_Command(s_pdHandle, PD_DPM_CONTROL_REQUEST_NEW_POWER, &rdo) != kStatus_PD_Success)
{
    // fallback o ritenta con un profilo diverso
}
```

Consulta `pd/usb_pd.h` per l’elenco completo di `pd_dpm_command_t` e `pd_control_t`.

## Timer e scheduler

- L’adapter ESP32-S3 installa un timer hardware (esp_timer) che richiama `PD_TimerIsrFunction` a 1 kHz.
- Il port Arduino deve invocare `PD_PortArduino_TaskTick()` (che a sua volta chiama `PD_TimerIsrFunction`) ad ogni iterazione di `loop()`; mantieni `loop()` rapido per evitare timeout.
- Se implementi un port personalizzato, assicurati di fornire una sorgente di tick stabile da 1 ms.

## Integrazione specifica per piattaforme

### ESP32-S3 (ESP-IDF)

- Includi i file sotto `pd/port/esp32s3/` e abilita `CONFIG_FREERTOS_HZ=1000` per avere un tick da 1 ms (consigliato).
- Inizializza la periferica I2C selezionata prima di invocare `PD_InstanceInit` o rimuovi l’auto-config nel port se già configurata altrove.
- Configura il GPIO ALERT come input con interrupt su fronte discendente; il driver si occupa di registrare l'ISR tramite `gpio_isr_handler_add`.
- Prevedi un task dedicato al PD con priorità medio-alta (es. 5) e stack >= 4 KB.
- Abilita logging via `ESP_LOGI` nello stack se vuoi tracciare la negoziazione (definisci `PD_CONFIG_DEBUG_LOG` a 1 e implementa `USB_StackDebugPrintf`).

### Arduino (core AVR/ARM)

- Richiama `Wire.begin()` nel `setup()` (il port non inizializza implicitamente la periferica).
- Se il microcontrollore non dispone di `INPUT_PULLDOWN`, configura resistenze esterne e imposta `alertPullMode = kHAL_GpioNoPull` nella struttura PHY.
- Il port supporta fino a 8 interrupt GPIO simultanei; evita di installare più istanze di stack su schede con risorse limitate.
- Inserisci `PD_PortArduino_TaskTick();` e `PD_InstanceTask(s_pdHandle);` in cima al `loop()` e limita qualsiasi altra elaborazione a slice brevi (<1 ms).
- Per loggare eventi PD collega `Serial.begin()` e stampa dal callback applicativo.

## Diagnostica e debug

- Abilita il logging definendo `USB_PD_CONFIG_LOG_LEVEL` o le macro di debug in `usb_pd_config.h`.
- Usa `PD_Control(pdHandle, PD_CONTROL_GET_PD_STATE, &state)` per interrogare lo stato corrente della state-machine.
- In caso di timeout frequenti, verifica che `PD_TimerIsrFunction` venga eseguita ogni 1 ms e che l'I2C risponda entro 1 ms.
- Per problemi di alimentazione, controlla i callback nella struttura `pd_power_handle_callback_t` (VBUS on/off, misura corrente, VCONN).
- Il driver PTN5110 espone `PDPTN5110_Control(pdInstance, PD_PHY_GET_FAULT_STATUS, ...)` per leggere lo stato del TCPC.

## Build con PlatformIO

- Posiziona la libreria in `lib/usbpd-stack` (submodule o copia locale).
- Assicurati che `library.json` sia presente nella root della libreria.
- Imposta nel tuo `platformio.ini`:

  ```ini
  [env:esp32s3]
  platform = espressif32
  framework = espidf
  board = esp32s3dev
  build_flags = -DPD_CONFIG_TARGET_ESP32S3=1
  lib_deps = file://lib/usbpd-stack

  [env:arduino]
  platform = atmelavr
  framework = arduino
  board = uno
  build_flags = -DPD_CONFIG_TARGET_ARDUINO=1
  lib_deps = file://lib/usbpd-stack
  build_src_filter = +<*> -<pd/port/esp32s3/*>
  ```

- Aggiungi al progetto le dipendenze hardware (es. driver I2C e HAL della tua board) e configura le variabili di ambiente per eventuali file aggiuntivi.

## Risorse aggiuntive

- `README.md`: panoramica generale e note sull’integrazione PlatformIO.
- `pd/usb_pd_config.h`: riferimento completo delle macro di configurazione.
- `pd/usb_pd_policy.c` e `pd/usb_pd_phy.h`: approfondimenti su policy engine e driver TCPC.
- `pd/compliance_test_report/`: output Ellisys utilizzabile come riferimento durante debug di interoperabilità.

## Domande frequenti (FAQ)

- **Posso usare un altro TCPC?** Non senza scrivere un nuovo driver in `pd/ptn5110/` e adattare le API `pd_phy_api_interface_t`.
- **Come gestisco più porte PD?** Imposta `PD_CONFIG_MAX_PORT > 1`, crea più istanze `pd_instance_t` e assicurati che ogni port abbia il proprio alert GPIO e bus I2C.
- **Posso eseguire la stack senza RTOS?** Sì, usando il port Arduino o creando un adapter cooperativo personalizzato che fornisca mutex/eventi no-op e un tick da 1 ms.
