# PCU - Power Control Unit

## Introduzione

La PCU (Power Control Unit) gestisce la distribuzione e il monitoraggio della potenza all'interno del sistema. Si occupa del controllo di vari dispositivi attraverso ADC e comunicazione CAN, oltre a implementare meccanismi di fault detection.

---

## Alimentazione

All'accensione, la PCU abilita sempre:

- **ef**
- **12V**
- **24V**

---

## Monitoraggio ADC

Il sistema di acquisizione dati (ADC) è attualmente utilizzato solo per monitoraggio. I valori acquisiti sono:

- **12V**
- **Driverless**
- **24V**
- **Pompe**
- **Ventole batteria e radiatore**
- **Embedded**
- **Attuazione sterzo**

---

## Eventi in comunicazione dall'esterno

### Protocollo CAN

La PCU gestisce diversi eventi ricevuti via CAN, a seconda del modulo di origine.

#### **Driver**

- **Brake Light**: Accensione della luce freno quando il valore del freno è >= 5%

#### **PCU**

##### **Mode 0**

Controllo delle pompe e ventole:

- **Pompe**
  - `pump_enable_left`
  - `pump_speed_left`
  - `pump_enable_right`
  - `pump_speed_right`
- **Ventole radiatore**
  - `fanrad_enable_left`
  - `fanrad_speed_left`
  - `fanrad_enable_right`
  - `fanrad_speed_right`
- **Ventole batteria**
  - `fanbatt_enable_left`
  - `fanbatt_speed_left`
  - `fanbatt_enable_right`
  - `fanbatt_speed_right`

##### **Mode 1**

- Controllo di `rf`: Impostazione di `rf` a ON o OFF con invio di ACK (`PcuRfAck`).

##### **Mode 2**

- **enable\_dv**
- **enable\_embedded**

---

## Eventi in Loop

La PCU esegue controlli periodici per il rilevamento di fault. Il segnale corrispondente viene impostato a ON se il fault è 1. Questo meccanismo sarà affinato durante i test.

Aggiornamento ogni **5 secondi**, con trasmissione dei dati in CAN tramite `PcuFault`.

I fault monitorati sono:

- **12V**
- **Driverless**
- **24V**
- **Pompe**
- **Ventole batteria**

