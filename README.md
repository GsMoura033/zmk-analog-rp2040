# ZMK — RP2040 + PSP analog stick

Firmware ZMK que transforma um **joystick analógico PSP 2-axis** ligado a um **RP2040** num **mouse USB** (movimento + clique).

## Hardware suportado

| Board ZMK | Placa física típica | Artefacto CI |
|---|---|---|
| `sparkfun_pro_micro_rp2040` | Pro Micro RP2040 / Elite-Pi / clones pin-compatíveis | `psp_stick_promicro` |
| `rpi_pico` | Raspberry Pi Pico | `psp_stick_pico` |

## Ligação (IMPORTANTE: 3,3 V)

O ADC do RP2040 aguenta no máximo **3,3 V**. **Não** ligues o potenciómetro do módulo a 5 V.

| Fio do módulo PSP | Pro Micro RP2040 | Raspberry Pi Pico |
|---|---|---|
| **VCC** | **3V3** (não 5V / RAW) | **3V3** |
| **GND** | GND | GND |
| **VRx** | **A0** (D18 / GPIO26 / ADC0) | **GP26** (ADC0) |
| **VRy** | **A1** (D19 / GPIO27 / ADC1) | **GP27** (ADC1) |
| **SW** (botão) | **D2** (GPIO2) | **GP2** |

Se o teu módulo *só* funcionar a 5 V, usa **divisor de tensão** (ex. 10k+10k) em VRx e VRy antes do ADC.

```
         5V (só se precisares)
          |
         [10k]
          +----> ADC (máx 3.3V)
         [10k]
          |
         GND
```

## O que faz

- Empurrar o stick → move o cursor (mouse relativo)
- Premir o stick (SW) → clique esquerdo

## Build (GitHub Actions)

1. Faz push para `main` (ou abre um PR / `workflow_dispatch`).
2. Em **Actions**, descarrega o artefact `firmware`.
3. Flash:
   - Entra em modo bootloader (BOOTSEL no Pico / botão boot no Pro Micro).
   - Copia o `.uf2` correspondente para o drive USB que aparecer.

## Ajustar sensibilidade / drift

Edita `boards/shields/psp_stick/psp_stick.overlay`:

| Propriedade | Efeito |
|---|---|
| `mv-deadzone` | Zona morta no centro (sobe se o cursor “andar sozinho”) |
| `scale-divisor` | Mais alto = cursor mais lento |
| `scale-multiplier` | Mais alto = cursor mais rápido |
| `invert` no eixo | Inverte X ou Y |
| `mv-mid` | Centro em mV (~1650 com rail 3,3 V) |

### Console USB (debug do ADC)

Os builds CI usam o snippet `zmk-usb-logging`. Depois de flashar:

1. Liga o stick **já com o joystick ligado** (VCC→3V3).
2. No Windows, em *Gerenciador de Dispositivos → Portas (COM e LPT)*, anota o novo **COMx** do PSP Stick.
3. Abre esse COM no **PuTTY** (Serial, 115200) ou Monitor Serial.
4. Procura linhas `AIN… raw: … mv: …` — isso é o valor real para afinar `mv-mid`.

Depois de editar: commit + push → novo build.

## Stack

- ZMK `v0.3`
- [badjeff/zmk-analog-input-driver](https://github.com/badjeff/zmk-analog-input-driver) (ADC → input events)
- `CONFIG_ZMK_POINTING` (HID mouse)

## Estrutura

```
config/                 # keymap/conf de utilizador + west.yml
boards/shields/psp_stick/
build.yaml              # alvos CI
.github/workflows/      # build ZMK
```
